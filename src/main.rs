// SPDX-License-Identifier: Apache-2.0
//! `vyges-cts <job.json>`: replays a job's steps — the commands the reference ran before
//! `clock_tree_synthesis` (`read_db`, `read_liberty`, `set_wire_rc`, `set_dont_use`,
//! `set_cts_config`, `set_debug_level`, `read_sdc`), then the command — and prints the run's
//! output lines as JSON: `{"lines": [...], "stop": null | "error CTS-0074" | "refused: …",
//! "outcome": "no_clock_nets" | "discovered"}`.

use serde_json::{json, Value};
use std::collections::{BTreeSet, HashMap, HashSet};
use vyges_cts::buffers::BufferLibs;
use vyges_cts::discovery::{ClockDesign, ITerm, Pin, SdcClock};
use vyges_cts::log::{Log, Res, Stop};
use vyges_cts::options::Options;
use vyges_cts::run::{Design, Outcome};
use vyges_opendb::Db;
use vyges_sta::liberty::{Cell, Library};

thread_local! {
    static LUT: vyges_stt::flute::lut::Lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
}

/// Every LEF master: its site and the IMPLANT layers among its obstructions (the resizer's view).
fn rsz_masters(db: &Db) -> Res<std::collections::BTreeMap<String, vyges_rsz::preamble::Master>> {
    let e = |e: vyges_opendb::Error| Stop::Refused(format!("database: {e}"));
    let mut out = std::collections::BTreeMap::new();
    let mut layer_type: std::collections::BTreeMap<i64, (String, String)> = std::collections::BTreeMap::new();
    for i in 0..db.num_masters().map_err(e)? {
        let name = db.nth_master_name(i).map_err(e)?;
        let mut implant_obs = Vec::new();
        for (layer, ..) in db.master_obstruction_boxes(&name).map_err(e)? {
            let (lname, ltype) = layer_type.entry(layer).or_insert_with(|| {
                let lname = db.layer_name_by_number(layer);
                let ltype = db.layer_get_type(&lname).unwrap_or_default();
                (lname, ltype)
            });
            if ltype.eq_ignore_ascii_case("IMPLANT") && !implant_obs.contains(lname) {
                implant_obs.push(lname.clone());
            }
        }
        let (width, height) = (db.master_get_width(&name), db.master_get_height(&name));
        let area = i64::from(width) * i64::from(height);
        out.insert(name.clone(), vyges_rsz::preamble::Master { site: db.master_get_site(&name), area, width, height, is_core: db.master_is_core(&name), logic_std: db.master_get_type(&name).map_err(e)? == "CORE", implant_obs });
    }
    Ok(out)
}

/// A `get_lib_cells` glob: `*` and `?`.
fn glob_match(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut pi, mut ti, mut star, mut mark) = (0usize, 0usize, None::<usize>, 0usize);
    while ti < t.len() {
        if pi < p.len() && (p[pi] == '?' || p[pi] == t[ti]) {
            pi += 1;
            ti += 1;
        } else if pi < p.len() && p[pi] == '*' {
            star = Some(pi);
            mark = ti;
            pi += 1;
        } else if let Some(sp) = star {
            pi = sp + 1;
            mark += 1;
            ti = mark;
        } else {
            return false;
        }
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

fn split_iterm(s: &str) -> ITerm {
    let (i, p) = s.rsplit_once('/').unwrap_or(("", s));
    (i.to_string(), p.to_string())
}

fn some(s: String) -> Option<String> {
    (!s.is_empty()).then_some(s)
}

struct CliDesign {
    db: Db,
    libs: Vec<Library>,
    rc: vyges_est::rc::Rc,
    tech: String,
    clocks: Vec<SdcClock>,
    dont_use: BTreeSet<String>,
    /// `set_dont_use` alone (the resizer adds the liberty's own).
    set_dont_use: BTreeSet<String>,
    /// `set_routing_alpha` was given (not modelled for the Steiner trees).
    routing_alpha_set: bool,
    /// set_cts_config's clock buffer string or footprint (findBuffers' clock test reads it).
    clock_pattern: vyges_rsz::preamble::ClockPattern,
    masters: HashSet<String>,
    /// `dbNet` ids by name (the block's net order).
    net_ids: HashMap<String, u32>,
    bterms: HashSet<String>,
    /// The SDC has `set_max_fanout` on something other than the design (not modelled).
    sdc_max_fanout: bool,
    /// `set_max_fanout V [current_design]` (`Sdc::fanoutLimit(top cell)`).
    design_max_fanout: Option<f32>,
    /// `set_max_fanout` on a cell (`Sdc::setFanoutLimit(Cell*)`): the last value given.
    cell_max_fanout: HashMap<String, f32>,
    /// The liberty clock data the estimator and the constant propagation read.
    liberty: vyges_est::liberty::LibertyClocks,
    /// The constraints as read.
    sdc: Option<vyges_loom::sdc::Sdc>,
    /// The clock arrivals on the written design (timed when first asked): per pin, each max path of
    /// a rising clock edge in the search's order, with the net its clock path starts on.
    clock_arrivals: Option<HashMap<String, Vec<(f32, String)>>>,
}

impl CliDesign {
    fn refresh(&mut self) {
        self.masters = (0..self.db.num_masters().unwrap_or(0)).filter_map(|i| self.db.nth_master_name(i).ok()).collect();
        self.net_ids = self.db.block_get_nets().into_iter().enumerate().map(|(i, n)| (n, i as u32)).collect();
        self.bterms = self.db.block_get_b_terms().into_iter().collect();
    }
}

impl BufferLibs for CliDesign {
    fn libraries(&self) -> &[Library] {
        &self.libs
    }
    fn has_master(&self, name: &str) -> bool {
        self.masters.contains(name)
    }
    fn liberty_cell(&self, name: &str) -> Option<&Cell> {
        self.libs.iter().find_map(|l| l.cells.get(name))
    }
    fn lef_clock_input(&self, master: &str) -> Option<String> {
        self.db.master_get_m_terms(master).into_iter().find(|t| self.db.mterm_get_io_type(master, t) == "INPUT" && self.db.mterm_get_sig_type(master, t) == "CLOCK")
    }
    fn resizer_dont_use(&self, cell: &str) -> bool {
        self.dont_use.contains(cell)
    }
}

impl ClockDesign for CliDesign {
    fn net_id(&self, net: &str) -> u32 {
        self.net_ids.get(net).copied().unwrap_or(u32::MAX)
    }
    fn net_iterms(&self, net: &str) -> Vec<ITerm> {
        self.db.net_get_i_terms(net).iter().map(|s| split_iterm(s)).collect()
    }
    fn net_bterms(&self, net: &str) -> Vec<String> {
        self.db.net_get_b_terms(net)
    }
    fn net_is_special(&self, net: &str) -> bool {
        self.db.net_is_special(net)
    }
    fn net_first_output(&self, net: &str) -> Option<ITerm> {
        some(self.db.net_get_first_output(net)).map(|s| split_iterm(&s))
    }
    fn net_first_bterm(&self, net: &str) -> Option<String> {
        some(self.db.net_get1st_b_term(net))
    }
    fn iterm_is_input_signal(&self, t: &ITerm, io: bool) -> bool {
        self.db.iterm_is_input_signal(&t.0, &t.1, io)
    }
    fn iterm_is_output_signal(&self, t: &ITerm, _io: bool) -> bool {
        self.db.iterm_is_output_signal(&t.0, &t.1)
    }
    fn iterm_net(&self, t: &ITerm) -> Option<String> {
        some(self.db.iterm_get_net(&t.0, &t.1))
    }
    fn bterm_net(&self, port: &str) -> Option<String> {
        some(self.db.bterm_get_net(port))
    }
    fn bterm_is_input(&self, port: &str) -> bool {
        self.db.bterm_get_io_type(port) == "INPUT"
    }
    fn bterm_is_inout(&self, port: &str) -> bool {
        self.db.bterm_get_io_type(port) == "INOUT"
    }
    fn inst_iterms(&self, inst: &str) -> Vec<ITerm> {
        self.db.inst_get_i_terms(inst).iter().map(|s| split_iterm(s)).collect()
    }
    fn inst_master(&self, inst: &str) -> String {
        self.db.inst_get_master(inst)
    }
    fn inst_is_block(&self, inst: &str) -> bool {
        self.db.inst_is_block(inst)
    }
    fn inst_is_placed(&self, inst: &str) -> bool {
        self.db.inst_is_placed(inst)
    }
    fn inst_is_fixed(&self, inst: &str) -> bool {
        self.db.inst_is_fixed(inst)
    }
    fn inst_is_pad(&self, inst: &str) -> bool {
        self.db.inst_is_pad(inst)
    }
    fn inst_is_timing_source(&self, inst: &str) -> bool {
        self.db.inst_get_source_type(inst) == "TIMING"
    }
    fn liberty_cell(&self, master: &str) -> Option<&Cell> {
        BufferLibs::liberty_cell(self, master)
    }
    fn iterm_avg_xy(&self, t: &ITerm) -> Option<(i32, i32)> {
        self.db.iterm_avg_xy(&t.0, &t.1)
    }
    fn inst_locations(&self) -> Vec<(i32, i32)> {
        self.db.block_get_insts().iter().map(|i| (self.db.inst_get_location_x(i), self.db.inst_get_location_y(i))).collect()
    }
    fn has_hierarchy(&self) -> bool {
        !self.db.block_get_mod_insts().is_empty()
    }
    fn steiner_tree(&self, net: &str, xs: &[i32], ys: &[i32], drvr: usize) -> Res<vyges_stt::Tree> {
        if self.routing_alpha_set {
            return vyges_cts::log::refuse("set_routing_alpha: the Steiner builder's alpha is not modelled");
        }
        // The builder's default alpha.
        LUT.with(|lut| vyges_stt::make_steiner_tree(lut, xs, ys, drvr, 0.3).0).ok_or_else(|| Stop::Refused(format!("net {net}: no Steiner tree")))
    }
    fn make_clock_net(&mut self, name: &str) -> Res<String> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("clock net {name}: {e}"));
        let name = self.db.make_new_net_name(name, "IF_NEEDED").map_err(e)?;
        self.db.create_net(&name).map_err(e)?;
        self.db.net_set_sig_type(&name, "CLOCK").map_err(e)?;
        self.refresh();
        Ok(name)
    }
    fn make_clone(&mut self, gater: &str, name: &str, net: &str) -> Res<()> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("clone {name}: {e}"));
        let master = self.db.inst_get_master(gater);
        self.db.create_inst(&master, name).map_err(e)?;
        self.db.inst_set_source_type(name, "TIMING").map_err(e)?;
        for t in self.inst_iterms(name) {
            if self.iterm_is_input_signal(&t, true) {
                if let Some(n) = some(self.db.iterm_get_net(gater, &t.1)) {
                    self.db.connect(&t.0, &t.1, &n).map_err(e)?;
                }
            } else if self.iterm_is_output_signal(&t, true) {
                self.db.connect(&t.0, &t.1, net).map_err(e)?;
            }
        }
        self.refresh();
        Ok(())
    }
    fn reconnect(&mut self, t: &ITerm, net: &str) -> Res<()> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("{}/{}: {e}", t.0, t.1));
        self.db.disconnect(&t.0, &t.1).map_err(e)?;
        self.db.connect(&t.0, &t.1, net).map_err(e)
    }
    fn place(&mut self, inst: &str, at: (i32, i32)) -> Res<()> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("{inst}: {e}"));
        self.db.set_inst_location(inst, at.0, at.1).map_err(e)?;
        self.db.inst_set_placement_status(inst, "PLACED").map_err(e)
    }
    fn fork_register_net(&mut self, net: &str, sinks: &[ITerm], root_buffer: &str, top_buffer: &str) -> Res<String> {
        if !self.db.block_get_mod_insts().is_empty() {
            return vyges_cts::log::refuse("the register tree's buffer in a hierarchical design (its target module) is not modelled");
        }
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("forkRegisterClockNetwork: {e}"));
        let second = format!("{net}_regs");
        self.db.create_net(&second).map_err(e)?;
        self.db.net_set_sig_type(&second, "CLOCK").map_err(e)?;
        for (inst, pin) in sinks {
            self.db.disconnect(inst, pin).map_err(e)?;
            self.db.connect(inst, pin, &second).map_err(e)?;
        }
        self.db.create_inst(root_buffer, top_buffer).map_err(e)?;
        // The new net's terminal bounding box, before the buffer joins it.
        let x = (self.db.net_get_term_b_box_x_min(&second) + self.db.net_get_term_b_box_x_max(&second)) / 2;
        let y = (self.db.net_get_term_b_box_y_min(&second) + self.db.net_get_term_b_box_y_max(&second)) / 2;
        self.db.inst_set_source_type(top_buffer, "TIMING").map_err(e)?;
        self.db.set_inst_location(top_buffer, x, y).map_err(e)?;
        self.db.inst_set_placement_status(top_buffer, "PLACED").map_err(e)?;
        let terms = self.inst_iterms(top_buffer);
        let input = terms.iter().find(|t| self.iterm_is_input_signal(t, true)).cloned();
        let output = terms.iter().find(|t| self.iterm_is_output_signal(t, true)).cloned();
        let (Some(i), Some(o)) = (input, output) else { return vyges_cts::log::refuse(format!("{root_buffer}: no input or output terminal")) };
        self.db.connect(&i.0, &i.1, net).map_err(e)?;
        self.db.connect(&o.0, &o.1, &second).map_err(e)?;
        self.refresh();
        Ok(second)
    }
}

impl vyges_cts::write::TreeDb for CliDesign {
    fn iterm_io_type(&self, t: &ITerm) -> String {
        self.db.iterm_get_io_type(&t.0, &t.1)
    }
    fn inst_first_output(&self, inst: &str) -> Option<ITerm> {
        // odb names the terminal `inst/pin`.
        some(self.db.inst_get_first_output(inst)).map(|n| (inst.to_string(), n.strip_prefix(&format!("{inst}/")).map(String::from).unwrap_or(n)))
    }
    fn net_sig_type(&self, net: &str) -> String {
        self.db.net_get_sig_type(net)
    }
    fn net_iterm_count(&self, net: &str) -> u32 {
        self.db.net_get_i_term_count(net)
    }
    fn net_input_port_xy(&self, net: &str) -> Option<(i32, i32)> {
        let mut at = None;
        for b in self.db.net_get_b_terms(net) {
            if self.db.bterm_get_io_type(&b) != "INPUT" {
                continue;
            }
            for i in 0..self.db.num_bterm_get_b_pins(&b) {
                let s = self.db.bpin_get_placement_status(&b, i);
                if s == "NONE" || s == "UNPLACED" {
                    continue;
                }
                if let Some(&(_, x0, y0, _, _)) = self.db.bpin_layer_boxes(&b, i).ok().as_ref().and_then(|v| v.first()) {
                    at = Some((x0, y0));
                }
                break;
            }
        }
        at
    }
    fn connect(&mut self, t: &ITerm, net: &str) -> Res<()> {
        self.db.connect(&t.0, &t.1, net).map_err(|e| Stop::Refused(format!("connect {}/{} {net}: {e}", t.0, t.1)))
    }
    fn disconnect(&mut self, t: &ITerm) -> Res<()> {
        self.db.disconnect(&t.0, &t.1).map_err(|e| Stop::Refused(format!("disconnect {}/{}: {e}", t.0, t.1)))
    }
    fn create_placed_inst(&mut self, master: &str, name: &str, x: i32, y: i32) -> Res<()> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("create {name}: {e}"));
        self.db.create_inst(master, name).map_err(e)?;
        self.db.inst_set_source_type(name, "TIMING").map_err(e)?;
        self.db.set_inst_location(name, x, y).map_err(e)?;
        self.db.inst_set_placement_status(name, "PLACED").map_err(e)
    }
    fn create_clock_net(&mut self, name: &str) -> Res<()> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("create net {name}: {e}"));
        if self.net_ids.contains_key(name) {
            return vyges_cts::log::refuse(format!("net {name} already exists (the reference's create would fail)"));
        }
        self.db.create_net(name).map_err(e)?;
        self.db.net_set_sig_type(name, "CLOCK").map_err(e)?;
        self.net_ids.insert(name.to_string(), self.net_ids.len() as u32);
        Ok(())
    }
    fn set_net_clock(&mut self, net: &str) -> Res<()> {
        self.db.net_set_sig_type(net, "CLOCK").map_err(|e| Stop::Refused(format!("{net}: {e}")))
    }
    fn destroy_net(&mut self, net: &str) -> Res<()> {
        self.db.destroy_net(net).map_err(|e| Stop::Refused(format!("destroy {net}: {e}")))?;
        self.net_ids.remove(net);
        Ok(())
    }
    fn destroy_inst(&mut self, inst: &str) -> Res<()> {
        self.db.destroy_inst(inst).map_err(|e| Stop::Refused(format!("destroy {inst}: {e}")))
    }
    fn resizer_dont_use_cell(&self, cell: &str) -> bool {
        self.dont_use.contains(cell)
    }
    fn iterm_avg_xy_logged(&mut self, t: &ITerm) -> (Option<(i32, i32)>, Vec<String>) {
        let (at, text) = self.db.with_captured_logs(|db| db.iterm_avg_xy(&t.0, &t.1));
        (at, text.lines().map(String::from).filter(|l| !l.is_empty()).collect())
    }
    fn liberty_libraries(&self) -> &[Library] {
        &self.libs
    }
    fn routing_layers(&self) -> Vec<(String, i32, i32, bool)> {
        let mut layers: Vec<(i32, String)> = self.db.tech_get_layers().into_iter().map(|l| (self.db.layer_get_routing_level(&l), l)).filter(|(lv, _)| *lv > 0).collect();
        layers.sort_by_key(|(lv, _)| *lv);
        layers.into_iter().map(|(_, l)| (l.clone(), self.db.layer_get_width(&l) as i32, self.db.layer_get_pitch(&l), self.db.layer_is_backside(&l))).collect()
    }
    fn create_ndr(&mut self, name: &str) -> Res<bool> {
        self.db.ndr_create(name).map_err(|e| Stop::Refused(format!("NDR {name}: {e}")))
    }
    fn set_ndr_layer(&mut self, ndr: &str, layer: &str, width: i32, spacing: i32) -> Res<()> {
        let e = |e: vyges_opendb::Error| Stop::Refused(format!("NDR {ndr} {layer}: {e}"));
        if !self.db.ndr_set_layer_width(ndr, layer, width).map_err(e)? || !self.db.ndr_set_layer_spacing(ndr, layer, spacing).map_err(e)? {
            return vyges_cts::log::refuse(format!("NDR {ndr}: no layer {layer}"));
        }
        Ok(())
    }
    fn set_net_ndr(&mut self, net: &str, ndr: &str) -> Res<()> {
        self.db.net_set_non_default_rule(net, ndr).map_err(|e| Stop::Refused(format!("{net} NDR {ndr}: {e}")))
    }
    fn has_master_cell(&self, name: &str) -> bool {
        self.masters.contains(name)
    }
    fn sink_clock_arrival(&mut self, t: &ITerm, top_net: &str) -> Res<Option<f32>> {
        if self.clock_arrivals.is_none() {
            let table = self.clock_arrival_table().map_err(Stop::Refused)?;
            self.clock_arrivals = Some(table);
        }
        let pin = format!("{}/{}", t.0, t.1);
        Ok(self.clock_arrivals.as_ref().expect("timed").get(&pin).and_then(|v| v.iter().find(|(_, net)| net == top_net).map(|(a, _)| *a)))
    }
    fn net_first_bterm_center(&self, net: &str) -> Option<(i32, i32)> {
        let b = self.db.net_get_b_terms(net).into_iter().next()?;
        Some(((self.db.bterm_get_b_box_x_min(&b) + self.db.bterm_get_b_box_x_max(&b)) / 2, (self.db.bterm_get_b_box_y_min(&b) + self.db.bterm_get_b_box_y_max(&b)) / 2))
    }
    fn insert_buffer_before_loads(&mut self, net: &str, loads: &[ITerm], master: &str, loc: (i32, i32), buf_name: &str, net_name: &str) -> Res<String> {
        let name = self.db.insert_buffer_before_loads(Some(net), loads, &[], master, Some(loc), buf_name, Some(net_name), "IF_NEEDED", true).map_err(|e| Stop::Refused(format!("insertBufferBeforeLoads {buf_name}: {e}")))?;
        self.refresh();
        Ok(name)
    }
}

impl CliDesign {
    /// `setAllClocksPropagated` then the latency balancer's `initSta`: the written design timed
    /// with every non-virtual clock propagated, on placement parasitics estimated now; per pin,
    /// each max path of a rising clock edge (in the search's order) and the net its clock path
    /// starts on.
    fn clock_arrival_table(&mut self) -> Result<HashMap<String, Vec<(f32, String)>>, String> {
        use vyges_rsz::env::{master_pins, placement_parasitics_at, port_caps_at, sdc_env, search_sdc, sim_constants};
        if self.routing_alpha_set {
            return Err("set_routing_alpha before the latency balancer's parasitics: not modelled".into());
        }
        let libs = vyges_rsz::preamble::Libs { libs: self.libs.clone(), scenes: Vec::new(), scene_libs: Vec::new(), link_scene: None };
        let netlist = vyges_grt::timer::netlist(&self.db);
        let mut s = self.sdc.take().ok_or("the latency balancer without constraints: not modelled")?;
        // Every non-virtual clock propagated, whatever set_propagated_clock named.
        let named = std::mem::take(&mut s.propagated);
        // `[all_inputs]` / `[all_outputs]` delays as the ports they name (the search reads ports).
        vyges_rsz::env::expand_io_delay_ports(&mut s, &netlist);
        let r = (|| -> Result<HashMap<String, Vec<(f32, String)>>, String> {
            let clock_sources: Vec<String> = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
            let mut rc = self.rc.clone();
            rc.sort_clk_and_signal_layers();
            let caps = port_caps_at(None, Some(&s), &libs, &netlist)?;
            let par = placement_parasitics_at(&self.db, &rc, &self.liberty, &clock_sources, true, 0.3, 1, &caps)?;
            let mut env = sdc_env(&s, &libs, &netlist)?;
            env.constants = sim_constants(&self.db, &self.liberty)?;
            let mpins = master_pins(&self.db)?;
            let mut g = vyges_rsz::repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
            g.find_delays(&par[0], None)?;
            let time_scale = libs.default_library().ok_or("no liberty library")?.time_scale;
            let ssdc = search_sdc(&s, time_scale, true, false)?;
            let mut search = vyges_sta::search::Search::in_graph_order(&g, &ssdc);
            search.find_arrivals()?;
            let mut out: HashMap<String, Vec<(f32, String)>> = HashMap::new();
            for (v, vx) in g.vertices.iter().enumerate() {
                for p in &search.paths[v] {
                    let (Some(edge), 1) = (p.tag.clk_edge, p.tag.mm) else { continue };
                    if edge % 2 != 0 {
                        continue;
                    }
                    // The clock path's start (`PathExpanded::startPath`) and its net.
                    let (mut sv, mut sp) = (v, p);
                    while let Some((pv, pp)) = search.prev_path(sp) {
                        (sv, sp) = (pv, pp);
                    }
                    let start = &g.vertices[sv].name;
                    let net = match start.rsplit_once('/') {
                        Some((i, pin)) => self.db.iterm_get_net(i, pin),
                        None => self.db.bterm_get_net(start),
                    };
                    out.entry(vx.name.clone()).or_default().push((p.arrival, net));
                }
            }
            Ok(out)
        })();
        s.propagated = named;
        self.sdc = Some(s);
        r
    }
}

impl vyges_cts::techchar::CharDesign for CliDesign {
    fn master_height(&self, name: &str) -> i32 {
        self.db.master_get_height(name) as i32
    }
}

impl CliDesign {
    /// `Resizer::repairClkNets` on the written design, with the constraints as read.
    fn repair_clk_nets_with(&mut self, s: &vyges_loom::sdc::Sdc, log: &mut Log, clk_buffers: &[String], max_wire_length: f64) -> Res<()> {
        use vyges_rsz::env::{master_pins, no_signal_cap, placement_parasitics_at, port_caps_at, sdc_env, sim_constants};
        use vyges_rsz::odb_design::{net_info, sdc_cap_pins, sdc_port_cap_limits, sizing_limits, CliDesign as RszDesign, TimerLog};
        let refused = |e: String| Stop::Refused(format!("repair_clock_nets: {e}"));
        if self.routing_alpha_set {
            return vyges_cts::log::refuse("set_routing_alpha before repair_clock_nets' parasitics: not modelled");
        }
        let libs = vyges_rsz::preamble::Libs { libs: self.libs.clone(), scenes: Vec::new(), scene_libs: Vec::new(), link_scene: None };
        let lib0 = libs.default_library().ok_or_else(|| Stop::Refused("no liberty library".into()))?;
        if lib0.time_scale != 1e-9f32 {
            return vyges_cts::log::refuse("a library time unit other than 1ns: not modelled");
        }
        let limits = vyges_rsz::timing::Limits {
            design_max_transition: s.max_transition.map(|v| vyges_rsz::timing::user_time_to_sta(v, lib0.time_scale)),
            design_max_fanout: s.max_fanout.map(|v| v as f32),
            design_max_capacitance: s.max_capacitance.map(|v| vyges_rsz::timing::user_cap_to_sta(v, lib0.cap_scale)),
            port_max_capacitance: sdc_port_cap_limits(s, lib0.cap_scale),
        };
        let masters = rsz_masters(&self.db)?;
        let mut dont_use = self.set_dont_use.clone();
        for lib in &self.libs {
            dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
        }
        // `setAllClocksPropagated` ran just before: every non-virtual clock is propagated.
        let clock_sources: Vec<String> = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
        let clocks: Vec<Vec<String>> = s.clocks.iter().map(|c| c.sources.clone()).collect();
        let netlist = vyges_grt::timer::netlist(&self.db);
        let mut rc = self.rc.clone();
        rc.sort_clk_and_signal_layers();
        let caps = port_caps_at(None, Some(s), &libs, &netlist).map_err(refused)?;
        let parasitics = placement_parasitics_at(&self.db, &rc, &self.liberty, &clock_sources, true, 0.3, 1, &caps).map_err(refused)?;
        let wire_rc: Vec<vyges_rsz::buffered_net::WireRc> = (0..libs.scene_count().max(1))
            .map(|k| {
                let v = rc.resolved(&self.tech, k);
                vyges_rsz::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] }
            })
            .collect();
        let constants = sim_constants(&self.db, &self.liberty).map_err(refused)?;
        let mut env = sdc_env(s, &libs, &netlist).map_err(refused)?;
        env.constants = constants.clone();
        let mpins = master_pins(&self.db).map_err(refused)?;
        let info = net_info(&self.db, &netlist).map_err(refused)?;
        let estimating = !no_signal_cap(&self.db, &rc);
        let core = self.core_area();
        let core = (core != (0, 0, 0, 0)).then_some(core);
        let port_caps = env.port_pin_cap.clone();
        let block_limits = sizing_limits(&self.db).map_err(refused)?;
        let dbu = self.db.tech_get_db_units_per_micron();
        // dbNetwork::id(pin): an instance terminal's odb id << 1, a port's << 1 | 1 (`PinSet`'s order).
        let ids: HashMap<String, u64> = {
            let mut m = HashMap::new();
            for v in &netlist.insts {
                for p in mpins.get(&v.1).into_iter().flatten() {
                    if let Ok(id) = self.db.iterm_id(&v.0, p) {
                        m.insert(format!("{}/{p}", v.0), u64::from(id) << 1);
                    }
                }
            }
            for b in self.db.block_get_b_terms() {
                if let Ok(id) = self.db.bterm_id(&b) {
                    m.insert(b, (u64::from(id) << 1) | 1);
                }
            }
            m
        };
        let liberty = self.liberty.clone();
        let mut trace = vyges_rsz::trace::Trace::default();
        let r = {
            let mut design = RszDesign { db: &mut self.db, rc: &rc, liberty: &liberty, libs: &libs, clock_sources: &clock_sources, constants, alpha: 0.3, estimating, netlist, parasitics, invalid: BTreeSet::new(), info, core, port_caps, propagated: true, journal: Vec::new(), sdc_nets: BTreeSet::new(), sdc_pins: sdc_cap_pins(s), timer: TimerLog::default(), stale_out_arcs: std::collections::BTreeMap::new(), gr: None, gr_error: None, undoing: false, skips: None, skip_log: false };
            let inputs = vyges_rsz::repair_design::Inputs { libs: &libs, masters: &masters, dont_use: &dont_use, limits, clock_sources: &clock_sources, dbu, wire_rc, sdc: env, master_pins: mpins, sizing_limits: block_limits, clock_pattern: self.clock_pattern.clone() };
            let order = |p: &str| ids.get(p).copied().unwrap_or(u64::MAX);
            vyges_rsz::repair_design::repair_clk_nets(&inputs, &mut design, clk_buffers, &clocks, &order, max_wire_length, &mut trace)
        };
        if let Ok(path) = std::env::var("VYGES_CTS_RSZ_TRACE") {
            let _ = std::fs::write(&path, trace.text());
        }
        self.refresh();
        self.clock_arrivals = None;
        match r {
            Ok(outcome) => {
                for (code, msg) in outcome.clk_summary() {
                    let id = code.trim_start_matches("RSZ-").parse().unwrap_or(0);
                    log.info("RSZ", id, msg);
                }
                Ok(())
            }
            Err(vyges_rsz::Stop::Refused { msg, .. }) => vyges_cts::log::refuse(format!("repair_clock_nets: {msg}")),
            Err(vyges_rsz::Stop::Error { code, msg }) => match code.strip_prefix("RSZ-").and_then(|n| n.parse::<u32>().ok()) {
                Some(id) => log.error("RSZ", id, msg),
                None => vyges_cts::log::refuse(format!("repair_clock_nets: {code}: {msg}")),
            },
        }
    }
}

impl Design for CliDesign {
    fn core_size(&self) -> (i32, i32) {
        (self.db.block_get_core_area_dx(), self.db.block_get_core_area_dy())
    }
    fn dbu(&self) -> i32 {
        self.db.block_get_db_units_per_micron()
    }
    fn wire_signal_cap(&self) -> f64 {
        let v = self.rc.resolved(&self.tech, 0);
        if self.rc.resolve(&self.tech, |w| &w.signal_cap).is_empty() { 0.0 } else { (v[2] + v[3]) / 2.0 }
    }
    fn wire_clk_cap(&self) -> f64 {
        let v = self.rc.resolved(&self.tech, 0);
        if self.rc.resolve(&self.tech, |w| &w.clk_cap).is_empty() { 0.0 } else { (v[6] + v[7]) / 2.0 }
    }
    fn wire_clk_res(&self) -> f64 {
        let v = self.rc.resolved(&self.tech, 0);
        if self.rc.resolve(&self.tech, |w| &w.clk_res).is_empty() { 0.0 } else { (v[4] + v[5]) / 2.0 }
    }
    fn has_library(&self, name: &str) -> bool {
        self.libs.iter().any(|l| l.name == name)
    }
    fn find_net(&self, name: &str) -> Option<String> {
        self.net_ids.contains_key(name).then(|| name.to_string())
    }
    fn sdc_clocks(&self) -> Vec<SdcClock> {
        self.clocks.clone()
    }
    fn repair_clk_nets(&mut self, log: &mut Log, clk_buffers: &[String], max_wire_length: f64) -> Res<()> {
        let s = self.sdc.take().ok_or_else(|| Stop::Refused("repair_clock_nets without constraints: not modelled".into()))?;
        let r = self.repair_clk_nets_with(&s, log, clk_buffers, max_wire_length);
        self.sdc = Some(s);
        r
    }
    fn find_max_wire_length(&self) -> Res<f64> {
        let v = self.rc.resolved(&self.tech, 0);
        let rc = vyges_rsz::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] };
        // No scene with a signal resistance: -inf, before any buffer is looked at.
        if self.rc.resolve(&self.tech, |w| &w.signal_res).is_empty() || (rc.h_res + rc.v_res) / 2.0 <= 0.0 {
            return Ok(f64::NEG_INFINITY);
        }
        let libs = vyges_rsz::preamble::Libs { libs: self.libs.clone(), scenes: Vec::new(), scene_libs: Vec::new(), link_scene: None };
        let masters = rsz_masters(&self.db)?;
        let mut dont_use = self.set_dont_use.clone();
        for lib in &self.libs {
            dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
        }
        // Clock buffers excluded: the resizer's default, which only its hold repair turns off.
        let buffers = vyges_rsz::preamble::find_buffers_with(&libs, &masters, &dont_use, true, &self.clock_pattern).map_err(|e| Stop::Refused(format!("findBuffers: {}", e.message())))?;
        let (tgt_slews, _, _) = vyges_rsz::preamble::find_target_loads(&libs, &buffers.cells, &dont_use);
        let mut master_pins = HashMap::new();
        for m in &self.masters {
            let terms = self.db.master_mterms(m).map_err(|e| Stop::Refused(format!("{m}: {e}")))?;
            master_pins.insert(m.clone(), terms.into_iter().filter(|(_, t)| t != "POWER" && t != "GROUND").map(|(n, _)| n).collect());
        }
        let ctx = vyges_rsz::max_wire_length::Ctx { libs: &libs, buffers: &buffers.cells, tgt_slews, wire_rc: std::slice::from_ref(&rc), master_pins: &master_pins };
        let mut lengths = Vec::new();
        vyges_rsz::max_wire_length::find_max_wire_length(&ctx, &mut lengths).map_err(|e| Stop::Refused(format!("findMaxWireLength: {}", e.message())))
    }
    fn iterm_shape_boxes(&self, inst: &str, pin: &str) -> Vec<(i32, i32, i32, i32)> {
        self.db.iterm_shape_boxes(inst, pin)
    }
    fn core_area(&self) -> (i32, i32, i32, i32) {
        (self.db.block_get_core_area_x_min(), self.db.block_get_core_area_y_min(), self.db.block_get_core_area_x_max(), self.db.block_get_core_area_y_max())
    }
    fn master_size(&self, name: &str) -> Option<(i32, i32)> {
        self.masters.contains(name).then(|| (self.db.master_get_width(name) as i32, self.db.master_get_height(name) as i32))
    }
    fn merge_blockages(&self, bloat_w: i32, bloat_h: i32) -> (u32, u32, Vec<vyges_cts::run::Rect>) {
        self.db.cts_merge_blockages(bloat_w, bloat_h)
    }
    fn buffer_fanout_limit(&self, buffer: &str) -> Res<i32> {
        if self.sdc_max_fanout {
            return vyges_cts::log::refuse("set_max_fanout on a port or pin (the SDC fanout limits) is not modelled");
        }
        let mut fanout = i32::MAX;
        // The design's own limit first; without an output port it is the only one read.
        if let Some(f) = self.design_max_fanout {
            fanout = fanout.min(f as i32);
        }
        // The first output terminal that is not a supply.
        let out = self.db.master_get_m_terms(buffer).into_iter().find(|t| {
            let sig = self.db.mterm_get_sig_type(buffer, t);
            sig != "POWER" && sig != "GROUND" && self.db.mterm_get_io_type(buffer, t) == "OUTPUT"
        });
        // Without an output port only the design's limit is read.
        let Some(out) = out else { return Ok(self.design_max_fanout.map_or(0, |_| fanout)) };
        if let Some(&f) = self.cell_max_fanout.get(buffer) {
            fanout = fanout.min(f as i32);
        }
        let Some((lib, cell)) = self.libs.iter().find_map(|l| l.cells.get(buffer).map(|c| (l, c))) else {
            return vyges_cts::log::refuse(format!("{buffer}: no liberty cell"));
        };
        let Some(port) = cell.ports.iter().find(|p| p.name == out) else {
            return vyges_cts::log::refuse(format!("{buffer}/{out}: no liberty port"));
        };
        if let Some(f) = port.max_fanout {
            fanout = fanout.min(f as i32);
        } else if let Some(f) = lib.default_max_fanout {
            fanout = fanout.min(f as i32);
        }
        Ok(if fanout == i32::MAX { 0 } else { fanout })
    }
}

fn read_text(path: &str) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))
}

fn run(job: &Value) -> Result<Value, String> {
    let mut d = CliDesign { db: Db::new(), libs: Vec::new(), rc: vyges_est::rc::Rc::new(), tech: String::new(), clocks: Vec::new(), dont_use: BTreeSet::new(), set_dont_use: BTreeSet::new(), routing_alpha_set: false, clock_pattern: Default::default(), masters: HashSet::new(), net_ids: HashMap::new(), bterms: HashSet::new(), sdc_max_fanout: false, design_max_fanout: None, cell_max_fanout: HashMap::new(), liberty: vyges_est::liberty::LibertyClocks::default(), sdc: None, clock_arrivals: None };
    let mut o = Options::default();
    let mut log = Log::default();
    let mut pre: Vec<String> = Vec::new();
    let units = |l: &vyges_est::liberty::LibertyClocks| -> Result<vyges_est::rc::Units, String> {
        let u = l.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(vyges_est::rc::Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    let finish = |log: Log, pre: &[String], status: &str, reason: Option<String>| json!({"tool": "vyges-cts", "status": status, "reason": reason, "before": pre, "lines": log.lines, "trace": log.trace});
    for step in job["steps"].as_array().ok_or("steps")? {
        let cmd = step["cmd"].as_str().ok_or("cmd")?;
        let args: Vec<String> = step["args"].as_array().map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default();
        let r: Res<Option<Outcome>> = (|| {
            match cmd {
                "read_db" => {
                    let path = args.last().ok_or_else(|| Stop::Refused("read_db path".into()))?;
                    d.db = Db::open(path).map_err(|e| Stop::Refused(format!("{path}: {e}")))?;
                    d.tech = d.db.tech_get_name();
                    d.refresh();
                }
                "read_liberty" => {
                    if args.iter().any(|a| a.starts_with('-')) {
                        return vyges_cts::log::refuse(format!("read_liberty {}: options are not modelled", args.join(" ")));
                    }
                    let path = args.last().ok_or_else(|| Stop::Refused("read_liberty path".into()))?;
                    let text = read_text(path).map_err(Stop::Refused)?;
                    d.liberty.read(&text).map_err(|e| Stop::Refused(format!("{path}: {e}")))?;
                    let group = vyges_sta::liberty_parse::parse(&text).map_err(|e| Stop::Refused(format!("{path}: {e}")))?;
                    d.libs.push(Library::read(&group).map_err(|e| Stop::Refused(format!("{path}: {e} (not modelled)")))?);
                }
                "define_corners" => return vyges_cts::log::refuse("define_corners: several scenes are not modelled"),
                "set_wire_rc" => {
                    d.rc.check_set_wire_rc(&d.db, &args).map_err(Stop::Refused)?;
                    d.rc.set_wire_rc(&d.db, units(&d.liberty).map_err(Stop::Refused)?, &args).map_err(Stop::Refused)?;
                }
                "set_layer_rc" => d.rc.set_layer_rc(&mut d.db, units(&d.liberty).map_err(Stop::Refused)?, &args).map_err(Stop::Refused)?,
                "set_dont_use" => {
                    for a in &args {
                        for pat in a.split_whitespace() {
                            if pat.contains(['/', '[', ']', '\\']) || (pat.starts_with('_') && pat.contains("_p_")) {
                                return vyges_cts::log::refuse(format!("set_dont_use {pat}: not a plain cell pattern (not modelled)"));
                            }
                            for lib in &d.libs {
                                let hits: Vec<String> = lib.cells.keys().filter(|c| glob_match(pat, c)).cloned().collect();
                                d.dont_use.extend(hits.iter().cloned());
                                d.set_dont_use.extend(hits);
                            }
                        }
                    }
                }
                "set_debug_level" => {
                    let [tool, group, level] = args.as_slice() else { return vyges_cts::log::refuse("set_debug_level: needs a tool, a group and a level") };
                    let level: i32 = level.parse().map_err(|_| Stop::Refused(format!("set_debug_level {level}: not an integer")))?;
                    log.debug.insert((tool.clone(), group.clone()), level);
                }
                "read_sdc" => {
                    let path = args.last().ok_or_else(|| Stop::Refused("read_sdc path".into()))?;
                    let s = vyges_loom::sdc::Sdc::load(path).map_err(|e| Stop::Refused(format!("{path}: {}", e.0)))?;
                    if let Some(q) = s.unevaluated_queries.first() {
                        return vyges_cts::log::refuse(format!("{path}: {q}: object query not evaluated, not modelled"));
                    }
                    for c in ["set_disable_timing", "set_case_analysis", "create_generated_clock"] {
                        if s.ignored.iter().any(|i| i == c) {
                            return vyges_cts::log::refuse(format!("{path}: {c} (the clock network it changes) is not modelled"));
                        }
                    }
                    // `set_max_fanout V [current_design]` is the design's limit (read); on any other
                    // object it is not modelled.
                    d.sdc_max_fanout = s.max_fanout_on_objects;
                    d.design_max_fanout = s.max_fanout.map(|v| v as f32);
                    d.clocks = s
                        .clocks
                        .iter()
                        .map(|c| SdcClock {
                            name: c.name.clone(),
                            leaf_pins: c.sources.iter().filter(|x| !x.is_empty()).map(|x| if d.bterms.contains(x) { Pin::Port(x.clone()) } else { Pin::Term(split_iterm(x)) }).collect(),
                        })
                        .collect();
                    d.sdc = Some(s);
                }
                "set_routing_alpha" => d.routing_alpha_set = true,
                "set_max_fanout" => {
                    // `set_max_fanout value objects`: only library cells (by name) are modelled.
                    let [value, objects] = args.as_slice() else { return vyges_cts::log::refuse(format!("set_max_fanout {}: not a value and objects", args.join(" "))) };
                    let v: f32 = value.parse().map_err(|_| Stop::Refused(format!("set_max_fanout {value}: not a number")))?;
                    for name in objects.split_whitespace() {
                        if d.libs.iter().any(|l| l.cells.contains_key(name)) {
                            d.cell_max_fanout.insert(name.to_string(), v);
                        } else {
                            return vyges_cts::log::refuse(format!("set_max_fanout on {name} (not a library cell) is not modelled"));
                        }
                    }
                }
                "set_cts_config" => {
                    let dd = &d;
                    vyges_cts::options::set_cts_config(&mut o, &mut log, &MastersOf(dd), &args, &mut |o, log, b| vyges_cts::buffers::set_buffer_list(o, log, dd, b))?;
                    // The resizer's clock-buffer pattern, as set_cts_config left it.
                    d.clock_pattern = vyges_rsz::preamble::ClockPattern::from_settings(o.clock_buffer_string.as_deref(), o.clock_buffer_footprint.as_deref());
                }
                "clock_tree_synthesis" => {
                    // The command's own lines only: what the steps before it printed is kept apart.
                    pre.append(&mut log.lines);
                    return vyges_cts::run::clock_tree_synthesis(&mut o, &mut log, &mut d, &args).map(Some);
                }
                other => return vyges_cts::log::refuse(format!("{other}: not a modelled step")),
            }
            Ok(None)
        })();
        match r {
            Ok(None) => {}
            // No clock net: the command's whole output, nothing to build — checked (clock_nets 0).
            Ok(Some(Outcome::NoClockNets)) => {
                let mut v = finish(log, &pre, "up_to_date", None);
                v["clock_nets"] = json!(0);
                return Ok(v);
            }
            // Clock nets found and the trees built in memory: what is modelled ends here
            // (`modelled_through`); nothing is written, so the command is refused rather than done.
            // The trees written: the command is done; the design is written when the job asks.
            Ok(Some(Outcome::Done(f))) => {
                if let Some(path) = job["write_def"].as_str() {
                    d.db.write_def(path).map_err(|e| format!("write_def {path}: {e}"))?;
                }
                if let Some(path) = job["write_db"].as_str() {
                    d.db.write(path).map_err(|e| format!("write_db {path}: {e}"))?;
                }
                let mut v = finish(log, &pre, "done", None);
                v["clock_nets"] = json!(f.builders.len());
                return Ok(v);
            }
            Ok(Some(Outcome::Built(f, _))) => {
                let mut v = finish(log, &pre, "refused", Some("writing the clock trees to the database is not modelled yet".into()));
                v["modelled_through"] = json!("htree");
                v["clock_nets"] = json!(f.builders.len());
                v["builders"] = json!(f.builders.iter().map(|b| json!({"net": b.net, "input_net": b.input_net, "clock": b.sdc_clock, "sinks": b.sinks.len(), "parent": b.parent, "tree_type": b.tree_type})).collect::<Vec<_>>());
                return Ok(v);
            }
            Err(Stop::Error { tool, id }) => {
                let mut v = finish(log, &pre, "error", Some(format!("{tool}-{id:04}")));
                v["code"] = json!(format!("{tool}-{id:04}"));
                return Ok(v);
            }
            Err(Stop::Refused(why)) => return Ok(finish(log, &pre, "refused", Some(why))),
        }
    }
    // No clock_tree_synthesis step: nothing was checked.
    Ok(finish(log, &pre, "vacuous", Some("the job has no clock_tree_synthesis step".into())))
}

struct MastersOf<'a>(&'a CliDesign);

impl vyges_cts::options::Masters for MastersOf<'_> {
    fn has_master(&self, name: &str) -> bool {
        self.0.masters.contains(name)
    }
}

const USAGE: &str = "\
vyges physical cts — clock tree synthesis of a placed design: the clock buffers chosen, each clock
net found and its sinks collected (the trees themselves are not built yet)

USAGE:
  vyges physical cts clock_tree_synthesis <job.json> [-o FILE]
  vyges physical cts --describe
  vyges physical cts --help
  vyges physical cts --version

JOB FIELDS:
  steps        required — the commands in order, each {\"cmd\": ..., \"args\": [...]}, as a flow
               script passes them: read_db, read_liberty, set_wire_rc, set_layer_rc,
               set_dont_use, set_debug_level, read_sdc, set_cts_config, set_max_fanout,
               then clock_tree_synthesis [options]
  write_def    optional — where to write the design as DEF when the command is done
  write_db     optional — where to write the design as a database when the command is done

MODELLED:
  the buffer list (given, or inferred from the libraries), the root, sink and characterization
  buffers, clock-gate cloning, every clock net and its sinks, the characterization, each net's
  H-tree (sink clustering, simple segments, the obstruction-aware legalizer, macro and register
  trees), the trees written with their dummy loads and clock NDRs, the per-tree report, and the
  latency balancing of trees with sub-trees — the report's lines are the command's own, in its
  order.
  Not yet: -repair_clock_nets, hierarchical designs, clock buffer strings and footprints.

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

EXIT STATUS:
  0  done         the clock trees were built, written to the design and reported
  0  up_to_date   clock nets were looked for and none was found: nothing to build
  2  vacuous      the job has no clock_tree_synthesis step. NOT a pass.
  2  error        usage, unreadable input, or an error the command raises (its line is reported)
  3  refused      an input or a stage this engine does not model — see `reason` (and
                  `modelled_through` when the trees were built but not written)
";

const PIN_TOKEN: &str = "@OPENROAD_PIN@";

fn describe() -> String {
    DESCRIBE.replace(PIN_TOKEN, vyges_opendb::OPENROAD_PIN)
}

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "@OPENROAD_PIN@",
  "name": "cts",
  "summary": "clock tree synthesis of a placed design: clock buffers chosen, clock nets found, an H-tree built per net and written with its dummy loads",
  "maturity": "structured",
  "provenance_limitations": [
    "input_hash covers the argument vector, not the content of the job file or of the design files it names.",
    "status is one of done, up_to_date, vacuous, refused or error. done means the trees were built, written and reported; up_to_date means clock nets were looked for and none was found. A run that needs a stage not yet modelled (sink clustering, clock NDRs, clock-net repair, latency balancing of sub-trees) is refused, with modelled_through: htree when the trees were built but not written. The declared assertion passes on done and up_to_date. Exit status is 0 for done and up_to_date, 2 for vacuous and for error, 3 for refused.",
    "Modelled: buffer list inference, root/sink/characterization buffer selection, clock-gate cloning, clock-net discovery through gaters and bidirect pads. Refused rather than guessed: hierarchical designs where a clone or register-tree buffer is made, insertion-delay (clock-tree-path) cells, several corners, set_disable_timing, case analysis and generated clocks."
  ],
  "invocation": {
    "args_template": ["clock_tree_synthesis", "{job}"],
    "optional": [ { "arg": "out", "flag": "-o" } ],
    "emits_json": true
  },
  "inputs": {
    "type": "object",
    "required": ["job"],
    "properties": {
      "job": { "type": "string", "description": "path to a JSON job: {steps: [{cmd, args}]}" },
      "out": { "type": "string", "description": "write the JSON report to FILE instead of stdout" }
    }
  },
  "consumes": ["job"],
  "artifacts": [],
  "assertion": {
    "id": "clock-trees-built",
    "field": "status",
    "pass_when": { "in": ["done", "up_to_date"] }
  }
}"#;

fn link(flag: &str) -> Option<(&'static str, &'static str)> {
    Some(match flag {
        "--bug-report" => ("Report a bug", "https://github.com/vyges/community/issues/new?template=bug_report_template.yaml"),
        "--feature-request" => ("Request a feature", "https://github.com/vyges/community/issues/new?labels=enhancement"),
        "--sponsor" => ("Sponsor Vyges", "https://github.com/sponsors/vyges-ip"),
        "--star" => ("Star this tool", "https://github.com/vyges-tools/cts"),
        _ => return None,
    })
}

fn exit_for(status: &str) -> u8 {
    match status {
        "done" | "up_to_date" => 0,
        "refused" => 3,
        _ => 2,
    }
}

fn main() -> std::process::ExitCode {
    use std::process::ExitCode;
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut out: Option<String> = None;
    let mut positional: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "-o" => match args.get(i + 1) {
                Some(v) => {
                    out = Some(v.clone());
                    i += 1;
                }
                None => {
                    eprintln!("vyges-cts: -o needs a FILE");
                    return ExitCode::from(2);
                }
            },
            "--json" => {}
            a if a.starts_with('-') && !positional.is_empty() => {
                eprintln!("vyges-cts: unknown option {a}\n\n{USAGE}");
                return ExitCode::from(2);
            }
            a => positional.push(a),
        }
        i += 1;
    }
    match positional.first().copied() {
        None | Some("-h") | Some("--help") => {
            print!("{USAGE}");
            return ExitCode::SUCCESS;
        }
        Some("-V") | Some("--version") => {
            println!("vyges-cts {} ({})\nCopyright (c) Vyges. Apache-2.0.", env!("CARGO_PKG_VERSION"), env!("VYGES_GIT_SHA"));
            return ExitCode::SUCCESS;
        }
        Some("--describe") => {
            println!("{}", describe());
            return ExitCode::SUCCESS;
        }
        Some(f) if link(f).is_some() => {
            let (label, url) = link(f).expect("a link flag");
            println!("{label}:\n  {url}");
            return ExitCode::SUCCESS;
        }
        Some("clock_tree_synthesis") if positional.len() == 2 => {}
        _ => {
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    }
    let path = positional[1];
    let report = match std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}")).and_then(|t| serde_json::from_str::<Value>(&t).map_err(|e| format!("{path}: {e}"))) {
        Err(e) => json!({ "tool": "vyges-cts", "status": "error", "reason": e }),
        Ok(job) => match run(&job) {
            Ok(r) => r,
            Err(e) => json!({ "tool": "vyges-cts", "status": "error", "reason": e }),
        },
    };
    let status = report["status"].as_str().unwrap_or("error").to_string();
    emit_events(&report);
    let text = format!("{report}\n");
    match &out {
        Some(f) => {
            if let Err(e) = std::fs::write(f, &text) {
                eprintln!("vyges-cts: {f}: {e}");
                return ExitCode::from(2);
            }
        }
        None => print!("{text}"),
    }
    ExitCode::from(exit_for(&status))
}

/// The run's lines as structured events (`vyges-events/1.0`): each `[INFO|WARNING|ERROR TOOL-nnnn]
/// text` line the reference's logger would print under its own code and severity, any other line
/// (`report` text) as information; a refusal under `CTS-REFUSED`. The JSON report is unchanged.
fn emit_events(report: &Value) {
    use vyges_events::{emit, Event, Severity};
    let lines = report["before"].as_array().into_iter().flatten().chain(report["lines"].as_array().into_iter().flatten());
    for l in lines.filter_map(Value::as_str) {
        let parsed = l.strip_prefix('[').and_then(|r| r.split_once("] ")).and_then(|(head, text)| head.split_once(' ').map(|(sev, code)| (sev, code, text)));
        let ev = match parsed {
            Some((sev, code, text)) => {
                let severity = match sev {
                    "ERROR" | "CRITICAL" => Severity::Error,
                    "WARNING" => Severity::Warn,
                    _ => Severity::Info,
                };
                Event::new("vyges-cts", severity, text.to_string()).with_code(code)
            }
            None => Event::new("vyges-cts", Severity::Info, l.to_string()),
        };
        emit(&ev);
    }
    if report["status"] == "refused" {
        if let Some(why) = report["reason"].as_str() {
            emit(&Event::new("vyges-cts", Severity::Warn, why.to_string()).with_code("CTS-REFUSED"));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_descriptor_is_valid_json_and_names_the_engine() {
        let d: Value = serde_json::from_str(&describe()).expect("--describe must be valid JSON");
        assert_eq!(d["name"], "cts");
        assert_eq!(d["openroad_pin"].as_str().map(str::len), Some(40));
    }

    // ⛔ `maturity` has exactly THREE legal values — discovered, structured, workflow-validated —
    // and the parser maps anything else to `discovered`, where the verdict is suppressed to
    // `unknown`. This engine said `experimental`, which threw every verdict away. `structured`:
    // a versioned operation, a normalized result and a declared assertion; the correlation
    // harness lives outside the repository, so not `workflow-validated`.
    #[test]
    fn the_maturity_is_a_legal_rung() {
        let d: Value = serde_json::from_str(&describe()).expect("--describe must be valid JSON");
        let m = d["maturity"].as_str().expect("maturity is a string");
        assert!(["discovered", "structured", "workflow-validated"].contains(&m), "{m}");
        assert_eq!(m, "structured");
    }

    // Rule (vacuous convention): only a checked nothing-to-do exits 0; a refusal never does.
    #[test]
    fn only_done_and_up_to_date_exit_zero() {
        assert_eq!(exit_for("done"), 0);
        assert_eq!(exit_for("up_to_date"), 0);
        assert_eq!(exit_for("refused"), 3);
        assert_eq!(exit_for("vacuous"), 2);
        assert_eq!(exit_for("error"), 2);
    }
}
