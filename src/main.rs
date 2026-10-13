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
    clock_buffer_pattern: bool,
    masters: HashSet<String>,
    /// `dbNet` ids by name (the block's net order).
    net_ids: HashMap<String, u32>,
    bterms: HashSet<String>,
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
    fn find_max_wire_length(&self) -> Res<f64> {
        let v = self.rc.resolved(&self.tech, 0);
        let rc = vyges_rsz::buffered_net::WireRc { h_res: v[0], v_res: v[1], h_cap: v[2], v_cap: v[3] };
        // No scene with a signal resistance: -inf, before any buffer is looked at.
        if self.rc.resolve(&self.tech, |w| &w.signal_res).is_empty() || (rc.h_res + rc.v_res) / 2.0 <= 0.0 {
            return Ok(f64::NEG_INFINITY);
        }
        if self.clock_buffer_pattern {
            return vyges_cts::log::refuse("findBuffers with a clock buffer string or footprint is not modelled");
        }
        let libs = vyges_rsz::preamble::Libs { libs: self.libs.clone(), scenes: Vec::new(), scene_libs: Vec::new(), link_scene: None };
        let masters = rsz_masters(&self.db)?;
        let mut dont_use = self.set_dont_use.clone();
        for lib in &self.libs {
            dont_use.extend(lib.cells.values().filter(|c| c.dont_use).map(|c| c.name.clone()));
        }
        // Clock buffers excluded: the resizer's default, which only its hold repair turns off.
        let buffers = vyges_rsz::preamble::find_buffers(&libs, &masters, &dont_use, true).map_err(|e| Stop::Refused(format!("findBuffers: {}", e.message())))?;
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
}

fn read_text(path: &str) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))
}

fn run(job: &Value) -> Result<Value, String> {
    let mut d = CliDesign { db: Db::new(), libs: Vec::new(), rc: vyges_est::rc::Rc::new(), tech: String::new(), clocks: Vec::new(), dont_use: BTreeSet::new(), set_dont_use: BTreeSet::new(), routing_alpha_set: false, clock_buffer_pattern: false, masters: HashSet::new(), net_ids: HashMap::new(), bterms: HashSet::new() };
    let mut liberty = vyges_est::liberty::LibertyClocks::default();
    let mut o = Options::default();
    let mut log = Log::default();
    let mut pre: Vec<String> = Vec::new();
    let units = |l: &vyges_est::liberty::LibertyClocks| -> Result<vyges_est::rc::Units, String> {
        let u = l.units.ok_or("RC before a liberty library: the timer's default units are not modelled")?;
        Ok(vyges_est::rc::Units { resistance: u.resistance, capacitance: u.capacitance, distance: u.distance })
    };
    let finish = |log: Log, pre: &[String], status: &str, reason: Option<String>| json!({"tool": "vyges-cts", "status": status, "reason": reason, "before": pre, "lines": log.lines});
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
                    liberty.read(&text).map_err(|e| Stop::Refused(format!("{path}: {e}")))?;
                    let group = vyges_sta::liberty_parse::parse(&text).map_err(|e| Stop::Refused(format!("{path}: {e}")))?;
                    d.libs.push(Library::read(&group).map_err(|e| Stop::Refused(format!("{path}: {e} (not modelled)")))?);
                }
                "define_corners" => return vyges_cts::log::refuse("define_corners: several scenes are not modelled"),
                "set_wire_rc" => {
                    d.rc.check_set_wire_rc(&d.db, &args).map_err(Stop::Refused)?;
                    d.rc.set_wire_rc(&d.db, units(&liberty).map_err(Stop::Refused)?, &args).map_err(Stop::Refused)?;
                }
                "set_layer_rc" => d.rc.set_layer_rc(&mut d.db, units(&liberty).map_err(Stop::Refused)?, &args).map_err(Stop::Refused)?,
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
                    for c in ["set_disable_timing", "set_case_analysis", "create_generated_clock"] {
                        if s.ignored.iter().any(|i| i == c) {
                            return vyges_cts::log::refuse(format!("{path}: {c} (the clock network it changes) is not modelled"));
                        }
                    }
                    d.clocks = s
                        .clocks
                        .iter()
                        .map(|c| SdcClock {
                            name: c.name.clone(),
                            leaf_pins: c.sources.iter().filter(|x| !x.is_empty()).map(|x| if d.bterms.contains(x) { Pin::Port(x.clone()) } else { Pin::Term(split_iterm(x)) }).collect(),
                        })
                        .collect();
                }
                "set_routing_alpha" => d.routing_alpha_set = true,
                "set_cts_config" => {
                    if args.iter().any(|a| a.starts_with("-clock_buffer_")) {
                        d.clock_buffer_pattern = true;
                    }
                    let dd = &d;
                    vyges_cts::options::set_cts_config(&mut o, &mut log, &MastersOf(dd), &args, &mut |o, log, b| vyges_cts::buffers::set_buffer_list(o, log, dd, b))?;
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
            // Clock nets found: what is modelled ends here (`modelled_through`); the trees are not
            // built, so the command is refused rather than reported as done.
            Ok(Some(Outcome::Discovered(f))) => {
                let mut v = finish(log, &pre, "refused", Some("characterization and clock tree building are not modelled yet".into()));
                v["modelled_through"] = json!("discovery");
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
               set_dont_use, set_debug_level, read_sdc, set_cts_config, then
               clock_tree_synthesis [options]

MODELLED:
  the buffer list (given, or inferred from the libraries), the root, sink and characterization
  buffers, clock-gate cloning, every clock net and its sinks (CTS-0049..0052, CTS-0007..0011,
  CTS-0008, CTS-0082) — the report's lines are the command's own, in its order.
  Not yet: characterization, the trees, dummy loads, latency balancing — a design with a clock
  net is refused after discovery (modelled_through: discovery).

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

EXIT STATUS:
  0  up_to_date   clock nets were looked for and none was found: nothing to build
  2  vacuous      the job has no clock_tree_synthesis step. NOT a pass.
  2  error        usage, unreadable input, or an error the command raises (its line is reported)
  3  refused      an input this engine does not model — or a clock tree to build — see `reason`
";

const PIN_TOKEN: &str = "@OPENROAD_PIN@";

fn describe() -> String {
    DESCRIBE.replace(PIN_TOKEN, vyges_opendb::OPENROAD_PIN)
}

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "openroad_pin": "@OPENROAD_PIN@",
  "name": "cts",
  "summary": "clock tree synthesis of a placed design: clock buffers chosen, clock nets found and their sinks collected",
  "maturity": "experimental",
  "provenance_limitations": [
    "input_hash covers the argument vector, not the content of the job file or of the design files it names.",
    "status is one of up_to_date, vacuous, refused or error. up_to_date means clock nets were looked for and none was found. A design WITH a clock net is refused after discovery (modelled_through: discovery): the trees are not built yet. The declared assertion passes on up_to_date only. Exit status is 0 for up_to_date, 2 for vacuous and for error, 3 for refused.",
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
    "pass_when": { "in": ["up_to_date"] }
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
        "up_to_date" => 0,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_descriptor_is_valid_json_and_names_the_engine() {
        let d: Value = serde_json::from_str(&describe()).expect("--describe must be valid JSON");
        assert_eq!(d["name"], "cts");
        assert_eq!(d["openroad_pin"].as_str().map(str::len), Some(40));
    }

    // Rule (vacuous convention): only a checked nothing-to-do exits 0; a refusal never does.
    #[test]
    fn only_up_to_date_exits_zero() {
        assert_eq!(exit_for("up_to_date"), 0);
        assert_eq!(exit_for("refused"), 3);
        assert_eq!(exit_for("vacuous"), 2);
        assert_eq!(exit_for("error"), 2);
    }
}
