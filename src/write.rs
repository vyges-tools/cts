// SPDX-License-Identifier: Apache-2.0
//! Writing the trees to the database (`writeDataToDb`): each tree's buffers and subnets, then its
//! dummy loads, then the per-tree report read back from the written database.
//!
//! The database edits are made in the reference's order — the order a net's terminals are listed
//! in, and so the written design, follows from it. Each stage is its own function, named after the
//! reference's.

use crate::discovery::{ClockDesign, ITerm};
use crate::htree::{Clock, InstRef, TreeType};
use crate::log::{refuse, Log, Res};
use std::collections::{BTreeMap, HashMap, HashSet};

/// The database edits and reads the writer needs beyond discovery's.
pub trait TreeDb: ClockDesign {
    /// `dbITerm::getIoType` (`INPUT`, `OUTPUT`, `INOUT`, `FEEDTHRU`).
    fn iterm_io_type(&self, t: &ITerm) -> String;
    /// `dbInst::getFirstOutput`: the first non-supply OUTPUT terminal.
    fn inst_first_output(&self, inst: &str) -> Option<ITerm>;
    fn net_sig_type(&self, net: &str) -> String;
    /// `getITermCount`.
    fn net_iterm_count(&self, net: &str) -> u32;
    /// The location `countSinksPostDbWrite` takes from an INPUT port on the net: the first box of
    /// the first placed pin of each such port (the last port wins).
    fn net_input_port_xy(&self, net: &str) -> Option<(i32, i32)>;
    fn connect(&mut self, t: &ITerm, net: &str) -> Res<()>;
    fn disconnect(&mut self, t: &ITerm) -> Res<()>;
    /// `dbInst::create` (top module), `TIMING`, at `(x, y)`, `PLACED`.
    fn create_placed_inst(&mut self, master: &str, name: &str, x: i32, y: i32) -> Res<()>;
    /// As [`create_placed_inst`](Self::create_placed_inst), in `owner`'s module (`dbInst::create(..,
    /// parent)`).
    fn create_placed_inst_beside(&mut self, master: &str, name: &str, x: i32, y: i32, owner: &str) -> Res<()>;
    /// `dbModNet::destroy` of the pin's hierarchical net (a port `pin` when `inst` is empty); false
    /// when it has none.
    fn destroy_pin_mod_net(&mut self, inst: &str, pin: &str) -> Res<bool>;
    /// `dbNetwork::hierarchicalConnect(src, dst, name)`.
    fn hierarchical_connect(&mut self, src: &ITerm, dst: &ITerm, name: &str) -> Res<()>;
    /// `dbNet::create` + `CLOCK`.
    fn create_clock_net(&mut self, name: &str) -> Res<()>;
    fn set_net_clock(&mut self, net: &str) -> Res<()>;
    fn destroy_net(&mut self, net: &str) -> Res<()>;
    fn destroy_inst(&mut self, inst: &str) -> Res<()>;
    /// The resizer's `dontUse` (its own list, `set_dont_use`).
    fn resizer_dont_use_cell(&self, cell: &str) -> bool;
    /// `dbITerm::getAvgXY`, and the messages the database logged while answering (a pin it
    /// cannot place warns), in order.
    fn iterm_avg_xy_logged(&mut self, t: &ITerm) -> (Option<(i32, i32)>, Vec<String>);
    /// The liberty libraries, in read order.
    fn liberty_libraries(&self) -> &[vyges_sta::liberty::Library];
    /// The routing layers in routing order: name, default width, pitch, backside.
    fn routing_layers(&self) -> Vec<(String, i32, i32, bool)>;
    /// A block non-default rule; false when the name is taken.
    fn create_ndr(&mut self, name: &str) -> Res<bool>;
    /// The rule's width and spacing on a layer (its layer rule made when absent).
    fn set_ndr_layer(&mut self, ndr: &str, layer: &str, width: i32, spacing: i32) -> Res<()>;
    fn set_net_ndr(&mut self, net: &str, ndr: &str) -> Res<()>;
    /// `findMaster`.
    fn has_master_cell(&self, name: &str) -> bool;
    /// The clock arrival `getVertexClkArrival` reads at a sink: on the written design, clocks
    /// propagated and the placement parasitics estimated (timed once, when first asked), the first
    /// max path of a rising clock edge whose clock path starts on `top_net`; `None` without one.
    fn sink_clock_arrival(&mut self, t: &ITerm, top_net: &str) -> Res<Option<f32>>;
    /// The centre of the net's first port's bounding box.
    fn net_first_bterm_center(&self, net: &str) -> Option<(i32, i32)>;
    /// `dbNet::insertBufferBeforeLoads` (`IF_NEEDED` names, loads on other nets allowed): the new
    /// instance's name.
    fn insert_buffer_before_loads(&mut self, net: &str, loads: &[ITerm], master: &str, loc: (i32, i32), buf_name: &str, net_name: &str) -> Res<String>;
}

/// `CtsOptions::NdrStrategy`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ndr {
    None,
    RootOnly,
    Half,
    Full,
}

impl Ndr {
    /// The Tcl argument, case-insensitive; `None` for a value the command rejects.
    pub fn parse(s: &str) -> Option<Ndr> {
        match s.to_ascii_uppercase().as_str() {
            "NONE" => Some(Ndr::None),
            "ROOT_ONLY" => Some(Ndr::RootOnly),
            "HALF" => Some(Ndr::Half),
            "FULL" => Some(Ndr::Full),
            _ => None,
        }
    }
}

/// One built tree and what the writer and its report read of the builder.
#[derive(Debug, Clone)]
pub struct Tree {
    pub clock: Clock,
    pub net: String,
    /// The SDC clock and the net the clock enters on (`getTopInputNet`).
    pub sdc_clock: String,
    pub top_input_net: String,
    pub tree_type: TreeType,
    pub top_buffer_name: String,
    /// `setDrivingNet` (the register tree's: the macro tree's net).
    pub driving_net: Option<String>,
    pub parent: Option<usize>,
    pub has_children: bool,
    /// Buffers (indices into `clock.buffers`) in the builder's level and leaf sets.
    pub tree_buffers: HashSet<usize>,
    pub leaf_buffers: HashSet<usize>,
    pub n_dummies: i32,
    /// Subnets whose net was removed while writing (no net object).
    pub removed_subnets: HashSet<usize>,
}

/// What the writer reads of the options.
#[derive(Debug, Clone)]
pub struct WOptions {
    pub dummy_load: bool,
    pub max_fanout: u32,
    pub buffer_list: Vec<String>,
    pub db_units: i32,
    pub ndr: Ndr,
}

/// The writer's run state (the engine members it touches).
#[derive(Default)]
pub struct Writer {
    pub dummy_load_index: u32,
    pub num_clock_subnets: i32,
    pub num_buffers_inserted: i32,
    /// `inst2clkbuf_`: a database instance → (tree, buffer).
    pub inst_to_buffer: HashMap<String, (usize, usize)>,
}

/// `getFirstInput`: the first input-signal terminal.
fn first_input(d: &dyn TreeDb, inst: &str) -> Option<ITerm> {
    d.inst_iterms(inst).into_iter().find(|t| d.iterm_is_input_signal(t, true))
}

/// A sink's `inst/pin` as a terminal.
fn sink_term(name: &str) -> ITerm {
    let (i, p) = name.rsplit_once('/').unwrap_or((name, ""));
    (i.to_string(), p.to_string())
}

/// `writeDataToDb`.
pub fn write_data_to_db(log: &mut Log, d: &mut dyn TreeDb, w: &mut Writer, o: &WOptions, trees: &mut [Tree]) -> Res<()> {
    let mut dummies: HashSet<String> = HashSet::new();
    for i in 0..trees.len() {
        write_clock_nets_to_db(log, d, w, trees, i)?;
        if o.dummy_load {
            let n = write_dummy_loads_to_db(log, d, w, o, &mut trees[i], &mut dummies)?;
            trees[i].n_dummies = n;
        }
    }
    // NDRs after every tree is written: levels are numbered across trees.
    if o.ndr != Ndr::None {
        write_clock_ndrs_to_db(log, d, trees, o.ndr)?;
    }
    for i in 0..trees.len() {
        let t = &trees[i];
        let sinks: HashSet<ITerm> = t.clock.sinks.iter().map(|s| sink_term(&s.name)).collect();
        let report_full_tree = t.parent.is_none() && t.has_children;
        if sinks.len() < 2 {
            log.info("CTS", 124, format!("Clock net \"{}\"", t.clock.name));
            log.info("CTS", 125, format!(" Sinks {}", sinks.len()));
            continue;
        }
        let mut c = Count::default();
        let mut visited = HashSet::new();
        count_sinks_post_db_write(log, d, w, trees, i, o.db_units, &t.net, &mut c, 0, 0, report_full_tree, &sinks, &dummies, &mut visited)?;
        log.info("CTS", 98, format!("Clock net \"{}\"", t.clock.name));
        log.info("CTS", 99, format!(" Sinks {}", c.sinks));
        log.info("CTS", 100, format!(" Leaf buffers {}", c.leaf_sinks));
        if c.sinks > 0 {
            log.info("CTS", 101, format!(" Average sink wire length {:.2} um", c.sink_wire_length / f64::from(c.sinks)));
        }
        log.info("CTS", 102, format!(" Path depth {} - {}", c.min_depth, c.max_depth));
        if o.dummy_load {
            log.info("CTS", 207, format!(" Dummy loads inserted {}", t.n_dummies));
        }
    }
    Ok(())
}

/// `writeClockNetsToDb`.
fn write_clock_nets_to_db(log: &mut Log, d: &mut dyn TreeDb, w: &mut Writer, trees: &mut [Tree], ti: usize) -> Res<()> {
    let top_net = trees[ti].net.clone();
    // getNetDriverParentModule: the clock net's driver — an instance's output, else a port — whose
    // instance's module the buffers are made in (the top for a port).
    let driver = d.net_first_output(&top_net);
    let driver_port = if driver.is_none() { d.net_bterms(&top_net).into_iter().find(|p| d.bterm_is_input(p)) } else { None };
    disconnect_all_sinks_from_net(d, &top_net)?;
    // destroyClockModNet: the driver's module net loses its loads to the tree; only the root
    // buffer goes back on the flat net.
    if d.has_hierarchy() {
        match (&driver, &driver_port) {
            (Some(t), _) => {
                d.destroy_pin_mod_net(&t.0, &t.1)?;
            }
            (None, Some(p)) => {
                d.destroy_pin_mod_net("", p)?;
            }
            (None, None) => {}
        }
    }
    // The register tree's top buffer goes back on the net that drives it.
    if trees[ti].tree_type == TreeType::Register {
        let top = trees[ti].top_buffer_name.clone();
        if d.inst_master(&top).is_empty() {
            // `findInst` found none: nothing to reconnect.
        } else {
            let input = first_input(d, &top).ok_or_else(|| crate::log::Stop::Refused(format!("{top}: no input")))?;
            let net = trees[ti].driving_net.clone().ok_or_else(|| crate::log::Stop::Refused("a register tree without a driving net".into()))?;
            d.connect(&input, &net)?;
        }
    }
    let owner = driver.as_ref().filter(|_| d.has_hierarchy()).map(|t| t.0.clone());
    create_clock_buffers(log, d, w, trees, ti, owner.as_deref())?;
    // The root buffer on the clock net.
    let clock_name = trees[ti].clock.name.clone();
    let top_inst = format!("clkbuf_0_{clock_name}");
    let input = first_input(d, &top_inst).ok_or_else(|| crate::log::Stop::Refused(format!("{top_inst}: no input")))?;
    d.connect(&input, &top_net)?;
    d.set_net_clock(&top_net)?;

    let mut fanout: BTreeMap<i32, i32> = BTreeMap::new();
    let mut num_clk_nets = 0i32;
    let mut num_fixed_nets = 0i32;
    let mut root_sub_net = None;
    let mut removed: HashSet<InstRef> = HashSet::new();
    let n_subnets = trees[ti].clock.subnets.len();
    for s in 0..n_subnets {
        let mut sub = trees[ti].clock.subnets[s].clone();
        let mut output_found = true;
        let mut input_found = true;
        if sub.name == format!("clknet_0_{clock_name}") {
            root_sub_net = Some(s);
        }
        d.create_clock_net(&sub.name)?;
        num_clk_nets += 1;
        let driver = trees[ti].clock.inst(sub.insts[0]).name.clone();
        let driver_input = first_input(d, &driver);
        let input_net = driver_input.as_ref().and_then(|t| d.iterm_net(t));
        let output = d.inst_first_output(&driver);
        match &output {
            None => output_found = false,
            Some(o) => d.connect(o, &sub.name)?,
        }
        let num_sinks = sub.insts.len() as i32 - 1;
        if num_sinks == 0 {
            input_found = false;
        }
        for &r in &sub.insts[1..] {
            let pin = match r {
                InstRef::Buffer(b) => first_input(d, &trees[ti].clock.buffers[b].name),
                InstRef::Sink(k) => Some(sink_term(&trees[ti].clock.sinks[k].name)),
            };
            let placed = pin.as_ref().map(|p| d.inst_is_placed(&p.0));
            input_found = still_connecting(input_found, placed);
            if input_found {
                let pin = pin.as_ref().expect("found");
                d.connect(pin, &sub.name)?;
                // The connection resolved through the module hierarchy.
                if d.has_hierarchy() {
                    let Some(o) = &output else {
                        return refuse(format!("{}: a hierarchical connection from a buffer with no output", sub.name));
                    };
                    d.hierarchical_connect(o, pin, &sub.name)?;
                    // hierarchicalConnect ends in renameWithModNetInHighestHier: the flat net takes
                    // its highest module net's name (`clknet_1_0__leaf_clk` -> `..._clk_1`). The
                    // reference holds the dbNet, so follow the rename here.
                    if let Some(n) = d.iterm_net(o) {
                        if n != sub.name {
                            sub.name = n.clone();
                            trees[ti].clock.subnets[s].name = n;
                        }
                    }
                }
            }
        }
        if sub.leaf {
            *fanout.entry(num_sinks).or_insert(0) += 1;
        }
        if !input_found || !output_found {
            disconnect_all_pins_from_net(d, &sub.name)?;
            d.destroy_net(&sub.name)?;
            trees[ti].removed_subnets.insert(s);
            num_fixed_nets += 1;
            num_clk_nets -= 1;
            d.destroy_inst(&driver)?;
            removed.insert(sub.insts[0]);
            if let Some(n) = input_net {
                check_upstream_connections(d, &n, &mut num_fixed_nets, &mut num_clk_nets)?;
            }
        }
    }
    let Some(root) = root_sub_net else {
        return log.error("CTS", 85, format!("Could not find the root of {clock_name}"));
    };
    let (mut min_path, mut max_path) = (i32::MAX, i32::MIN);
    let root_sinks: Vec<InstRef> = trees[ti].clock.subnets[root].insts[1..].to_vec();
    for r in root_sinks {
        if !removed.contains(&r) {
            if let InstRef::Buffer(b) = r {
                let (lo, hi) = branch_buffer_count(d, &trees[ti].clock, b, 1);
                min_path = min_path.min(lo);
                max_path = max_path.max(hi);
            }
        } else {
            // `removeSinks`: the root subnet keeps its driver and the sinks not removed.
            let sub = &mut trees[ti].clock.subnets[root];
            let kept: Vec<InstRef> = sub.insts[1..].iter().copied().filter(|x| !removed.contains(x)).collect();
            sub.insts.truncate(1);
            sub.insts.extend(kept);
        }
    }
    log.info("CTS", 12, format!("    Minimum number of buffers in the clock path: {min_path}."));
    log.info("CTS", 13, format!("    Maximum number of buffers in the clock path: {max_path}."));
    if num_fixed_nets > 0 {
        log.info("CTS", 14, format!("    {num_fixed_nets} clock nets were removed/fixed."));
    }
    log.info("CTS", 15, format!("    Created {num_clk_nets} clock nets."));
    w.num_clock_subnets += num_clk_nets;
    let dist: Vec<String> = fanout.iter().map(|(k, v)| format!("{k}:{v}")).collect();
    // The distribution is built with a trailing ", " cut to "." — and the message adds its own.
    log.info("CTS", 16, format!("    Fanout distribution for the current clock = {}..", dist.join(", ")));
    log.info("CTS", 17, format!("    Max level of the clock tree: {}.", trees[ti].clock.max_level));
    Ok(())
}

/// `disconnectAllSinksFromNet`: every INPUT terminal, in the net's order.
fn disconnect_all_sinks_from_net(d: &mut dyn TreeDb, net: &str) -> Res<()> {
    for t in d.net_iterms(net) {
        if d.iterm_io_type(&t) == "INPUT" {
            d.disconnect(&t)?;
        }
    }
    Ok(())
}

/// `disconnectAllPinsFromNet`.
fn disconnect_all_pins_from_net(d: &mut dyn TreeDb, net: &str) -> Res<()> {
    for t in d.net_iterms(net) {
        d.disconnect(&t)?;
    }
    Ok(())
}

/// `checkUpstreamConnections`: a net left with at most one terminal is removed, with the buffer
/// that terminal belongs to, walking up.
fn check_upstream_connections(d: &mut dyn TreeDb, net: &str, fixed: &mut i32, clk_nets: &mut i32) -> Res<()> {
    let mut net = net.to_string();
    while d.net_iterm_count(&net) <= 1 {
        let Some(first) = d.net_iterms(&net).into_iter().next() else {
            disconnect_all_pins_from_net(d, &net)?;
            d.destroy_net(&net)?;
            break;
        };
        let buffer = first.0.clone();
        let input = first_input(d, &buffer);
        disconnect_all_pins_from_net(d, &net)?;
        d.destroy_net(&net)?;
        let Some(next) = input.and_then(|t| d.iterm_net(&t)) else {
            return refuse(format!("checkUpstreamConnections: {buffer} has no input net"));
        };
        net = next;
        *fixed += 1;
        *clk_nets -= 1;
        d.destroy_inst(&buffer)?;
    }
    Ok(())
}

/// `createClockBuffers`: every buffer of the tree, in the order added, CTS-0018.
fn create_clock_buffers(log: &mut Log, d: &mut dyn TreeDb, w: &mut Writer, trees: &[Tree], ti: usize, owner: Option<&str>) -> Res<()> {
    let mut n = 0u32;
    for (bi, b) in trees[ti].clock.buffers.iter().enumerate() {
        match owner {
            Some(o) => d.create_placed_inst_beside(&b.master, &b.name, b.x, b.y, o)?,
            None => d.create_placed_inst(&b.master, &b.name, b.x, b.y)?,
        }
        w.inst_to_buffer.insert(b.name.clone(), (ti, bi));
        n += 1;
    }
    log.info("CTS", 18, format!("    Created {n} clock buffers."));
    w.num_buffers_inserted += n as i32;
    Ok(())
}

/// `branchBufferCount`: the fewest and most buffers from a root-net buffer to a sink.
fn branch_buffer_count(d: &dyn TreeDb, clock: &Clock, b: usize, counter: i32) -> (i32, i32) {
    let name = &clock.buffers[b].name;
    let (mut min_path, mut max_path) = (i32::MAX, i32::MIN);
    let Some(out) = d.inst_first_output(name) else { return (min_path, max_path) };
    let Some(net) = d.iterm_net(&out) else { return (min_path, max_path) };
    for t in d.net_iterms(&net) {
        if t == out {
            continue;
        }
        match clock.buffers.iter().position(|x| x.name == t.0) {
            None => {
                min_path = min_path.min(counter + 1);
                max_path = max_path.max(counter + 1);
            }
            Some(c) => {
                let (lo, hi) = branch_buffer_count(d, clock, c, counter + 1);
                min_path = min_path.min(lo);
                max_path = max_path.max(hi);
            }
        }
    }
    (min_path, max_path)
}

/// `LibertyPort::capacitance()`: the larger of the rise and fall max capacitances.
fn port_cap(p: &vyges_sta::liberty::Port) -> f32 {
    p.capacitance[vyges_sta::liberty::RISE][vyges_sta::liberty::MAX].max(p.capacitance[vyges_sta::liberty::FALL][vyges_sta::liberty::MAX])
}

/// `getInputPinCap`: the pin's liberty port capacitance (0 without one).
pub fn input_pin_cap(d: &dyn TreeDb, t: &ITerm) -> f32 {
    let master = d.inst_master(&t.0);
    d.liberty_cell(&master).and_then(|c| c.ports.iter().find(|p| p.name == t.1)).map_or(0.0, port_cap)
}

/// `sta::fuzzyEqual` for floats.
pub(crate) fn sta_fuzzy_equal(a: f32, b: f32) -> bool {
    if a == b {
        return true;
    }
    if a == 0.0 || b == 0.0 {
        return (a - b).abs() < 1e-15;
    }
    (a - b).abs() < 1e-6 * a.abs().max(b.abs())
}

/// `writeDummyLoadsToDb`.
fn write_dummy_loads_to_db(log: &mut Log, d: &mut dyn TreeDb, w: &mut Writer, o: &WOptions, t: &mut Tree, dummies: &mut HashSet<String>) -> Res<i32> {
    let (mut out_cap, ideal, driver_subnet, need) = compute_ideal_output_caps(log, d, t);
    if !need {
        return Ok(0);
    }
    let candidates = find_candidate_dummy_cells(log, d, o)?;
    let mut n = 0;
    let n_subnets = t.clock.subnets.len();
    for s in 0..n_subnets {
        let mut k = 1;
        // The sink list is read by index as it grows (a dummy joins another subnet).
        while k < t.clock.subnets[s].insts.len() {
            let r = t.clock.subnets[s].insts[k];
            k += 1;
            let InstRef::Buffer(b) = r else { continue };
            let (Some(&cap), Some(&id)) = (out_cap.get(&b), ideal.get(&b)) else { continue };
            if sta_fuzzy_equal(cap, id) {
                continue;
            }
            if insert_dummy_cell(log, d, w, o, t, b, &driver_subnet, &candidates, cap, id, &mut out_cap)? {
                dummies.insert(format!("clkload{}", w.dummy_load_index - 1));
                n += 1;
            }
        }
    }
    Ok(n)
}

/// `computeIdealOutputCaps`: each driver's load (its sinks' input pin caps), then each buffer's
/// ideal as the largest load among its siblings. True when any buffer differs.
#[allow(clippy::type_complexity)]
fn compute_ideal_output_caps(log: &mut Log, d: &dyn TreeDb, t: &Tree) -> (HashMap<usize, f32>, HashMap<usize, f32>, HashMap<usize, usize>, bool) {
    let mut out_cap: HashMap<usize, f32> = HashMap::new();
    let mut driver_subnet: HashMap<usize, usize> = HashMap::new();
    let mut sink_cap: HashMap<usize, f32> = HashMap::new();
    for (s, sub) in t.clock.subnets.iter().enumerate() {
        let mut total = 0.0f32;
        for &r in &sub.insts[1..] {
            let pin = match r {
                InstRef::Buffer(b) => first_input(d, &t.clock.buffers[b].name),
                InstRef::Sink(k) => Some(sink_term(&t.clock.sinks[k].name)),
            };
            total += pin.map_or(0.0, |p| input_pin_cap(d, &p));
        }
        match sub.insts[0] {
            InstRef::Buffer(b) => {
                driver_subnet.insert(b, s);
                out_cap.insert(b, total);
            }
            InstRef::Sink(k) => {
                sink_cap.insert(k, total);
            }
        }
    }
    let mut ideal: HashMap<usize, f32> = HashMap::new();
    let mut need = false;
    for sub in &t.clock.subnets {
        let mut max_cap = f32::MIN_POSITIVE;
        for &r in &sub.insts[1..] {
            if let InstRef::Buffer(b) = r {
                let c = out_cap.get(&b).copied().unwrap_or(0.0);
                if c > max_cap {
                    max_cap = c;
                }
            }
        }
        for &r in &sub.insts[1..] {
            if let InstRef::Buffer(b) = r {
                ideal.insert(b, max_cap);
                let cap = out_cap.get(&b).copied().unwrap_or(0.0);
                if !sta_fuzzy_equal(cap, max_cap) {
                    need = true;
                    let drv = t.clock.inst(sub.insts[0]).name.clone();
                    log.debug("CTS", "dummy load", 1, || format!("{drv} => {} cap:{} idealCap:{} delCap:{}", t.clock.buffers[b].name, crate::log::fmt_e(f64::from(cap), 2), crate::log::fmt_e(f64::from(max_cap), 2), crate::log::fmt_e(f64::from(max_cap - cap), 2)));
                }
            }
        }
    }
    (out_cap, ideal, driver_subnet, need)
}

/// A dummy-load candidate: the cell and its input capacitance.
#[derive(Debug, Clone)]
struct Candidate {
    cell: String,
    cap: f32,
}

/// `getInputCap` of a cell: its buffer input port's capacitance (0 without).
fn cell_input_cap(c: &vyges_sta::liberty::Cell) -> f32 {
    c.buffer_ports().map_or(0.0, |(i, _)| port_cap(i))
}

/// A case-insensitive glob (`*` and `?`).
fn glob_nocase(pat: &str, s: &str) -> bool {
    let p: Vec<char> = pat.to_lowercase().chars().collect();
    let s: Vec<char> = s.to_lowercase().chars().collect();
    fn m(p: &[char], s: &[char]) -> bool {
        match p.first() {
            None => s.is_empty(),
            Some('*') => (0..=s.len()).any(|i| m(&p[1..], &s[i..])),
            Some('?') => !s.is_empty() && m(&p[1..], &s[1..]),
            Some(c) => s.first() == Some(c) && m(&p[1..], &s[1..]),
        }
    }
    m(&p, &s)
}

/// `findCandidateDummyCells`: the buffer list, then clock inverters — by attribute, else named
/// CLKINV, else INV, else any — sorted by input capacitance.
fn find_candidate_dummy_cells(log: &mut Log, d: &dyn TreeDb, o: &WOptions) -> Res<Vec<Candidate>> {
    let mut out: Vec<Candidate> = Vec::new();
    for b in &o.buffer_list {
        if let Some(c) = d.liberty_cell(b) {
            out.push(Candidate { cell: b.clone(), cap: cell_input_cap(c) });
        }
    }
    let candidate = |c: &vyges_sta::liberty::Cell| !c.dont_use && !d.resizer_dont_use_cell(&c.name) && !c.always_on && !c.is_isolation_cell && !c.is_level_shifter;
    let libs = d.liberty_libraries();
    let mut inverters: Vec<Candidate> = Vec::new();
    let pass = |inverters: &mut Vec<Candidate>, keep: &dyn Fn(&vyges_sta::liberty::Cell) -> bool| {
        for lib in libs {
            for c in lib.cells.values() {
                if c.is_inverter() && keep(c) && candidate(c) {
                    inverters.push(Candidate { cell: c.name.clone(), cap: cell_input_cap(c) });
                }
            }
        }
    };
    pass(&mut inverters, &|c| c.is_clock_cell);
    if inverters.is_empty() {
        pass(&mut inverters, &|c| glob_nocase("*CLKINV*", &c.name));
    }
    if inverters.is_empty() {
        pass(&mut inverters, &|c| glob_nocase("*INV*", &c.name));
    }
    if inverters.is_empty() {
        pass(&mut inverters, &|_| true);
    }
    out.extend(inverters);
    // Ascending input cap; equal caps fall back to the cells' read order, which is not modelled.
    for i in 0..out.len() {
        for j in i + 1..out.len() {
            if out[i].cap == out[j].cap && out[i].cell != out[j].cell {
                return refuse(format!("dummy-load candidates {} and {} tie on input cap (cell read order is not modelled)", out[i].cell, out[j].cell));
            }
        }
    }
    out.sort_by(|a, b| a.cap.partial_cmp(&b.cap).expect("finite"));
    if log.debug_on("CTS", "dummy load", 1) {
        for c in &out {
            log.debug("CTS", "dummy load", 1, || format!("  {} is a dummy cell candidate with input cap={}", c.cell, crate::log::fmt_e(f64::from(c.cap), 3)));
        }
    }
    Ok(out)
}

/// `insertDummyCell`: the candidate whose cap best fills the gap, placed on the buffer and loading
/// its output net. False when the buffer's subnet is already at the fanout limit.
#[allow(clippy::too_many_arguments)]
fn insert_dummy_cell(log: &mut Log, d: &mut dyn TreeDb, w: &mut Writer, o: &WOptions, t: &mut Tree, b: usize, driver_subnet: &HashMap<usize, usize>, candidates: &[Candidate], cap: f32, ideal: f32, _out_cap: &mut HashMap<usize, f32>) -> Res<bool> {
    let Some(&sub) = driver_subnet.get(&b) else {
        return log.error("CTS", 120, format!("Subnet was not found for clock buffer {}.", t.clock.buffers[b].name));
    };
    if (t.clock.subnets[sub].insts.len() - 1) as u32 == o.max_fanout {
        return Ok(false);
    }
    let delta = ideal - cap;
    let caps: Vec<f32> = candidates.iter().map(|c| c.cap).collect();
    let Some(best) = find_best_dummy_cell(&caps, delta).map(|i| &candidates[i]) else { return refuse("no dummy-load candidate") };
    log.debug("CTS", "dummy load", 1, || format!("insertDummyCell {} at {}", t.clock.buffers[b].name, best.cell));
    // `placeDummyCell`: the database instance is `clkload<n>`; the tree's copy gets the net suffix.
    let name = format!("clkload{}", w.dummy_load_index);
    w.dummy_load_index += 1;
    let (x, y) = (t.clock.buffers[b].x, t.clock.buffers[b].y);
    d.create_placed_inst(&best.cell, &name, x, y)?;
    t.clock.buffers.push(crate::htree::ClockInst { name: format!("{name}_{}", t.clock.name), master: best.cell.clone(), x, y, insertion_delay: 0.0, input_cap: 0.0 });
    let dummy = t.clock.buffers.len() - 1;
    log.debug("CTS", "dummy load", 1, || format!("  placed dummy instance {name} at ( {x} {y} )"));
    // `connectDummyCell`.
    let src = t.clock.buffers[b].name.clone();
    let out = d.inst_first_output(&src).ok_or_else(|| crate::log::Stop::Refused(format!("{src}: no output")))?;
    let net = d.iterm_net(&out).ok_or_else(|| crate::log::Stop::Refused(format!("{src}: output unconnected")))?;
    let input = first_input(d, &name).ok_or_else(|| crate::log::Stop::Refused(format!("{name}: no input")))?;
    d.connect(&input, &net)?;
    t.clock.subnets[sub].insts.push(InstRef::Buffer(dummy));
    Ok(true)
}

/// `findBestDummyCell`: the candidate whose input cap is nearest the gap — the FIRST on a tie.
fn find_best_dummy_cell(caps: &[f32], delta: f32) -> Option<usize> {
    let mut min_diff = f32::MAX;
    let mut best = None;
    for (i, &c) in caps.iter().enumerate() {
        let diff = (c - delta).abs();
        if diff < min_diff {
            min_diff = diff;
            best = Some(i);
        }
    }
    best
}

/// A subnet's sinks are connected while every one so far was found and placed. ⛔ The flag is
/// sticky: once one sink is missing (`None`) or unplaced, no LATER sink of the net is connected
/// either (the net is then removed).
fn still_connecting(so_far: bool, placed: Option<bool>) -> bool {
    so_far && placed == Some(true)
}

/// What `countSinksPostDbWrite` accumulates.
#[derive(Default)]
struct Count {
    sinks: u32,
    leaf_sinks: u32,
    sink_wire_length: f64,
    min_depth: i32,
    max_depth: i32,
}

/// `isAnyTreeBuffer(getClockFromInst(inst))` for tree `ti`: one of its own level or leaf buffers.
fn is_any_tree_buffer(w: &Writer, trees: &[Tree], ti: usize, inst: &str) -> bool {
    w.inst_to_buffer.get(inst).is_some_and(|&(t, b)| t == ti && (trees[ti].tree_buffers.contains(&b) || trees[ti].leaf_buffers.contains(&b)))
}

fn is_leaf_buffer(w: &Writer, trees: &[Tree], ti: usize, inst: &str) -> bool {
    w.inst_to_buffer.get(inst).is_some_and(|&(t, b)| t == ti && trees[ti].leaf_buffers.contains(&b))
}

/// `countSinksPostDbWrite`: walk the written tree from a net, counting sinks, leaf buffers, the
/// Manhattan wire length to each sink, and the depth range.
#[allow(clippy::too_many_arguments)]
fn count_sinks_post_db_write(log: &mut Log, d: &mut dyn TreeDb, w: &Writer, trees: &[Tree], ti: usize, dbu: i32, net: &str, c: &mut Count, curr_wl: u32, depth: i32, full_tree: bool, sinks: &HashSet<ITerm>, dummies: &HashSet<String>, visited: &mut HashSet<String>) -> Res<()> {
    if d.net_sig_type(net) != "CLOCK" {
        return log.error("CTS", 369, format!("Unexpected data net '{net}' found during clock tree traversal"));
    }
    if !visited.insert(net.to_string()) {
        return Ok(());
    }
    let iterms = d.net_iterms(net);
    let (mut dx, mut dy) = (0i32, 0i32);
    for t in &iterms {
        if d.iterm_io_type(t) != "INPUT" {
            let (at, msgs) = d.iterm_avg_xy_logged(t);
            for m in msgs {
                log.report(m);
            }
            if let Some((x, y)) = at {
                (dx, dy) = (x, y);
            }
            break;
        }
    }
    if let Some((x, y)) = d.net_input_port_xy(net) {
        (dx, dy) = (x, y);
    }
    let tree = &trees[ti];
    for t in &iterms {
        if d.iterm_io_type(t) != "INPUT" {
            continue;
        }
        let inst = &t.0;
        let (at, msgs) = d.iterm_avg_xy_logged(t);
        for m in msgs {
            log.report(m);
        }
        // An unplaced pin leaves the receiver at the origin (`getAvgXY` reports and writes 0, 0).
        let (rx, ry) = at.unwrap_or((0, 0));
        let dist = ((dx - rx).unsigned_abs()).wrapping_add((dy - ry).unsigned_abs());
        let mut terminate = if full_tree { sinks.contains(t) } else { !is_any_tree_buffer(w, trees, ti, inst) };
        let mut true_sink = true;
        if tree.tree_type != TreeType::Register && depth == 0 && tree.top_buffer_name != *inst {
            terminate = true;
            true_sink = false;
        }
        let output = d.inst_first_output(inst);
        let out_net = output.as_ref().and_then(|o| d.iterm_net(o));
        if out_net.as_deref() == Some(net) {
            terminate = true;
            true_sink = false;
        }
        if !terminate {
            match &out_net {
                Some(n) => count_sinks_post_db_write(log, d, w, trees, ti, dbu, n, c, curr_wl.wrapping_add(dist), depth + 1, full_tree, sinks, dummies, visited)?,
                None => {
                    let master = d.inst_master(inst);
                    let kind = match d.liberty_cell(&master) {
                        Some(cell) if cell.is_inverter() => "Inverter",
                        Some(cell) if cell.is_buffer() => "Buffer",
                        _ => "Complex cell",
                    };
                    if !dummies.contains(inst) {
                        log.info("CTS", 121, format!("{kind} '{inst}' has unconnected output pin."));
                    }
                }
            }
            if is_leaf_buffer(w, trees, ti, inst) {
                c.leaf_sinks += 1;
            }
        } else if true_sink {
            c.sinks += 1;
            c.sink_wire_length += f64::from(dist.wrapping_add(curr_wl)) / f64::from(dbu);
            c.max_depth = c.max_depth.max(depth);
            if (c.min_depth > 0 && depth < c.min_depth) || c.min_depth == 0 {
                c.min_depth = depth;
            }
        }
    }
    Ok(())
}

/// One tree's nets and their levels (`ClockTreeLevels`).
struct TreeLevels {
    nets: Vec<(String, i32)>,
    max_level: i32,
}

/// `computeClockTreeLevels`: each tree walked breadth first from its root net; a level is a
/// branching (or a gater into a sub-tree), not a buffer stage; leaf nets are left out.
fn compute_clock_tree_levels(log: &mut Log, d: &dyn TreeDb, trees: &[Tree]) -> Vec<TreeLevels> {
    // Every net of a built tree → its leaf flag (`None`: a tree's input net).
    let mut tree_nets: HashMap<String, Option<bool>> = HashMap::new();
    let mut sub_tree_roots: HashSet<String> = HashSet::new();
    for t in trees {
        tree_nets.entry(t.net.clone()).or_insert(None);
        sub_tree_roots.insert(t.net.clone());
        for (s, sub) in t.clock.subnets.iter().enumerate() {
            if !t.removed_subnets.contains(&s) {
                tree_nets.insert(sub.name.clone(), Some(sub.leaf));
            }
        }
    }
    let mut out = Vec::new();
    let mut visited: HashSet<String> = HashSet::new();
    let walk = |root: &str, visited: &mut HashSet<String>, out: &mut Vec<TreeLevels>| {
        let mut net_levels: Vec<(String, i32)> = Vec::new();
        let mut feeds_sub_tree: HashSet<String> = HashSet::new();
        let mut queue: std::collections::VecDeque<(String, i32)> = std::collections::VecDeque::new();
        queue.push_back((root.to_string(), 0));
        visited.insert(root.to_string());
        while let Some((net, level)) = queue.pop_front() {
            net_levels.push((net.clone(), level));
            // The set is unordered in the reference; a tree reaches each net once, so the levels
            // do not depend on it. Kept in first-seen order here.
            let mut child_nets: Vec<String> = Vec::new();
            for t in d.net_iterms(&net) {
                if !d.iterm_is_input_signal(&t, true) {
                    continue;
                }
                for o in d.inst_iterms(&t.0) {
                    if !d.iterm_is_output_signal(&o, true) {
                        continue;
                    }
                    let Some(out_net) = d.iterm_net(&o) else { continue };
                    if !tree_nets.contains_key(&out_net) {
                        continue;
                    }
                    if sub_tree_roots.contains(&out_net) {
                        feeds_sub_tree.insert(net.clone());
                    }
                    if !child_nets.contains(&out_net) {
                        child_nets.push(out_net);
                    }
                }
            }
            let root_trunk = sub_tree_roots.contains(&net);
            let n_children = child_nets.len();
            for child in child_nets {
                let one_level_down = !root_trunk && (n_children > 1 || sub_tree_roots.contains(&child));
                if visited.insert(child.clone()) {
                    queue.push_back((child, if one_level_down { level + 1 } else { level }));
                }
            }
        }
        let mut tree = TreeLevels { nets: Vec::new(), max_level: 0 };
        for (net, level) in net_levels {
            if tree_nets.get(&net).copied().flatten() == Some(true) && !feeds_sub_tree.contains(&net) {
                continue;
            }
            tree.max_level = tree.max_level.max(level);
            tree.nets.push((net, level));
        }
        if !tree.nets.is_empty() {
            out.push(tree);
        }
    };
    for t in trees {
        if t.parent.is_none() && !visited.contains(&t.net) {
            walk(&t.net, &mut visited, &mut out);
        }
    }
    for t in trees {
        if !visited.contains(&t.net) {
            log.debug("CTS", "ndr", 1, || format!("Clock net {} is not reachable from the root of its clock tree, numbering its levels from 0", t.net));
            walk(&t.net, &mut visited, &mut out);
        }
    }
    out
}

/// `createClockNDR`: `CTS_NDR_<n>` (the first free name), every frontside routing layer at its
/// default width and twice its default spacing (pitch less width).
fn create_clock_ndr(log: &mut Log, d: &mut dyn TreeDb) -> Res<String> {
    let mut i = 0;
    let name = loop {
        let name = format!("CTS_NDR_{i}");
        i += 1;
        if d.create_ndr(&name)? {
            break name;
        }
    };
    for (level, (layer, width, pitch, backside)) in d.routing_layers().into_iter().enumerate() {
        if backside {
            continue;
        }
        let space = pitch - width;
        if width <= 0 || space <= 0 {
            return log.error("CTS", 208, format!("Clock NDR settings for layer {layer}: defaultSpace: {space}, defaultWidth: {width}"));
        }
        d.set_ndr_layer(&name, &layer, width, 2 * space)?;
        log.debug("CTS", "ndr", 1, || format!("  NDR rule set to layer {} {layer} as space={} width={width} vs. default space={space} width={width}", level + 1, 2 * space));
    }
    Ok(name)
}

/// `writeClockNDRsToDb`: the rule on each tree's nets up to the strategy's level.
fn write_clock_ndrs_to_db(log: &mut Log, d: &mut dyn TreeDb, trees: &[Tree], strategy: Ndr) -> Res<()> {
    let levels = compute_clock_tree_levels(log, d, trees);
    let mut rule: Option<String> = None;
    let mut count = 0;
    for tree in &levels {
        let max_ndr_level = match strategy {
            Ndr::RootOnly => 0,
            Ndr::Half => (tree.max_level + 2) / 2 - 1,
            Ndr::Full => tree.max_level,
            Ndr::None => return Ok(()),
        };
        log.debug("CTS", "ndr", 1, || format!("Clock tree rooted at {} has {} levels without the leaf nets, applying NDR to levels 0-{max_ndr_level}", tree.nets[0].0, tree.max_level + 1));
        for (net, level) in &tree.nets {
            if *level > max_ndr_level {
                continue;
            }
            if rule.is_none() {
                rule = Some(create_clock_ndr(log, d)?);
            }
            d.set_net_ndr(net, rule.as_deref().expect("made"))?;
            count += 1;
            log.debug("CTS", "ndr", 1, || format!("Applied NDR to: {net} (level {level})"));
        }
    }
    if let Some(r) = rule {
        log.debug("CTS", "ndr", 1, || format!("Non-default rule {r} for double spacing has been applied to {count} clock nets"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`findBestDummyCell`): the nearest input cap to the gap wins; a tie keeps the first.
    #[test]
    fn best_dummy_cell_is_the_first_nearest() {
        assert_eq!(find_best_dummy_cell(&[1.0, 2.0, 4.0], 2.2), Some(1));
        assert_eq!(find_best_dummy_cell(&[1.0, 3.0], 2.0), Some(0), "1 and 3 are equally near 2: the first");
        assert_eq!(find_best_dummy_cell(&[], 2.0), None);
    }

    /// Rule (`writeClockNetsToDb`): the connecting flag is sticky — after a missing or unplaced
    /// sink, later placed sinks are not connected.
    #[test]
    fn an_unplaced_sink_stops_the_rest_of_the_net() {
        let mut on = true;
        let mut connected = Vec::new();
        for placed in [Some(true), Some(false), Some(true)] {
            on = still_connecting(on, placed);
            connected.push(on);
        }
        assert_eq!(connected, [true, false, false]);
        assert!(!still_connecting(true, None));
    }

    /// Rule (`sta::fuzzyEqual`): equal within 1e-6 relative, zero against 1e-15 absolute.
    #[test]
    fn caps_compare_fuzzily() {
        assert!(sta_fuzzy_equal(1.0e-15, 1.0e-15 * (1.0 + 1e-7)));
        assert!(!sta_fuzzy_equal(1.0e-15, 1.1e-15));
        assert!(sta_fuzzy_equal(0.0, 1e-16));
    }
}
