// SPDX-License-Identifier: Apache-2.0
//! The command and its sequence: `clock_tree_synthesis` (the Tcl proc: its options, in its order)
//! and `runTritonCts`, each stage its own function, in the reference's order.
//!
//! Modelled so far: the setup and the characterization, clock-net discovery, the run that finds
//! no clock net, and each tree's H-tree built in memory ([`Outcome::Built`]). Writing the trees
//! to the database is the next slice.

use crate::buffers::{self, BufferLibs};
use crate::discovery::{self, ClockDesign, Found, SdcClock, Setup};
use crate::log::{refuse, Log, Res};
use crate::options::{self, Masters, Options};

/// A rectangle `(x0, y0, x1, y1)` in DBU.
pub type Rect = (i32, i32, i32, i32);

/// What the design must answer for a run.
pub trait Design: ClockDesign + BufferLibs + crate::techchar::CharDesign + crate::write::TreeDb {
    /// `block->getCoreArea()` width and height, and DBU per micron.
    fn core_size(&self) -> (i32, i32);
    fn dbu(&self) -> i32;
    /// `wireSignalCapacitance` / `wireClkCapacitance` / `wireClkResistance` of the command scene
    /// (per meter).
    fn wire_signal_cap(&self) -> f64;
    fn wire_clk_cap(&self) -> f64;
    fn wire_clk_res(&self) -> f64;
    /// Whether a library of that name was read (`network_->findLibrary`).
    fn has_library(&self, name: &str) -> bool;
    /// `-clk_nets`: each name's net (`block->findNet`), `None` when one is missing.
    fn find_net(&self, name: &str) -> Option<String>;
    /// The SDC clocks, in the SDC's order.
    fn sdc_clocks(&self) -> Vec<SdcClock>;
    /// `Resizer::findMaxWireLength(false)` in meters: the least over the scenes with a signal wire
    /// resistance and the resizer's buffers; `-inf` when no scene has one.
    fn find_max_wire_length(&self) -> Res<f64>;
    /// The non-via shapes of an instance terminal (`dbITermShapeItr`), placed: `(x0, y0, x1, y1)`.
    fn iterm_shape_boxes(&self, inst: &str, pin: &str) -> Vec<(i32, i32, i32, i32)>;
    /// `block->getCoreArea()` as `(xMin, yMin, xMax, yMax)`.
    fn core_area(&self) -> (i32, i32, i32, i32);
    /// A master's width and height (DBU), `None` when absent.
    fn master_size(&self, name: &str) -> Option<(i32, i32)>;
    /// `mergeBlockages`: placed block instances (grown by hard halos) and hard placement
    /// blockages, bloated then shrunk by the margins: the blocks' largest width and height, and
    /// the merged regions' extents, in order.
    fn merge_blockages(&self, bloat_w: i32, bloat_h: i32) -> (u32, u32, Vec<Rect>);
    /// `getBufferFanoutLimit`: the least of the design's, the buffer output port's and cell's SDC
    /// fanout limits and the liberty port's (else the library default) max fanout; 0 for none.
    fn buffer_fanout_limit(&self, buffer: &str) -> Res<i32>;
}

struct M<'a>(&'a dyn Design);

impl Masters for M<'_> {
    fn has_master(&self, name: &str) -> bool {
        self.0.has_master(name)
    }
}

/// How a run ended.
#[derive(Debug)]
pub enum Outcome {
    /// No valid clock net (CTS-0082): the command's whole output.
    NoClockNets,
    /// Clock nets found and each tree built in memory, the writing not modelled for this run.
    Built(Found, Vec<crate::htree::Clock>),
    /// The trees written to the database and reported: the command's whole output.
    Done(Found),
}

/// The `clock_tree_synthesis` proc: obsolete-flag warnings, then each option in the proc's order,
/// then `run_triton_cts`.
pub fn clock_tree_synthesis(o: &mut Options, log: &mut Log, d: &mut dyn Design, args: &[String]) -> Res<Outcome> {
    let (keys, flags) = options::parse_key_args(log, "clock_tree_synthesis", args, options::CTS_KEYS, options::CTS_FLAGS)?;
    let flag = |f: &str| flags.iter().any(|x| x == f);
    if let Some(l) = keys.get("-library") {
        o.cts_library = Some(l.clone());
    }
    if flag("-post_cts_disable") {
        log.warn("CTS", 115, "-post_cts_disable is obsolete.");
    }
    o.other.insert("sink_clustering".into(), flag("-sink_clustering_enable").to_string());
    for k in ["-sink_clustering_size", "-sink_clustering_max_diameter", "-macro_clustering_size", "-macro_clustering_max_diameter"] {
        if let Some(v) = keys.get(k) {
            o.other.insert(k.into(), v.clone());
        }
    }
    if flag("-balance_levels") {
        log.warn("CTS", 132, "-balance_levels is obsolete.");
    }
    for k in ["-sink_clustering_levels", "-num_static_layers", "-distance_between_buffers", "-branching_point_buffers_distance", "-clustering_exponent", "-clustering_unbalance_ratio"] {
        if let Some(v) = keys.get(k) {
            o.other.insert(k.into(), v.clone());
        }
    }
    // `-buf_list`, else the list in force, else an inferred one.
    match keys.get("-buf_list") {
        Some(b) => buffers::set_buffer_list(o, log, d, b)?,
        None if o.buffer_list.is_empty() => buffers::set_buffer_list(o, log, d, "")?,
        None => {}
    }
    if let Some(v) = keys.get("-wire_unit") {
        o.other.insert("-wire_unit".into(), v.clone());
    }
    if let Some(n) = keys.get("-clk_nets") {
        // `setClockNets`: each name must be a net, else CTS-0056 from the proc.
        let names = options::words(n);
        let mut nets = Vec::new();
        for name in &names {
            match d.find_net(name) {
                Some(net) => nets.push(net),
                None => return log.error("CTS", 56, "Error when finding -clk_nets in DB."),
            }
        }
        o.clock_nets = nets;
        o.clock_nets_text = n.clone();
    }
    if let Some(v) = keys.get("-tree_buf") {
        o.other.insert("-tree_buf".into(), v.clone());
    }
    // Without `-root_buf` the buffers in force stay (none: inferred).
    if let Some(r) = keys.get("-root_buf") {
        options::set_root_buffer(o, log, &M(d), r)?;
    }
    o.sink_buffers = keys.get("-sink_clustering_buffer").map(|s| options::words(s)).unwrap_or_default();
    if let Some(v) = keys.get("-sink_buffer_max_cap_derate") {
        let v = options::num(v)?;
        if !(0.0..=1.0).contains(&v) {
            return log.error("CTS", 109, "sink_buffer_max_cap_derate needs to be between 0 and 1.0.");
        }
        o.sink_buffer_max_cap_derate = v;
        o.sink_buffer_max_cap_derate_set = true;
    }
    if let Some(v) = keys.get("-delay_buffer_derate") {
        if options::num(v)? < 0.0 {
            return log.error("CTS", 123, "delay_buffer_derate needs to be greater than or equal to 0.");
        }
        o.other.insert("-delay_buffer_derate".into(), v.clone());
    }
    if flag("-obstruction_aware") {
        log.warn("CTS", 128, "-obstruction_aware is obsolete.");
    }
    if flag("-no_obstruction_aware") {
        o.other.insert("obstruction_aware".into(), "false".into());
    }
    o.other.insert("dummy_load".into(), (!flag("-dont_use_dummy_load")).to_string());
    if let Some(v) = keys.get("-apply_ndr") {
        o.other.insert("-apply_ndr".into(), v.clone());
    }
    o.other.insert("repair_clock_nets".into(), flag("-repair_clock_nets").to_string());
    o.insertion_delay = !flag("-no_insertion_delay");
    run_triton_cts(o, log, d)
}

/// `runTritonCts`, as far as it is modelled.
pub fn run_triton_cts(o: &mut Options, log: &mut Log, d: &mut dyn Design) -> Res<Outcome> {
    let r = run_stages(o, log, d);
    // The end-of-run reset (`rootBuffers_`, `sinkBuffers_` cleared, the max wire length 0).
    o.root_buffers.clear();
    o.sink_buffers.clear();
    o.max_wl = Ok(0);
    r
}

/// The command's stages, in order.
fn run_stages(o: &mut Options, log: &mut Log, d: &mut dyn Design) -> Res<Outcome> {
    let (root, sink, mut lut, char_setup, max_fanout, delay_buffers) = setup_characterization(o, log, d)?;
    find_clock_roots(o, log);
    let found = populate_triton_cts(o, log, d, &root)?;
    if found.builders.is_empty() {
        log.warn("CTS", 82, "No valid clock nets in the design.");
        return Ok(Outcome::NoClockNets);
    }
    check_characterization(log, d, &lut)?;
    let ho = tree_options(o, d, &root, &sink, char_setup, max_fanout)?;
    let mut ho = ho;
    let (mut trees, legalizers) = build_clock_trees(log, d, &mut ho, &mut lut, &found)?;
    // `-apply_ndr` (default half); a value the command's type check rejects is refused.
    let ndr = match o.other.get("-apply_ndr") {
        None => crate::write::Ndr::Half,
        Some(v) => crate::write::Ndr::parse(v).ok_or_else(|| crate::log::Stop::Refused(format!("-apply_ndr {v}: not a strategy")))?,
    };
    let wo = crate::write::WOptions { dummy_load: o.other.get("dummy_load").is_none_or(|v| v == "true"), max_fanout: ho.max_fanout, buffer_list: o.buffer_list.clone(), db_units: d.dbu(), ndr };
    let mut w = crate::write::Writer::default();
    crate::write::write_data_to_db(log, d, &mut w, &wo, &mut trees)?;
    // `setAllClocksPropagated` prints nothing: the timer the balancer asks is built on the written
    // design with every non-virtual clock propagated and the placement parasitics re-estimated.
    if o.other.get("repair_clock_nets").is_some_and(|v| v == "true") {
        return refuse("-repair_clock_nets is not modelled");
    }
    let unit = f64::from(d.dbu());
    let bo = crate::latency::BOptions { delay_buffers: &delay_buffers, cap_per_dbu: d.wire_clk_cap() * 1e-6 / unit, res_per_dbu: d.wire_clk_res() * 1e-6 / unit, wire_segment_unit: lut.length_unit as i32 };
    crate::latency::balance_macro_register_latencies(log, d, &bo, &trees, &legalizers, o.insertion_delay)?;
    Ok(Outcome::Done(found))
}

/// `checkCharacterization`: every buffer master the segments use exists; CTS-0097 counts them.
fn check_characterization(log: &mut Log, d: &dyn Design, lut: &crate::techchar::Lut) -> Res<()> {
    let mut visited: std::collections::HashSet<&str> = std::collections::HashSet::new();
    for seg in &lut.segments {
        for m in &seg.buffer_masters {
            if !visited.contains(m.as_str()) {
                if d.has_master(m) {
                    visited.insert(m);
                } else {
                    return log.error("CTS", 81, format!("Buffer {m} is not in the loaded DB."));
                }
            }
        }
    }
    log.info("CTS", 97, format!("Characterization used {} buffer(s) types.", visited.len()));
    Ok(())
}

/// The options the tree reads, as the command and the setup left them.
fn tree_options(o: &Options, d: &dyn Design, root: &str, sink: &str, char_setup: (u32, f64, f64), limits: (i32, i32)) -> Res<crate::htree::HOptions> {
    let (unit, cap_per_dbu, sink_buffer_input_cap) = char_setup;
    let get = |k: &str| o.other.get(k).map(|v| options::num(v)).transpose();
    // `ord::microns_to_dbu`: rounded.
    let dbu_of = |k: &str| -> Res<Option<i32>> { Ok(get(k)?.map(|v| (v * f64::from(d.dbu())).round() as i32)) };
    let buffer_distance = dbu_of("-distance_between_buffers")?;
    let vertex_distance = dbu_of("-branching_point_buffers_distance")?;
    let (root_max_fanout, sink_max_fanout) = limits;
    // `setupCharacterization`: the root buffer's limit when below the leaf sinks (15); the sink
    // buffer's caps the clustering sizes and, when lower, the max fanout.
    let mut max_fanout = 0u32;
    if root_max_fanout != 0 && 15 > root_max_fanout as u32 {
        max_fanout = root_max_fanout as u32;
    }
    let mut size = get("-sink_clustering_size")?.map(|v| v as u32);
    let mut macro_size = get("-macro_clustering_size")?.map(|v| v as u32);
    if sink_max_fanout != 0 {
        // `limitSinkClusteringSizes` (sets the size, so it counts as given).
        let limit = sink_max_fanout as u32;
        size = Some(size.map_or(limit, |s| limit.min(s)));
        if (sink_max_fanout as u32) < max_fanout {
            max_fanout = sink_max_fanout as u32;
        }
    }
    let _ = &mut macro_size;
    let diameter = get("-sink_clustering_max_diameter")?;
    let macro_diameter = get("-macro_clustering_max_diameter")?;
    Ok(crate::htree::HOptions {
        root_buffer: root.into(),
        sink_buffer: sink.into(),
        tree_buffer: o.other.get("-tree_buf").cloned(),
        sink_clustering: o.other.get("sink_clustering").is_some_and(|v| v == "true"),
        max_diameter: diameter.unwrap_or(50.0),
        max_diameter_set: diameter.is_some(),
        sink_clustering_size: size.unwrap_or(30),
        sink_clustering_size_set: size.is_some(),
        macro_max_diameter: macro_diameter.unwrap_or(50.0),
        macro_max_diameter_set: macro_diameter.is_some(),
        macro_sink_clustering_size: macro_size.unwrap_or(4),
        macro_sink_clustering_size_set: true,
        num_static_layers: get("-num_static_layers")?.map_or(0, |v| v as u32),
        clustering_power: get("-clustering_exponent")?.map_or(4, |v| v as u32),
        clustering_capacity: get("-clustering_unbalance_ratio")?.unwrap_or(0.6),
        max_fanout,
        max_wl: o.max_wl.clone()?,
        wire_segment_unit: unit,
        db_units: d.dbu(),
        max_slew: 4,
        num_max_leaf_sinks: 15,
        clock_tree_max_depth: 100,
        fake_lut_entries: true,
        obstruction_aware: o.other.get("obstruction_aware").is_none_or(|v| v != "false"),
        buffer_distance: buffer_distance.unwrap_or(100 * d.dbu()),
        sink_buffer_input_cap,
        cap_per_dbu,
        sink_clustering_levels: get("-sink_clustering_levels")?.map_or(0, |v| v as u32),
        simple_segments: buffer_distance.is_some(),
        vertex_buffers: vertex_distance.is_some(),
        vertex_buffer_distance: vertex_distance.unwrap_or(240 * d.dbu()),
    })
}

/// `computeITermPosition`: the mean of the terminal's non-via shape centres (integer arithmetic).
fn iterm_position(d: &dyn Design, inst: &str, pin: &str) -> (i32, i32) {
    let (mut x, mut y, mut n) = (0i32, 0i32, 0i32);
    for (x0, y0, x1, y1) in d.iterm_shape_boxes(inst, pin) {
        x += x0 + (x1 - x0) / 2;
        y += y0 + (y1 - y0) / 2;
        n += 1;
    }
    if n > 0 {
        x /= n;
        y /= n;
    }
    (x, y)
}

/// `buildClockTrees`: each builder's blockages, then its tree.
#[allow(clippy::type_complexity)]
fn build_clock_trees(log: &mut Log, d: &dyn Design, ho: &mut crate::htree::HOptions, lut: &mut crate::techchar::Lut, found: &Found) -> Res<(Vec<crate::write::Tree>, Vec<crate::htree::Legalizer>)> {
    use crate::htree::{Clock, ClockInst, HDesign, HTree, TreeType};
    let mut master_size = std::collections::HashMap::new();
    for s in &lut.segments {
        for m in &s.buffer_masters {
            if let Some(sz) = d.master_size(m) {
                master_size.insert(m.clone(), sz);
            }
        }
    }
    for m in [&ho.root_buffer, &ho.sink_buffer].into_iter().chain(ho.tree_buffer.as_ref()) {
        if let Some(sz) = d.master_size(m) {
            master_size.insert(m.clone(), sz);
        }
    }
    let mut out = Vec::new();
    let mut legalizers = Vec::new();
    for b in &found.builders {
        let tree_type = match b.tree_type {
            Some("MacroTree") => TreeType::Macro,
            Some("RegisterTree") => TreeType::Register,
            _ => TreeType::Regular,
        };
        let mut clock = Clock::new(&b.net);
        for s in &b.sinks {
            let (inst, pin) = s.rsplit_once('/').ok_or_else(|| crate::log::Stop::Refused(format!("sink {s}: not inst/pin")))?;
            let (x, y) = iterm_position(d, inst, pin);
            let input_cap = crate::write::input_pin_cap(d, &(inst.to_string(), pin.to_string()));
            clock.sinks.push(ClockInst { name: s.clone(), master: String::new(), x, y, insertion_delay: 0.0, input_cap });
        }
        // `initBlockages`: the tree (else root) buffer's size sets the merge margins.
        let buffer = ho.tree_buffer.clone().unwrap_or_else(|| ho.root_buffer.clone());
        let blockages = match master_size.get(&buffer) {
            Some(&(w, h)) => {
                let unit = f64::from(lut.length_unit);
                let (bw, bh) = (f64::from(w) / unit, f64::from(h) / unit);
                let (max_dx, max_dy, rects) = d.merge_blockages((bw * unit / 2.0).ceil() as i32, (bh * unit / 2.0).ceil() as i32);
                // The macro clustering diameter becomes twice the largest block (microns) unless
                // given — and stays set for the trees after this one.
                let max_diameter = f64::from(2 * max_dx.max(max_dy)) / f64::from(d.dbu());
                if max_diameter != 0.0 && !ho.macro_max_diameter_set {
                    ho.macro_max_diameter = max_diameter.max(ho.macro_max_diameter);
                    ho.macro_max_diameter_set = true;
                }
                rects
            }
            None => Vec::new(),
        };
        let hd = HDesign { core: d.core_area(), blockages, master_size: master_size.clone() };
        let mut t = HTree::new(ho, &hd, lut, clock, tree_type);
        // The register tree's top buffer was named when its net was forked.
        if let Some(top) = &b.top_buffer {
            t.top_buffer_name = top.clone();
        }
        t.init_blockages(log)?;
        t.run(log)?;
        t.probe_tree(log);
        let driving_net = (tree_type == TreeType::Register).then(|| b.parent.map(|p| found.builders[p].net.clone())).flatten();
        let index = out.len();
        legalizers.push(t.legalizer());
        out.push(crate::write::Tree {
            sdc_clock: b.sdc_clock.clone(),
            top_input_net: b.input_net.clone(),
            net: b.net.clone(),
            tree_type,
            top_buffer_name: t.top_buffer_name.clone(),
            driving_net,
            parent: b.parent,
            has_children: found.builders.iter().any(|c| c.parent == Some(index)),
            tree_buffers: t.tree_level_buffers.iter().copied().collect(),
            leaf_buffers: t.first_level_sink_drivers.iter().chain(&t.second_level_sink_drivers).copied().collect(),
            n_dummies: 0,
            removed_subnets: Default::default(),
            clock: t.clock,
        });
    }
    Ok((out, legalizers))
}

/// `setupCharacterization`: the library check, the root and sink buffers, then the
/// characterization's opening (`TechChar::create` → `initCharacterization` up to the
/// characterization buffer). Returns the root and sink buffers.
#[allow(clippy::type_complexity)]
fn setup_characterization(o: &mut Options, log: &mut Log, d: &mut dyn Design) -> Res<(String, String, crate::techchar::Lut, (u32, f64, f64), (i32, i32), Vec<String>)> {
    if let Some(l) = &o.cts_library {
        if !d.has_library(l) {
            return log.error("CTS", 209, format!("Library {l} cannot be found because it is not loaded or name is incorrect"));
        }
        log.info("CTS", 210, format!("Clock buffers will be chosen from library {l}"));
    }
    let (core, dbu, cap) = (d.core_size(), d.dbu(), d.wire_signal_cap());
    let root = buffers::select_root_buffer(o, log, d, core, dbu, cap)?;
    let sink = buffers::select_sink_buffer(o, log, d, core, dbu, cap)?;
    // The buffers' fanout limits set options the trees read (`tree_options`).
    let sink_max_fanout = d.buffer_fanout_limit(&sink)?;
    let root_max_fanout = d.buffer_fanout_limit(&root)?;
    // The max wire length (DBU, `micronsToDbu` rounds): clock-gate cloning's threshold.
    o.max_wl = d.find_max_wire_length().map(|m| {
        let microns = m * 1e6;
        if microns > 0.0 {
            (microns * f64::from(d.dbu())).round() as i32
        } else {
            0
        }
    });
    let (lut, unit, delay_buffers) = init_characterization(o, log, d, &root, &sink)?;
    Ok((root, sink, lut, unit, (root_max_fanout, sink_max_fanout), delay_buffers))
}

/// `TechChar::initCharacterization`, to the characterization buffer.
/// Returns the table and (the characterization unit, capacitance per DBU, sink buffer input cap).
#[allow(clippy::type_complexity)]
fn init_characterization(o: &Options, log: &mut Log, d: &dyn Design, root: &str, sink: &str) -> Res<(crate::techchar::Lut, (u32, f64, f64), Vec<String>)> {
    init_clock_layer_res_cap(log, d);
    if o.buffer_list.is_empty() {
        return log.error("CTS", 73, "Buffer not found. Check your -buf_list input.");
    }
    buffers::finalize_root_sink_buffers(o, log, root, sink);
    let list = buffers::trim_sort_buffer_list(o, log, d, root, sink, o.buffer_list.clone())?;
    // "Runs after the buffer list sanity checks in trimSortBufferList".
    let delay_buffers = buffers::create_delay_buf_list(o, log, d)?;
    let (char_buf, slews, loads) = buffers::characterization_buffer(log, d, &list)?;
    // The rest of `initCharacterization`, then the patterns and the table (`TechChar::create`).
    let wire_unit = match o.other.get("-wire_unit") {
        None => 0,
        Some(v) => v.trim().parse::<f64>().map_err(|_| crate::log::Stop::Refused(format!("-wire_unit {v}: not a number")))? as u32,
    };
    let co = crate::techchar::CharOptions { wire_unit, max_char_slew: 0.0, max_char_cap: 0.0, slew_steps: 7, cap_steps: 20, wirelength_iterations: 4, fake_lut_entries: true };
    let w = crate::techchar::ClockWire { dbu: d.dbu(), cap: d.wire_clk_cap(), res: d.wire_clk_res() };
    let setup = crate::techchar::init_characterization(log, d, w, &co, list.clone(), &char_buf, sink, slews.clone(), loads.clone())?;
    let mut probe = format!("VYGTCI|{:016x}|{:016x}|{}|{}|{}|", setup.cap_per_dbu.to_bits(), setup.res_per_dbu.to_bits(), setup.unit, setup.char_buf, setup.masters.join(","));
    probe += &setup.wirelengths.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(",");
    probe += "|";
    probe += &setup.slews.iter().map(|x| format!("{:08x}", x.to_bits())).collect::<Vec<_>>().join(",");
    probe += "|";
    probe += &setup.loads.iter().map(|x| format!("{:08x}", x.to_bits())).collect::<Vec<_>>().join(",");
    probe += &format!("|{:08x}|{:08x}", setup.slew_step.to_bits(), setup.cap_step.to_bits());
    log.trace(probe);
    let (lut, _) = crate::techchar::create(log, d, &setup)?;
    crate::techchar::probe_segments(log, &lut);
    Ok((lut, (setup.unit, setup.cap_per_dbu, setup.sink_buffer_input_cap), delay_buffers))
}

/// `TechChar::initClockLayerResCap`: CTS-0104 when the clock wire's resistance or capacitance per
/// DBU is zero.
fn init_clock_layer_res_cap(log: &mut Log, d: &dyn Design) {
    let dbu = d.dbu() as f32;
    let cap = (d.wire_clk_cap() * 1e-6 / f64::from(dbu)) as f32;
    let res = (d.wire_clk_res() * 1e-6 / f64::from(dbu)) as f32;
    if res == 0.0 || cap == 0.0 {
        log.warn("CTS", 104, "Clock wire resistance/capacitance values are zero.\nUse set_wire_rc to set them.");
    }
}

/// `findClockRoots()`: only reports user roots (CTS-0001).
fn find_clock_roots(o: &Options, log: &mut Log) {
    if !o.clock_nets.is_empty() {
        log.info("CTS", 1, format!("Running TritonCTS with user-specified clock roots: {}.", o.clock_nets_text));
    }
}

/// `populateTritonCTS`.
fn populate_triton_cts(o: &Options, log: &mut Log, d: &mut dyn Design, root: &str) -> Res<Found> {
    let setup = Setup { clock_nets: &o.clock_nets, skip_nets: &o.skip_nets, insertion_delay: o.insertion_delay, buffer_list: &o.buffer_list, root_buffer: root, max_wl: o.max_wl.clone() };
    let clocks = d.sdc_clocks();
    discovery::populate_triton_cts(d, log, &setup, &clocks)
}

/// The refusal for a run that reaches what is not modelled yet.
pub fn beyond_slice_1<T>() -> Res<T> {
    refuse("characterization and tree building (cts slices 2+) are not modelled")
}
