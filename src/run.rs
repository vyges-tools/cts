// SPDX-License-Identifier: Apache-2.0
//! The command and its sequence: `clock_tree_synthesis` (the Tcl proc: its options, in its order)
//! and `runTritonCts`, each stage its own function, in the reference's order.
//!
//! Modelled so far (slice 1): the setup up to the characterization buffer, clock-net discovery,
//! and the run that finds no clock net. A run that finds one stops after discovery
//! ([`Outcome::Discovered`]): characterization and tree building are later slices.

use crate::buffers::{self, BufferLibs};
use crate::discovery::{self, ClockDesign, Found, SdcClock, Setup};
use crate::log::{refuse, Log, Res};
use crate::options::{self, Masters, Options};

/// What the design must answer for a run.
pub trait Design: ClockDesign + BufferLibs {
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
    /// Clock nets found; the run stops here (later slices build the trees).
    Discovered(Found),
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
    let (root, sink) = setup_characterization(o, log, d)?;
    find_clock_roots(o, log);
    let found = populate_triton_cts(o, log, d, &root)?;
    let _ = sink; // read by the trees (later slices)
    // The end-of-run reset (`rootBuffers_`, `sinkBuffers_` cleared, the max wire length 0).
    o.root_buffers.clear();
    o.sink_buffers.clear();
    o.max_wl = Ok(0);
    if found.builders.is_empty() {
        log.warn("CTS", 82, "No valid clock nets in the design.");
        return Ok(Outcome::NoClockNets);
    }
    Ok(Outcome::Discovered(found))
}

/// `setupCharacterization`: the library check, the root and sink buffers, then the
/// characterization's opening (`TechChar::create` → `initCharacterization` up to the
/// characterization buffer). Returns the root and sink buffers.
fn setup_characterization(o: &mut Options, log: &mut Log, d: &mut dyn Design) -> Res<(String, String)> {
    if let Some(l) = &o.cts_library {
        if !d.has_library(l) {
            return log.error("CTS", 209, format!("Library {l} cannot be found because it is not loaded or name is incorrect"));
        }
        log.info("CTS", 210, format!("Clock buffers will be chosen from library {l}"));
    }
    let (core, dbu, cap) = (d.core_size(), d.dbu(), d.wire_signal_cap());
    let root = buffers::select_root_buffer(o, log, d, core, dbu, cap)?;
    let sink = buffers::select_sink_buffer(o, log, d, core, dbu, cap)?;
    // The buffers' fanout limits set options the trees read (later).
    // The max wire length (DBU, `micronsToDbu` rounds): clock-gate cloning's threshold.
    o.max_wl = d.find_max_wire_length().map(|m| {
        let microns = m * 1e6;
        if microns > 0.0 {
            (microns * f64::from(d.dbu())).round() as i32
        } else {
            0
        }
    });
    init_characterization(o, log, d, &root, &sink)?;
    Ok((root, sink))
}

/// `TechChar::initCharacterization`, to the characterization buffer.
fn init_characterization(o: &Options, log: &mut Log, d: &dyn Design, root: &str, sink: &str) -> Res<(String, Vec<f32>, Vec<f32>)> {
    init_clock_layer_res_cap(log, d);
    if o.buffer_list.is_empty() {
        return log.error("CTS", 73, "Buffer not found. Check your -buf_list input.");
    }
    buffers::finalize_root_sink_buffers(o, log, root, sink);
    let list = buffers::trim_sort_buffer_list(o, log, d, root, sink, o.buffer_list.clone())?;
    // "Runs after the buffer list sanity checks in trimSortBufferList".
    let _delay_buffers = buffers::create_delay_buf_list(o, log, d)?;
    buffers::characterization_buffer(log, d, &list)
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
