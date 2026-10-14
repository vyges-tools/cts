// SPDX-License-Identifier: Apache-2.0
//! The characterization (`TechChar::create`): every buffer/wire topology of every characterized
//! wirelength timed on a scratch netlist at every load and input slew, the results grouped,
//! normalized and compiled into the wire-segment table the tree builder reads.
//!
//! The scratch netlist is timed by this programme's own timer (`vyges-sta`), with each net's pi
//! model and Elmore delay SET, as the reference sets them, rather than estimated.
//!
//! ⛔ **Types are the reference's.** The per-DBU wire RC is `double`; a pi model's values and
//! every result are `float`; the step sizes are `float`; the maximum characterized slew and cap are
//! `double` options. Each conversion happens where the reference's does.

use std::collections::{BTreeMap, HashMap};

use vyges_sta::graph::{Graph, NetParasitics};
use vyges_sta::liberty::Library;
use vyges_sta::netlist::{Conn, Net, Netlist, PortDir};
use vyges_sta::parasitics::PiElmore;
use vyges_sta::sdc::Sdc;
use vyges_sta::search::Search;

use crate::buffers::BufferLibs;
use crate::log::{refuse, Log, Res};

/// What the characterization reads of the design beyond the libraries.
pub trait CharDesign: BufferLibs {
    /// `dbMaster::getHeight` (DBU).
    fn master_height(&self, name: &str) -> i32;
}

/// The block's DBU and the command scene's clock wire RC (per meter).
#[derive(Debug, Clone, Copy)]
pub struct ClockWire {
    pub dbu: i32,
    pub cap: f64,
    pub res: f64,
}

/// `TechChar::initCharacterization`'s results.
#[derive(Debug, Clone)]
pub struct Setup {
    pub cap_per_dbu: f64,
    pub res_per_dbu: f64,
    pub unit: u32,
    pub length_unit: u32,
    pub char_buf: String,
    pub buf_in: String,
    pub buf_out: String,
    /// `masterNames_`: the trimmed buffer list.
    pub masters: Vec<String>,
    pub wirelengths: Vec<u32>,
    pub slews: Vec<f32>,
    pub loads: Vec<f32>,
    pub slew_step: f32,
    pub cap_step: f32,
    pub max_char_slew: f64,
    pub max_char_cap: f64,
    pub sink_buffer_input_cap: f64,
}

/// One wire segment of the table.
#[derive(Debug, Clone, PartialEq)]
pub struct WireSegment {
    pub length: u8,
    pub load: u8,
    pub output_slew: u8,
    pub power: f64,
    pub delay: u32,
    pub input_cap: u8,
    pub input_slew: u8,
    pub buffer_locations: Vec<f64>,
    pub buffer_masters: Vec<String>,
    pub wl2_first_buffer: i32,
    pub last_wl: i32,
}

/// The compiled table (`TechChar` after `create`).
#[derive(Debug, Clone, Default)]
pub struct Lut {
    pub segments: Vec<WireSegment>,
    pub key_to_segments: HashMap<u32, Vec<usize>>,
    pub length_unit: u32,
    pub length_unit_ratio: u32,
    pub min_segment_length: u32,
    pub max_segment_length: u32,
    pub min_capacitance: u32,
    pub max_capacitance: u32,
    pub min_slew: u32,
    pub max_slew: u32,
    pub actual_min_input_cap: u32,
}

const NUM_BITS_PER_FIELD: u32 = 10;
const MAX_NORMALIZED_VAL: u32 = (1 << NUM_BITS_PER_FIELD) - 1;

impl Lut {
    /// `computeKey`.
    pub fn key(length: u8, load: u8, output_slew: u8) -> u32 {
        u32::from(length) | (u32::from(load) << NUM_BITS_PER_FIELD) | (u32::from(output_slew) << (2 * NUM_BITS_PER_FIELD))
    }
}

/// The characterization options (`configure_cts_characterization`, `-wire_unit`).
#[derive(Debug, Clone, Default)]
pub struct CharOptions {
    /// `-wire_unit` (microns); 0: the characterization buffer's height × 10.
    pub wire_unit: u32,
    pub max_char_slew: f64,
    pub max_char_cap: f64,
    pub slew_steps: u32,
    pub cap_steps: u32,
    pub wirelength_iterations: u32,
    pub fake_lut_entries: bool,
}

/// `utl::sort_and_unique` then the 1 % neighbour filter (`sortAndUniquify`).
fn sort_and_uniquify(values: &mut Vec<f32>) {
    values.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    values.dedup();
    let mut i = 1;
    let mut prev = values[0];
    while i < values.len() {
        let curr = values[i];
        if (prev - curr).abs() / curr < 0.01 {
            values.remove(i);
        } else {
            i += 1;
            prev = curr;
        }
    }
}

/// `smallestDiffIter`: the later element of the closest neighbouring pair (first on ties).
fn smallest_diff(values: &[f32]) -> usize {
    let mut min = f32::MAX;
    let mut at = 0;
    for i in 1..values.len() {
        let d = (values[i] - values[i - 1]).abs();
        if d < min {
            min = d;
            at = i;
        }
    }
    at
}

/// `largestDiffIter`: the earlier element of the widest neighbouring pair (first on ties).
fn largest_diff(values: &[f32]) -> usize {
    let mut max = f32::MIN_POSITIVE;
    let mut at = 0;
    for i in 1..values.len() {
        let d = (values[i] - values[i - 1]).abs();
        if d > max {
            max = d;
            at = i - 1;
        }
    }
    at
}

/// `reduceOrExpand`: drop the element of the closest pair, or insert the midpoint (in double) of
/// the widest, until there are `limit`.
fn reduce_or_expand(values: &mut Vec<f32>, limit: usize) {
    if values.len() > limit {
        while values.len() != limit {
            let i = smallest_diff(values);
            values.remove(i);
        }
    } else if values.len() < limit {
        while values.len() != limit {
            let i = largest_diff(values);
            let v = ((f64::from(values[i]) + f64::from(values[i + 1])) / 2.0) as f32;
            values.insert(i + 1, v);
        }
    }
}

/// `initCharacterization` after the characterization buffer is chosen: the clock wire RC per
/// DBU, the buffer's ports, the wirelengths, the length unit, the slew and cap limits and the
/// slews and loads to test.
#[allow(clippy::too_many_arguments)]
pub fn init_characterization(log: &mut Log, d: &dyn CharDesign, w: ClockWire, co: &CharOptions, masters: Vec<String>, char_buf: &str, sink: &str, mut axis_slews: Vec<f32>, mut axis_loads: Vec<f32>) -> Res<Setup> {
    let dbu = w.dbu as f32;
    let cap_per_dbu = w.cap * 1e-6 / f64::from(dbu);
    let res_per_dbu = w.res * 1e-6 / f64::from(dbu);
    let Some(cell) = d.liberty_cell(char_buf) else { return log.error("CTS", 96, format!("No Liberty cell found for {char_buf}.")) };
    let Some((input, output)) = cell.buffer_ports() else { return log.error("CTS", 534, format!("Could not find buffer input port for {char_buf}.")) };
    let (buf_in, buf_out) = (input.name.clone(), output.name.clone());
    let height = d.master_height(char_buf) as u32;
    let iterations = co.wirelength_iterations;
    let mut max_wirelength = (height * 10) * iterations;
    let unit = if co.wire_unit == 0 { height * 10 } else { (co.wire_unit as f32 * dbu) as u32 };
    if co.fake_lut_entries {
        max_wirelength = max_wirelength.max(2 * unit);
    }
    let mut wirelengths = Vec::new();
    let mut w = unit;
    while w <= max_wirelength && w <= iterations * unit {
        wirelengths.push(w);
        w += unit;
    }
    if wirelengths.is_empty() {
        return log.error("CTS", 75, "Error generating the wirelengths to test.\n    Check the -wire_unit parameter or the technology files.");
    }
    let length_unit = height * 10 / 2;
    // The limits, from the characterization buffer's liberty unless given.
    let (max_slew, max_cap, max_char_slew, max_char_cap, sink_cap);
    if co.max_char_slew == 0.0 || co.max_char_cap == 0.0 {
        let lib = d.libraries().iter().find(|l| l.cells.contains_key(char_buf));
        let slew = output.max_transition.or(input.max_transition).or_else(|| lib.and_then(|l: &Library| l.default_max_transition));
        let Some(slew) = slew else { return log.error("CTS", 107, format!("No max slew found for cell {char_buf}.")) };
        let Some(cap) = output.max_capacitance else { return log.error("CTS", 108, format!("No max capacitance found for cell {char_buf}.")) };
        max_slew = slew;
        max_cap = cap;
        max_char_slew = f64::from(slew);
        max_char_cap = f64::from(cap);
        let Some(sc) = d.liberty_cell(sink) else { return log.error("CTS", 76, format!("No Liberty cell found for {sink}.")) };
        let Some((si, _)) = sc.buffer_ports() else { return refuse(format!("sink buffer {sink} has no buffer ports")) };
        sink_cap = f64::from(si.capacitance.iter().flatten().fold(f32::MIN, |a, &b| a.max(b)));
    } else {
        max_slew = co.max_char_slew as f32;
        max_cap = co.max_char_cap as f32;
        max_char_slew = co.max_char_slew;
        max_char_cap = co.max_char_cap;
        sink_cap = 0.0;
    }
    if !axis_slews.is_empty() {
        axis_slews.push(max_slew);
        sort_and_uniquify(&mut axis_slews);
    }
    if !axis_loads.is_empty() {
        axis_loads.push(max_cap);
        sort_and_uniquify(&mut axis_loads);
    }
    let slew_step = max_slew / co.slew_steps as f32;
    let cap_step = max_cap / co.cap_steps as f32;
    let (slews, loads) = if axis_loads.is_empty() && axis_slews.is_empty() {
        ((1..=co.slew_steps).map(|s| s as f32 * slew_step).collect(), (1..=co.cap_steps).map(|s| s as f32 * cap_step).collect())
    } else {
        reduce_or_expand(&mut axis_slews, co.slew_steps as usize);
        reduce_or_expand(&mut axis_loads, co.cap_steps as usize);
        (axis_slews, axis_loads)
    };
    if loads.is_empty() || slews.is_empty() {
        return log.error("CTS", 78, "Error generating the wirelengths to test.\n    Check the parameters -max_cap/-max_slew/-cap_inter/-slew_inter\n          or the technology files.");
    }
    Ok(Setup {
        cap_per_dbu,
        res_per_dbu,
        unit,
        length_unit,
        char_buf: char_buf.to_string(),
        buf_in,
        buf_out,
        masters,
        wirelengths,
        slews,
        loads,
        slew_step,
        cap_step,
        max_char_slew,
        max_char_cap,
        sink_buffer_input_cap: sink_cap,
    })
}

/// One topology (`SolutionData`): a chain from the input port through `buffers` (each after
/// `nodes_without_buf[k]` wire units) to the output port.
#[derive(Debug, Clone)]
struct Topology {
    /// Wire units before each buffer, then after the last (`nodesWithoutBufVector`).
    nodes_without_buf: Vec<u32>,
    /// Each buffer's current master.
    masters: Vec<String>,
    /// `topologyDescriptor`: lengths and masters as text.
    descriptor: Vec<String>,
    pure_wire: bool,
}

/// `createPatterns`: every subset of the wirelength's nodes as buffer positions (bit k = node k).
fn create_patterns(s: &Setup, wl: u32) -> Vec<Topology> {
    let nodes = wl / s.unit;
    let mut out = Vec::new();
    for bits in 0..(1u32 << nodes) {
        let mut t = Topology { nodes_without_buf: vec![], masters: vec![], descriptor: vec![], pure_wire: true };
        let mut without = 0u32;
        for node in 0..nodes {
            without += 1;
            if bits & (1 << node) != 0 {
                t.nodes_without_buf.push(without);
                t.masters.push(s.char_buf.clone());
                t.descriptor.push((without * s.unit).to_string());
                t.descriptor.push(s.char_buf.clone());
                without = 0;
                t.pure_wire = false;
            }
        }
        t.nodes_without_buf.push(without);
        if without != 0 {
            t.descriptor.push((without * s.unit).to_string());
        }
        out.push(t);
    }
    out
}

/// One pattern's results (`ResultData`).
#[derive(Debug, Clone)]
pub struct ResultData {
    pub wirelength: u32,
    pub topology: Vec<String>,
    pub pure_wire: bool,
    pub load: f32,
    pub in_slew: f32,
    pub total_power: f32,
    pub totalcap: f32,
    pub pin_arrival: f32,
    pub pin_slew: f32,
    /// The raw rise and fall slews (probes only).
    pub rise: f32,
    pub fall: f32,
}

/// The scratch netlist of one topology and each net's (first pin, last pin, wire units).
fn chain_netlist(s: &Setup, t: &Topology) -> (Netlist, Vec<(String, String, u32)>) {
    let mut nl = Netlist { insts: vec![], ports: vec![("in".into(), PortDir::Input), ("out".into(), PortDir::Output)], nets: vec![] };
    let mut ends = Vec::new();
    let mut from = Conn::Port(0);
    // ⛔ The scratch timer keeps the masters the topology was CREATED with: `swapMaster` on the
    // characterization block reaches the database (the descriptor, the input cap read through the
    // design's network) but not the timer built over that block, so every configuration is timed
    // with the characterization buffer.
    for k in 0..t.masters.len() {
        nl.insts.push((format!("buf{k}"), s.char_buf.clone()));
        let to = Conn::Inst(k, s.buf_in.clone());
        let (a, b) = (nl.pin_name(&from), nl.pin_name(&to));
        let pins = match &from {
            Conn::Port(_) => vec![to.clone(), from.clone()],
            _ => vec![from.clone(), to.clone()],
        };
        nl.nets.push(Net { name: format!("n{k}"), pins });
        ends.push((a, b, t.nodes_without_buf[k]));
        from = Conn::Inst(k, s.buf_out.clone());
    }
    let to = Conn::Port(1);
    let (a, b) = (nl.pin_name(&from), nl.pin_name(&to));
    let pins = match &from {
        Conn::Port(_) => vec![from.clone(), to.clone()],
        _ => vec![from.clone(), to.clone()],
    };
    let k = t.masters.len();
    nl.nets.push(Net { name: format!("n{k}"), pins });
    ends.push((a, b, t.nodes_without_buf[k]));
    (nl, ends)
}

/// `computeTopologyResults` on a timed graph.
#[allow(clippy::too_many_arguments)]
fn topology_results(s: &Setup, libs: &[Library], d: &dyn BufferLibs, t: &Topology, g: &Graph, out_v: usize, load: f32, in_slew: f32, wl: u32) -> Res<ResultData> {
    let incap = if t.pure_wire {
        (f64::from(load) + f64::from(wl) * s.cap_per_dbu) as f32
    } else {
        let length = t.descriptor[0].parse::<f64>().map_err(|_| crate::log::Stop::Refused("a topology length".into()))? as f32;
        let Some(cell) = d.liberty_cell(&t.masters[0]) else { return refuse(format!("{}: no liberty cell", t.masters[0])) };
        let Some(port) = cell.ports.iter().find(|p| p.name == s.buf_in) else { return refuse("the buffer input port") };
        let first_pin_cap = port.capacitance.iter().flatten().fold(f32::MIN, |a, &b| a.max(b));
        (f64::from(first_pin_cap) + f64::from(length) * s.cap_per_dbu) as f32
    };
    let _ = libs;
    let totalcap = (incap / s.cap_step).round() * s.cap_step;
    let sdc = Sdc { clocks: vec![], input_delays: vec![], output_delays: vec![], path_delays: vec![] };
    let mut search = Search::in_graph_order(g, &sdc);
    search.find_arrivals().map_err(crate::log::Stop::Refused)?;
    let arrival = search.paths[out_v].iter().find(|p| p.tag.rf == 1 && p.tag.mm == 1).map_or(0.0, |p| p.arrival);
    let (rise, fall) = (g.slew[out_v][0][1], g.slew[out_v][1][1]);
    let pin_slew = ((rise + fall) / 2.0 / s.slew_step).round() * s.slew_step;
    Ok(ResultData { wirelength: wl, topology: t.descriptor.clone(), pure_wire: t.pure_wire, load, in_slew, total_power: 0.0, totalcap, pin_arrival: arrival, pin_slew, rise, fall })
}

/// `getBufferingCombo`: the monotonic configurations among `buffers^nodes`.
fn buffering_combo(buffers: usize, nodes: usize) -> u32 {
    let total = buffers.pow(nodes as u32);
    let mut n = 0;
    for row in 0..total {
        let mut cur = row;
        let mut cfg = Vec::with_capacity(nodes);
        for _ in 0..nodes {
            cfg.push(cur % buffers);
            cur /= buffers;
        }
        if is_monotonic(&cfg) {
            n += 1;
        }
    }
    n
}

/// `isTopologyMonotonic`.
fn is_monotonic(row: &[usize]) -> bool {
    row.windows(2).all(|w| w[1] >= w[0])
}

/// `updateBufferTopologies`: the next monotonic configuration (a counter in base `masters`,
/// digit 0 least significant), each changed buffer swapped and its descriptor entry renamed.
fn update_buffer_topologies(s: &Setup, t: &mut Topology) {
    let ids: Vec<usize> = t.masters.iter().map(|m| s.masters.iter().position(|x| x == m).unwrap_or(s.masters.len())).collect();
    let nb = s.masters.len();
    let mut number: usize = ids.iter().enumerate().map(|(i, &d)| d * nb.pow(i as u32)).sum();
    let next = loop {
        number += 1;
        let mut n = number;
        let cfg: Vec<usize> = (0..ids.len())
            .map(|_| {
                let d = n % nb;
                n /= nb;
                d
            })
            .collect();
        if is_monotonic(&cfg) {
            break cfg;
        }
    };
    for (node, &id) in next.iter().enumerate() {
        let new = s.masters[id].clone();
        if new != t.masters[node] {
            t.masters[node] = new.clone();
            // `swapTopologyBuffer`: the node'th master entry of the descriptor.
            let mut counter = 0;
            for k in 0..t.descriptor.len() {
                if s.masters.contains(&t.descriptor[k]) {
                    if counter == node {
                        t.descriptor[k] = new.clone();
                        break;
                    }
                    counter += 1;
                }
            }
        }
    }
}

/// `CharKey` ordering: (load, wirelength, pinSlew, totalcap), as floats.
#[derive(Debug, Clone, Copy, PartialEq)]
struct CharKey(f32, f32, f32, f32);
impl Eq for CharKey {}
impl PartialOrd for CharKey {
    fn partial_cmp(&self, o: &CharKey) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for CharKey {
    fn cmp(&self, o: &CharKey) -> std::cmp::Ordering {
        let c = |a: f32, b: f32| a.partial_cmp(&b).unwrap_or(std::cmp::Ordering::Equal);
        c(self.0, o.0).then(c(self.1, o.1)).then(c(self.2, o.2)).then(c(self.3, o.3))
    }
}

/// `TechChar::create`: per wirelength, its patterns timed at every configuration, load and slew
/// (in that nesting), grouped by key; then post-processing and the table.
pub fn create(log: &mut Log, d: &dyn CharDesign, s: &Setup) -> Res<(Lut, Vec<ResultData>)> {
    let libs = d.libraries();
    let mut map: BTreeMap<CharKey, Vec<ResultData>> = BTreeMap::new();
    let mut all = Vec::new();
    for &wl in &s.wirelengths {
        // One timer per wirelength: its first update times everything (the patterns' inputs not
        // yet annotated, each last net at its own wire); every later one is incremental.
        for (topo, mut t) in create_patterns(s, wl).into_iter().enumerate() {
            let combos = buffering_combo(s.masters.len(), t.masters.len());
            if combos == 0 {
                continue;
            }
            let (nl, ends) = chain_netlist(s, &t);
            let mut g = Graph::build(libs, &nl).map_err(crate::log::Stop::Refused)?;
            // `setParasitics`: each net's pi model and its last pin's Elmore delay.
            let mut paras: HashMap<String, NetParasitics> = HashMap::new();
            for (k, (a, b, n)) in ends.iter().enumerate() {
                let wire_cap = f64::from(n * s.unit) * s.cap_per_dbu;
                let wire_res = f64::from(n * s.unit) * s.res_per_dbu;
                let pi = PiElmore { c2: (wire_cap / 2.0) as f32, rpi: wire_res as f32, c1: (wire_cap / 2.0) as f32, elmore: vec![(1, (wire_res * wire_cap) as f32)] };
                paras.insert(format!("n{k}"), NetParasitics { node_names: vec![a.clone(), b.clone()], pi: Some(pi), pi_rise_only: true, ..NetParasitics::default() });
            }
            let last = format!("n{}", ends.len() - 1);
            let base = paras[&last].pi.clone().expect("set");
            let out_v = g.vertices.iter().position(|v| v.name == "out").expect("the output port");
            let in_v = g.vertices.iter().position(|v| v.name == "in").expect("the input port");
            let last_drvr = g.vertices.iter().position(|v| v.name == ends[ends.len() - 1].0).expect("the last net's driver");
            let order = g.topo_order().map_err(crate::log::Stop::Refused)?;
            let index = g.pin_index();
            // The first topology's first update is the timer's first (full, with its settings);
            // every other topology was timed then, unannotated and at its base wire.
            let mut timed = topo != 0;
            if timed {
                g.find_delays(&paras, None).map_err(crate::log::Stop::Refused)?;
            }
            let mut left = combos;
            loop {
                for &load in &s.loads {
                    let p = paras.get_mut(&last).expect("the last net");
                    p.pi = Some(PiElmore { c2: base.c2, rpi: base.rpi, c1: base.c1 + load, elmore: vec![(1, base.rpi * (base.c1 + base.c2 + load))] });
                    let mut dirty = vec![last_drvr];
                    for &slew in &s.slews {
                        // `setAnnotatedSlew` on the input port's vertex, both transitions.
                        g.slew_annotated.insert(in_v, slew);
                        dirty.push(in_v);
                        if timed {
                            update_incremental(&mut g, &paras, &order, &index, &dirty)?;
                        } else {
                            g.find_delays(&paras, None).map_err(crate::log::Stop::Refused)?;
                            timed = true;
                        }
                        dirty.clear();
                        let r = topology_results(s, libs, d, &t, &g, out_v, load, slew, wl)?;
                        log.trace(format!(
                            "VYGTC|{}|{}|{:08x}|{:08x}|{:08x}|{:08x}|{:08x}|{:08x}|{:08x}",
                            wl,
                            r.topology.join(";"),
                            r.load.to_bits(),
                            r.in_slew.to_bits(),
                            r.pin_arrival.to_bits(),
                            r.rise.to_bits(),
                            r.fall.to_bits(),
                            r.totalcap.to_bits(),
                            r.pin_slew.to_bits()
                        ));
                        map.entry(CharKey(r.load, r.wirelength as f32, r.pin_slew, r.totalcap)).or_default().push(r.clone());
                        all.push(r);
                    }
                }
                if !t.pure_wire && combos > 1 {
                    update_buffer_topologies(s, &mut t);
                    // The swap reaches the network the pin capacitances are read through, not the
                    // timer's arcs, and invalidates nothing.
                    for (k, m) in t.masters.iter().enumerate() {
                        let Some(l) = libs.iter().position(|l| l.cells.contains_key(m)) else { return refuse(format!("{m}: no liberty cell")) };
                        let prefix = format!("buf{k}/");
                        for v in 0..g.vertices.len() {
                            if g.vertices[v].name.starts_with(&prefix) {
                                g.pin_cap_cell.insert(v, (l, m.clone()));
                            }
                        }
                    }
                }
                left -= 1;
                if left == 0 {
                    break;
                }
            }
        }
    }
    let lut = compile_lut(log, s, &post_process(s, &map))?;
    Ok((lut, all))
}

/// `updateTiming(false)` after an annotation or a parasitic change: the invalid vertices, then in
/// level order whatever they reach — a driver re-times its net and queues each load whose slew
/// moved by more than the fuzzy tolerance (`loadSlewChanged`: `delayEqual` over every slew); a
/// load queues its fanout. A stage whose input slew did not move is NOT re-timed and keeps the
/// values the previous update left.
fn update_incremental(g: &mut Graph, paras: &HashMap<String, NetParasitics>, order: &[usize], index: &HashMap<String, usize>, dirty0: &[usize]) -> Res<()> {
    let mut dirty = vec![false; g.vertices.len()];
    for &v in dirty0 {
        dirty[v] = true;
    }
    for &v in order {
        if !dirty[v] {
            continue;
        }
        if g.vertices[v].is_driver {
            let loads: Vec<usize> = g.out_edges[v].iter().filter(|&&e| matches!(g.edges[e].kind, vyges_sta::graph::EdgeKind::Wire)).map(|&e| g.edges[e].to).collect();
            let prev: Vec<[[f32; 2]; 2]> = loads.iter().map(|&l| g.slew[l]).collect();
            g.find_vertex_delays(v, paras, index, None).map_err(crate::log::Stop::Refused)?;
            for (k, &l) in loads.iter().enumerate() {
                let changed = (0..2).any(|rf| (0..2).any(|mm| !vyges_sta::fuzzy::equal(g.slew[l][rf][mm], prev[k][rf][mm])));
                if changed {
                    dirty[l] = true;
                }
            }
        } else {
            g.find_vertex_delays(v, paras, index, None).map_err(crate::log::Stop::Refused)?;
            for &e in &g.out_edges[v] {
                if matches!(g.edges[e].kind, vyges_sta::graph::EdgeKind::Gate { .. }) {
                    dirty[g.edges[e].to] = true;
                }
            }
        }
    }
    Ok(())
}

/// A result normalized for the table (`characterizationPostProcess`'s converted data).
#[derive(Debug, Clone)]
struct Converted {
    pin_slew: u32,
    in_slew: u32,
    totalcap: u32,
    load: u32,
    wirelength: u32,
    pin_arrival: f32,
    total_power: f32,
    pure_wire: bool,
    topology: Vec<String>,
}

/// `normalizeCharResults`: `ceil(value / step)`, at least 1, widening the range.
fn normalize(value: f32, step: f32, min: &mut u32, max: &mut u32) -> u32 {
    let mut v = (value / step).ceil() as u32;
    if v == 0 {
        v = 1;
    }
    *min = (*min).min(v);
    *max = (*max).max(v);
    v
}

/// The bounds `characterizationPostProcess` leaves.
struct Bounds {
    min_wl: u32,
    max_wl: u32,
    min_cap: u32,
    max_cap: u32,
    min_slew: u32,
    max_slew: u32,
}

/// `characterizationPostProcess`: in key order, every result under the slew limit normalized.
fn post_process(s: &Setup, map: &BTreeMap<CharKey, Vec<ResultData>>) -> PostProcessed {
    let mut b = Bounds { min_wl: u32::MAX, max_wl: 0, min_cap: u32::MAX, max_cap: 0, min_slew: u32::MAX, max_slew: 0 };
    let mut out = Vec::new();
    for r in map.values().flatten() {
        if f64::from(r.pin_slew) > s.max_char_slew {
            continue;
        }
        let pin_slew = normalize(r.pin_slew, s.slew_step, &mut b.min_slew, &mut b.max_slew);
        let in_slew = normalize(r.in_slew, s.slew_step, &mut b.min_slew, &mut b.max_slew);
        let totalcap = normalize(r.totalcap, s.cap_step, &mut b.min_cap, &mut b.max_cap);
        let load = normalize(r.load, s.cap_step, &mut b.min_cap, &mut b.max_cap);
        let wirelength = normalize(r.wirelength as f32, s.unit as f32, &mut b.min_wl, &mut b.max_wl);
        let pin_arrival = (r.pin_arrival / (s.slew_step / 5.0)).ceil();
        let topology = r
            .topology
            .iter()
            .map(|x| if s.masters.contains(x) { x.clone() } else { format!("{:.6}", x.parse::<f64>().unwrap_or(0.0) / f64::from(r.wirelength as f32)) })
            .collect();
        out.push(Converted { pin_slew, in_slew, totalcap, load, wirelength, pin_arrival, total_power: r.total_power, pure_wire: r.pure_wire, topology });
    }
    PostProcessed { rows: out, bounds: b }
}

/// The post-processed rows and their bounds.
struct PostProcessed {
    rows: Vec<Converted>,
    bounds: Bounds,
}

/// `compileLut`.
fn compile_lut(log: &mut Log, s: &Setup, pp: &PostProcessed) -> Res<Lut> {
    let ratio = s.unit / s.length_unit;
    let b = &pp.bounds;
    let mut lut = Lut {
        length_unit: s.length_unit,
        length_unit_ratio: ratio,
        min_segment_length: b.min_wl * ratio,
        max_segment_length: b.max_wl * ratio,
        min_capacitance: b.min_cap,
        max_capacitance: b.max_cap,
        min_slew: b.min_slew,
        max_slew: b.max_slew,
        actual_min_input_cap: u32::MAX,
        ..Lut::default()
    };
    if [lut.min_segment_length, lut.max_segment_length, lut.min_capacitance, lut.max_capacitance, lut.min_slew, lut.max_slew].iter().any(|&v| v > MAX_NORMALIZED_VAL) {
        return log.error("CTS", 65, format!("Normalized values in the LUT should be in the range [1, {MAX_NORMALIZED_VAL}\n    Check the table above to see the normalization ranges and your     characterization configuration."));
    }
    for row in &pp.rows {
        lut.actual_min_input_cap = lut.actual_min_input_cap.min(row.totalcap);
        let mut pin_slew = row.pin_slew;
        if row.pure_wire && pin_slew <= row.in_slew {
            pin_slew += 1;
        }
        let length = row.wirelength * ratio;
        let mut seg = WireSegment {
            length: length as u8,
            load: row.load as u8,
            output_slew: pin_slew as u8,
            power: f64::from(row.total_power),
            delay: row.pin_arrival as u32,
            input_cap: row.totalcap as u8,
            input_slew: row.in_slew as u8,
            buffer_locations: vec![],
            buffer_masters: vec![],
            wl2_first_buffer: 0,
            last_wl: 0,
        };
        let len = f64::from(length);
        if !row.pure_wire {
            let wl2_first = (row.topology[0].parse::<f64>().unwrap_or(0.0) * len).round() as i32;
            let mut last_wl = 0;
            let max_index = if row.topology.len() % 2 == 0 {
                row.topology.len()
            } else {
                last_wl = (row.topology[row.topology.len() - 1].parse::<f64>().unwrap_or(0.0) * len).round() as i32;
                row.topology.len() - 1
            };
            for x in &row.topology[..max_index] {
                if s.masters.contains(x) {
                    seg.buffer_masters.push(x.clone());
                } else {
                    seg.buffer_locations.push(x.parse::<f64>().unwrap_or(0.0));
                }
            }
            seg.last_wl = last_wl;
            seg.wl2_first_buffer = wl2_first;
        } else {
            let wl = (row.topology[0].parse::<f64>().unwrap_or(0.0) * len).round() as i32;
            seg.last_wl = wl;
            seg.wl2_first_buffer = wl;
        }
        let key = Lut::key(seg.length, seg.load, seg.output_slew);
        lut.key_to_segments.entry(key).or_default().push(lut.segments.len());
        lut.segments.push(seg);
    }
    Ok(lut)
}

/// The table's probe lines (`VYGTCS`), as the instrumented reference prints them.
pub fn probe_segments(log: &mut Log, lut: &Lut) {
    for (i, g) in lut.segments.iter().enumerate() {
        let locs: Vec<String> = g.buffer_locations.iter().map(|x| format!("{:016x}", x.to_bits())).collect();
        log.trace(format!(
            "VYGTCS|{i}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}",
            g.length,
            g.load,
            g.output_slew,
            g.delay,
            g.input_cap,
            g.input_slew,
            g.wl2_first_buffer,
            g.last_wl,
            locs.join(","),
            g.buffer_masters.join(",")
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`getBufferingCombo`): only configurations whose drive never decreases along the chain.
    #[test]
    fn monotonic_configurations() {
        assert_eq!(buffering_combo(4, 4), 35);
        assert_eq!(buffering_combo(4, 2), 10);
        assert_eq!(buffering_combo(1, 3), 1);
        assert_eq!(buffering_combo(3, 0), 1);
    }

    /// Rules (`reduceOrExpand`): drop the later of the closest pair; insert the midpoint after the
    /// earlier of the widest.
    #[test]
    fn axes_reduce_and_expand() {
        let mut v = vec![1.0, 2.0, 2.1, 5.0];
        reduce_or_expand(&mut v, 3);
        assert_eq!(v, [1.0, 2.0, 5.0]);
        reduce_or_expand(&mut v, 4);
        assert_eq!(v, [1.0, 2.0, 3.5, 5.0]);
    }
}
