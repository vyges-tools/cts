// SPDX-License-Identifier: Apache-2.0
//! Clock nets and their sinks: `populateTritonCTS`.
//!
//! One function per reference function, in call order:
//! - [`find_clk_nets`] — `dbSta::findClkNets` (the clock network: every pin a clock reaches
//!   through wires and combinational arcs, short of another clock's source), as nets;
//! - [`find_clock_roots`] — `findClockRoots(clk, nets)`: the nets of a clock's source pins;
//! - [`populate_triton_cts`] — per clock, per root net in id order: `cloneClockGaters`, CTS-0007,
//!   then [`init_one_clock_tree`];
//! - [`init_one_clock_tree`] — `initOneClockTree`: [`init_clock`] for the net, then recursion
//!   through each non-sink load's single clock output;
//! - [`init_clock`] — `initClock` → `initClockTreeForMacrosAndRegs` →
//!   [`separate_macro_reg_sinks`], then one tree (CTS-0010) or a macro tree and a register tree
//!   behind a new buffer (CTS-0011 twice).

use crate::log::{refuse, Log, Res, Stop};
use std::collections::{BTreeSet, HashMap, HashSet, VecDeque};
use vyges_sta::liberty::{Cell, Role};

/// An instance terminal: instance and master terminal names.
pub type ITerm = (String, String);

/// A pin of the timing graph: a block port, or an instance terminal.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Pin {
    Port(String),
    Term(ITerm),
}

/// One SDC clock: its name and its source (leaf) pins.
#[derive(Debug, Clone)]
pub struct SdcClock {
    pub name: String,
    pub leaf_pins: Vec<Pin>,
}

/// What discovery reads about (and writes to) the design.
pub trait ClockDesign {
    /// `dbNet` id: `PtrSet<dbNet>` iterates by it.
    fn net_id(&self, net: &str) -> u32;
    /// The net's instance terminals, in its `getITerms()` order.
    fn net_iterms(&self, net: &str) -> Vec<ITerm>;
    /// The net's block ports.
    fn net_bterms(&self, net: &str) -> Vec<String>;
    fn net_is_special(&self, net: &str) -> bool;
    /// `dbNet::getFirstOutput`.
    fn net_first_output(&self, net: &str) -> Option<ITerm>;
    /// `dbNet::get1stBTerm`.
    fn net_first_bterm(&self, net: &str) -> Option<String>;
    /// `dbITerm::isInputSignal(io)` / `isOutputSignal(io)`.
    fn iterm_is_input_signal(&self, t: &ITerm, io: bool) -> bool;
    fn iterm_is_output_signal(&self, t: &ITerm, io: bool) -> bool;
    fn iterm_net(&self, t: &ITerm) -> Option<String>;
    /// A port's net, and whether the port drives into the block (an input port).
    fn bterm_net(&self, port: &str) -> Option<String>;
    fn bterm_is_input(&self, port: &str) -> bool;
    fn bterm_is_inout(&self, port: &str) -> bool;
    /// The instance's terminals, in `getITerms()` order.
    fn inst_iterms(&self, inst: &str) -> Vec<ITerm>;
    fn inst_master(&self, inst: &str) -> String;
    fn inst_is_block(&self, inst: &str) -> bool;
    fn inst_is_placed(&self, inst: &str) -> bool;
    fn inst_is_fixed(&self, inst: &str) -> bool;
    fn inst_is_pad(&self, inst: &str) -> bool;
    /// `getSourceType() == TIMING`.
    fn inst_is_timing_source(&self, inst: &str) -> bool;
    /// The liberty cell a master links to.
    fn liberty_cell(&self, master: &str) -> Option<&Cell>;
    /// `dbITerm::getAvgXY`: the mean of the terminal's pin box centres, `None` without shapes.
    fn iterm_avg_xy(&self, t: &ITerm) -> Option<(i32, i32)>;
    /// Every instance's `getLocation()`.
    fn inst_locations(&self) -> Vec<(i32, i32)>;
    /// `dbNetwork::hasHierarchy`: the design was linked hierarchically (`link_design -hier`).
    fn has_hierarchy(&self) -> bool;
    /// `stt::makeSteinerTree(net, xs, ys, drvr)` with the net's routing alpha.
    fn steiner_tree(&self, net: &str, xs: &[i32], ys: &[i32], drvr: usize) -> Res<vyges_stt::Tree>;
    /// A CLOCK net named `name` (`makeNet(IF_NEEDED)`: uniquified only when taken); its name.
    fn make_clock_net(&mut self, name: &str) -> Res<String>;
    /// A `TIMING` instance `name` of `gater`'s master, each input on the gater's input nets in
    /// its terminal order, its output on `net`.
    fn make_clone(&mut self, gater: &str, name: &str, net: &str) -> Res<()>;
    /// A load moved to `net` (disconnect, then connect).
    fn reconnect(&mut self, t: &ITerm, net: &str) -> Res<()>;
    /// `setLocation` + `PLACED`.
    fn place(&mut self, inst: &str, at: (i32, i32)) -> Res<()>;
    /// `forkRegisterClockNetwork`'s edits: a CLOCK net `<net>_regs`, the register sinks moved to
    /// it, a `TIMING` buffer of the root master named `top_buffer` at the new net's terminal
    /// bounding-box centre, driven from `net`. Returns the new net's name.
    fn fork_register_net(&mut self, net: &str, sinks: &[ITerm], root_buffer: &str, top_buffer: &str) -> Res<String>;
}

/// What discovery reads from the options and the buffer selection.
pub struct Setup<'a> {
    /// `-clk_nets` (resolved names; empty: the SDC clocks).
    pub clock_nets: &'a [String],
    pub skip_nets: &'a [String],
    pub insertion_delay: bool,
    pub buffer_list: &'a [String],
    pub root_buffer: &'a str,
    /// `CtsOptions::getMaxWl` (DBU): the max wire length when the resizer found one, else 0 — or
    /// why it could not be computed (read only by cloning).
    pub max_wl: Res<i32>,
}

/// One clock tree to build (`TreeBuilder`): what slice 1 knows of it.
#[derive(Debug, Clone, PartialEq)]
pub struct Builder {
    /// The net the tree drives (`clockNet.setNetObj`).
    pub net: String,
    /// The root net it was found from (`clkInputNet`).
    pub input_net: String,
    pub sdc_clock: String,
    /// The sinks, as `inst/pin`, in the order added.
    pub sinks: Vec<String>,
    pub parent: Option<usize>,
    /// `TreeType::MacroTree` / `RegisterTree` when the net was split; `None` otherwise.
    pub tree_type: Option<&'static str>,
    pub top_buffer: Option<String>,
}

/// The state `populateTritonCTS` builds.
#[derive(Debug, Default)]
pub struct Found {
    pub builders: Vec<Builder>,
    /// `net2builder_`, by root net.
    pub net2builder: HashMap<String, usize>,
    pub visited: HashSet<String>,
    /// `staClockNets_` (empty when the user named the clock nets).
    pub sta_clock_nets: HashSet<String>,
    /// `regTreeRootBufIndex_`.
    pub reg_tree_root_buf_index: usize,
}

fn port_of<'a>(cell: &'a Cell, name: &str) -> Option<&'a vyges_sta::liberty::Port> {
    cell.port(name)
}

/// `LibertyPort::isRegClk`: the port is the `from` of a reg-clk-to-Q or latch-enable-to-Q arc.
fn is_reg_clk(cell: &Cell, port: &str) -> bool {
    cell.arc_sets.iter().any(|a| a.from == port && matches!(a.role, Role::RegClkToQ | Role::LatchEnToQ))
}

/// `isSink`: no liberty cell, a block, or a register clock port.
pub fn is_sink(d: &dyn ClockDesign, t: &ITerm) -> bool {
    let Some(cell) = d.liberty_cell(&d.inst_master(&t.0)) else { return true };
    if d.inst_is_block(&t.0) {
        return true;
    }
    port_of(cell, &t.1).is_some() && is_reg_clk(cell, &t.1)
}

/// `hasInsertionDelay`: the port's max clock-tree-path delay is non-zero. The delay's table is
/// not read here: a port with such an arc is refused.
fn has_insertion_delay(d: &dyn ClockDesign, s: &Setup, t: &ITerm) -> Res<bool> {
    if !s.insertion_delay {
        return Ok(false);
    }
    let Some(cell) = d.liberty_cell(&d.inst_master(&t.0)) else { return Ok(false) };
    if cell.arc_sets.iter().any(|a| a.to == t.1 && (a.timing_type == "max_clock_tree_path" || a.timing_type == "min_clock_tree_path")) {
        return refuse(format!("{}/{}: clock tree path delays (insertion delay) are not modelled", t.0, t.1));
    }
    Ok(false)
}

/// `Sdc::isLeafPinClock`: the pin is some clock's source.
fn is_leaf_pin_clock(clocks: &[SdcClock], p: &Pin) -> bool {
    clocks.iter().any(|c| c.leaf_pins.contains(p))
}

/// A timing-graph vertex: a pin's load side, or its driver side (a bidirect pin has both).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct Vertex {
    pin: Pin,
    driver: bool,
}

/// `graph_->pinVertices(pin)`: a port drives into the block when it is an input or inout, and is a
/// load when it is an output or inout; a terminal is a load when it is an input or inout and drives
/// when it is an output or inout.
fn pin_vertices(d: &dyn ClockDesign, p: &Pin) -> Vec<Vertex> {
    let (load, driver) = match p {
        Pin::Port(b) => (!d.bterm_is_input(b) || d.bterm_is_inout(b), d.bterm_is_input(b) || d.bterm_is_inout(b)),
        Pin::Term(t) => (d.iterm_is_input_signal(t, true), d.iterm_is_output_signal(t, true)),
    };
    let mut v = Vec::new();
    if load {
        v.push(Vertex { pin: p.clone(), driver: false });
    }
    if driver {
        v.push(Vertex { pin: p.clone(), driver: true });
    }
    v
}

/// `dbSta::findClkNets` (through `ClkNetwork::findClkPins`): from each clock's source pins' vertices,
/// every vertex reached through wire edges (a driver to its net's loads) and combinational arcs (a
/// load to its cell's outputs, never a bidirect pin to itself), never into another clock's source;
/// the nets of their pins.
pub fn find_clk_nets(d: &dyn ClockDesign, clocks: &[SdcClock]) -> HashSet<String> {
    let mut nets = HashSet::new();
    for clk in clocks {
        let mut visited: HashSet<Vertex> = HashSet::new();
        let mut queue: VecDeque<Vertex> = VecDeque::new();
        for p in &clk.leaf_pins {
            for v in pin_vertices(d, p) {
                if visited.insert(v.clone()) {
                    queue.push_back(v);
                }
            }
        }
        while let Some(v) = queue.pop_front() {
            let net = match &v.pin {
                Pin::Port(b) => d.bterm_net(b),
                Pin::Term(t) => d.iterm_net(t),
            };
            if let Some(n) = &net {
                nets.insert(n.clone());
            }
            let mut fanouts: Vec<Vertex> = Vec::new();
            if v.driver {
                if let Some(n) = &net {
                    for t in d.net_iterms(n) {
                        let p = Pin::Term(t.clone());
                        if p != v.pin && d.iterm_is_input_signal(&t, true) {
                            fanouts.push(Vertex { pin: p, driver: false });
                        }
                    }
                    for b in d.net_bterms(n) {
                        let p = Pin::Port(b.clone());
                        if p != v.pin && (!d.bterm_is_input(&b) || d.bterm_is_inout(&b)) {
                            fanouts.push(Vertex { pin: p, driver: false });
                        }
                    }
                }
            } else if let Pin::Term(t) = &v.pin {
                if let Some(cell) = d.liberty_cell(&d.inst_master(&t.0)) {
                    for a in cell.arc_sets.iter().filter(|a| a.from == t.1 && a.to != t.1 && a.role == Role::Combinational) {
                        fanouts.push(Vertex { pin: Pin::Term((t.0.clone(), a.to.clone())), driver: true });
                    }
                }
            }
            for f in fanouts {
                if !is_leaf_pin_clock(clocks, &f.pin) && visited.insert(f.clone()) {
                    queue.push_back(f);
                }
            }
        }
    }
    nets
}

/// `findClockRoots(clk, nets)`: each source pin's net (skip nets warned, CTS-0042), by net id.
/// A source without a net is kept as `None` (CTS-0040 later).
pub fn find_clock_roots(d: &dyn ClockDesign, log: &mut Log, s: &Setup, clk: &SdcClock) -> Vec<Option<String>> {
    let mut nets: BTreeSet<(u32, Option<String>)> = BTreeSet::new();
    for p in &clk.leaf_pins {
        let net = match p {
            Pin::Port(b) => d.bterm_net(b),
            Pin::Term(t) => d.iterm_net(t),
        };
        if let Some(n) = &net {
            if s.skip_nets.contains(n) {
                log.warn("CTS", 42, format!("Skipping root net {n}, specified by the user..."));
                continue;
            }
        }
        // A null net sorts first (no id).
        let id = net.as_deref().map_or(0, |n| d.net_id(n) + 1);
        nets.insert((id, net));
    }
    nets.into_iter().map(|(_, n)| n).collect()
}

/// `getSingleOutput(inst, input)`: the instance's first output terminal (other than the input)
/// whose net is a clock net.
fn get_single_output(d: &dyn ClockDesign, f: &Found, t: &ITerm) -> Option<ITerm> {
    d.inst_iterms(&t.0).into_iter().find(|o| o != t && d.iterm_is_output_signal(o, true) && d.iterm_net(o).is_some_and(|n| f.sta_clock_nets.contains(&n)))
}

/// `populateTritonCTS`. `clocks` are the SDC's, in its order.
pub fn populate_triton_cts(d: &mut dyn ClockDesign, log: &mut Log, s: &Setup, clocks: &[SdcClock]) -> Res<Found> {
    let mut f = Found::default();
    let mut info: Vec<(Vec<Option<String>>, String)> = Vec::new();
    if !s.clock_nets.is_empty() {
        let mut nets: Vec<String> = s.clock_nets.to_vec();
        nets.sort_by_key(|n| d.net_id(n));
        nets.dedup();
        info.push((nets.into_iter().map(Some).collect(), String::new()));
    } else {
        f.sta_clock_nets = find_clk_nets(d, clocks);
        let mut all: HashSet<String> = HashSet::new();
        for clk in clocks {
            let nets = find_clock_roots(d, log, s, clk);
            for n in nets.iter().flatten() {
                if all.contains(n) {
                    return log.error("CTS", 114, format!("Clock {} overlaps a previous clock.", clk.name));
                }
            }
            all.extend(nets.iter().flatten().cloned());
            info.push((nets, clk.name.clone()));
        }
    }
    // Every instance's location, so a clone never lands on one (mapLocationToSink_ keys).
    let mut occupied: BTreeSet<(i32, i32)> = d.inst_locations().into_iter().collect();
    let mut clone_visited: HashSet<String> = HashSet::new();
    for (nets, clk_name) in &info {
        for net in nets {
            let Some(net) = net else {
                log.warn("CTS", 40, format!("Net was not found in the design for {clk_name}, please check. Skipping..."));
                continue;
            };
            clone_clock_gaters(d, log, s, &mut f, net, &mut occupied, &mut clone_visited)?;
            if clk_name.is_empty() {
                log.info("CTS", 95, format!("Net \"{net}\" found."));
            } else {
                log.info("CTS", 7, format!("Net \"{net}\" found for clock \"{clk_name}\"."));
            }
            if !f.visited.contains(net) {
                init_one_clock_tree(d, log, s, clocks, &mut f, net, net, clk_name, None)?;
            }
        }
    }
    if f.builders.is_empty() {
        log.warn("CTS", 83, "No clock nets have been found.");
    }
    log.info("CTS", 8, format!("TritonCTS found {} clock nets.", f.builders.len()));
    Ok(f)
}

/// `cloneClockGaters(clkNet, occupiedPositions, visitedNets)`: depth first through each non-sink,
/// non-buffer, non-inverter load's clock output; then, for a net driven by a movable non-sink
/// cell, the Steiner tree over its loads (`findLongEdges`).
fn clone_clock_gaters(d: &mut dyn ClockDesign, log: &mut Log, s: &Setup, f: &mut Found, net: &str, occupied: &mut BTreeSet<(i32, i32)>, visited: &mut HashSet<String>) -> Res<()> {
    if !visited.insert(net.to_string()) {
        return Ok(()); // a cycle
    }
    let driver = d.net_first_output(net);
    let (mut xs, mut ys) = (Vec::new(), Vec::new());
    let mut point2pin: HashMap<(i32, i32), Vec<ITerm>> = HashMap::new();
    for t in d.net_iterms(net) {
        if Some(&t) == driver.as_ref() || !d.iterm_is_input_signal(&t, true) {
            continue;
        }
        let Some(p) = d.iterm_avg_xy(&t) else { return refuse(format!("{}/{}: no pin shapes (getAvgXY) is not modelled", t.0, t.1)) };
        xs.push(p.0);
        ys.push(p.1);
        point2pin.entry(p).or_default().push(t.clone());
        if is_sink(d, &t) {
            continue;
        }
        let Some(out) = get_single_output(d, f, &t) else { continue };
        let Some(out_net) = d.iterm_net(&out) else { continue };
        let Some(cell) = d.liberty_cell(&d.inst_master(&t.0)) else { continue };
        // Clock tree buffers and inverters are not gaters.
        if cell.is_inverter() || cell.is_buffer() {
            continue;
        }
        clone_clock_gaters(d, log, s, f, &out_net, occupied, visited)?;
    }
    let Some(drv) = driver else { return Ok(()) };
    // xs is empty when the only load is a port.
    if is_sink(d, &drv) || d.inst_is_fixed(&drv.0) || d.inst_is_pad(&drv.0) || xs.is_empty() {
        return Ok(());
    }
    let Some(dp) = d.iterm_avg_xy(&drv) else { return refuse(format!("{}/{}: no pin shapes (getAvgXY) is not modelled", drv.0, drv.1)) };
    point2pin.entry(dp).or_default().push(drv);
    // ⚠️ The driver is not among the points: index 0 is the FIRST LOAD.
    let tree = d.steiner_tree(net, &xs, &ys, 0)?;
    find_long_edges(d, log, s, f, &tree, dp, &mut point2pin, occupied)
}

/// `odb::Point::manhattanDistance`.
fn manhattan(a: (i32, i32), b: (i32, i32)) -> i64 {
    (i64::from(a.0) - i64::from(b.0)).abs() + (i64::from(a.1) - i64::from(b.1)).abs()
}

/// A bounding box grown point by point (`Rect::mergeInit` / `merge`); its centre as odb's
/// `xCenter`/`yCenter` (an empty one is `(0, 0)`: `INT_MAX + INT_MIN` halved).
struct BBox(i32, i32, i32, i32);

impl BBox {
    fn new() -> BBox {
        BBox(i32::MAX, i32::MAX, i32::MIN, i32::MIN)
    }
    fn merge(&mut self, p: (i32, i32)) {
        self.0 = self.0.min(p.0);
        self.1 = self.1.min(p.1);
        self.2 = self.2.max(p.0);
        self.3 = self.3.max(p.1);
    }
    fn center(&self) -> (i32, i32) {
        (((i64::from(self.0) + i64::from(self.2)) / 2) as i32, ((i64::from(self.1) + i64::from(self.3)) / 2) as i32)
    }
}

/// `findLongEdges`: the tree's branches grouped into clusters joined by edges shorter than the
/// max wire length; the cluster nearest the driver keeps the gater, each other valid cluster gets
/// a clone of it (new net `clonenet_<k>_<net>`, instance `clone_<k>_<gater>`), placed at its
/// loads' centre.
#[allow(clippy::too_many_arguments)]
fn find_long_edges(d: &mut dyn ClockDesign, log: &mut Log, s: &Setup, f: &mut Found, tree: &vyges_stt::Tree, driver_pt: (i32, i32), point2pin: &mut HashMap<(i32, i32), Vec<ITerm>>, occupied: &mut BTreeSet<(i32, i32)>) -> Res<()> {
    const G: &str = "clock gate cloning";
    let threshold = i64::from(s.max_wl.clone()?);
    log.debug("CTS", G, 1, || format!("Threshold = {threshold}"));
    let mut iterm2cluster: HashMap<usize, usize> = HashMap::new();
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    let first = point2pin[&driver_pt][0].clone();
    let icg_net = d.iterm_net(&first).ok_or_else(|| Stop::Refused("the gater's first pin has no net".into()))?;
    let icg_term = d.net_first_output(&icg_net).ok_or_else(|| Stop::Refused(format!("net {icg_net}: no driver")))?;
    let icg_name = icg_term.0.clone();
    for b in 0..tree.branch_count() {
        let br = tree.branch[b];
        let nb = tree.branch[br.n];
        let dist = manhattan((br.x, br.y), (nb.x, nb.y));
        let from = iterm2cluster.get(&b).copied();
        let to = iterm2cluster.get(&br.n).copied();
        if b == br.n {
            continue;
        }
        if dist >= threshold {
            if from.is_none() {
                iterm2cluster.insert(b, clusters.len());
                clusters.push(vec![b]);
            }
            if to.is_none() {
                iterm2cluster.insert(br.n, clusters.len());
                clusters.push(vec![br.n]);
            }
            continue;
        }
        match (from, to) {
            (Some(cf), Some(ct)) => {
                let kept = if clusters[cf].len() >= clusters[ct].len() { cf } else { ct };
                let removed = if clusters[cf].len() < clusters[ct].len() { cf } else { ct };
                // The removed cluster's points appended to the kept one, then it is emptied —
                // also when both are the same cluster (it ends empty).
                let moved = clusters[removed].clone();
                clusters[kept].extend(moved.iter().copied());
                let all = clusters[removed].clone();
                for p in all {
                    iterm2cluster.insert(p, kept);
                }
                clusters[removed].clear();
            }
            (Some(cf), None) => {
                iterm2cluster.insert(br.n, cf);
                clusters[cf].push(br.n);
            }
            (None, Some(ct)) => {
                iterm2cluster.insert(b, ct);
                clusters[ct].push(b);
            }
            (None, None) => {
                iterm2cluster.insert(b, clusters.len());
                iterm2cluster.insert(br.n, clusters.len());
                clusters.push(vec![b, br.n]);
            }
        }
    }
    let pins_at = |point2pin: &mut HashMap<(i32, i32), Vec<ITerm>>, b: usize| point2pin.entry((tree.branch[b].x, tree.branch[b].y)).or_default().clone();
    // The valid cluster (one with a load) nearest the driver.
    let (mut driver_cluster, mut min_dist, mut valid) = (None, i64::MAX, 0);
    for (n, cluster) in clusters.iter().enumerate() {
        if cluster.is_empty() {
            continue;
        }
        let (mut ok, mut bb) = (false, BBox::new());
        for &b in cluster {
            for t in pins_at(point2pin, b) {
                if !d.iterm_is_input_signal(&t, true) {
                    continue;
                }
                ok = true;
                bb.merge(d.iterm_avg_xy(&t).expect("a load with shapes"));
            }
        }
        if ok {
            valid += 1;
            let dist = manhattan(bb.center(), driver_pt);
            if dist < min_dist {
                driver_cluster = Some(n);
                min_dist = dist;
            }
        }
    }
    log.debug("CTS", G, 1, || format!("Found {valid} clusters"));
    // The reference finds the gater net's module (the clone's scope) here, but nothing is written
    // until a clone is made: a hierarchical design whose gaters keep one cluster each is cloned
    // exactly as a flat one (gated_clock4).
    let mut n_clones = 0;
    for (n, cluster) in clusters.iter().enumerate() {
        if cluster.is_empty() {
            continue;
        }
        let (clone, clone_net, disconnect) = if driver_cluster == Some(n) {
            log.debug("CTS", G, 1, || format!("Original cell {}", icg_term.0));
            log.debug("CTS", G, 2, || format!(" Original net {icg_net}"));
            (icg_term.0.clone(), icg_net.clone(), false)
        } else {
            if d.has_hierarchy() {
                return refuse("clock gate cloning in a hierarchical design (the clone's module) is not modelled");
            }
            n_clones += 1;
            let net_name = d.make_clock_net(&format!("clonenet_{n_clones}_{icg_net}"))?;
            f.sta_clock_nets.insert(net_name.clone());
            let buf_name = format!("clone_{n_clones}_{icg_name}");
            d.make_clone(&icg_term.0, &buf_name, &net_name)?;
            log.debug("CTS", G, 1, || format!("Creating clone {buf_name} from {icg_name}"));
            log.debug("CTS", G, 2, || format!(" New clone net {net_name}"));
            (buf_name, net_name, true)
        };
        let mut bb = BBox::new();
        for &b in cluster {
            for t in pins_at(point2pin, b) {
                if !d.iterm_is_input_signal(&t, true) {
                    continue;
                }
                log.debug("CTS", G, 2, || format!("  Connects sink {}/{}", t.0, t.1));
                bb.merge(d.iterm_avg_xy(&t).expect("a load with shapes"));
                if disconnect {
                    d.reconnect(&t, &clone_net)?;
                }
            }
        }
        // resolveLocationCollision: one DBU right until the spot is free.
        let mut loc = bb.center();
        while occupied.contains(&loc) {
            loc.0 += 1;
        }
        occupied.insert(loc);
        d.place(&clone, loc)?;
    }
    log.debug("CTS", G, 1, || format!("Created {n_clones} clones"));
    Ok(())
}

/// `initOneClockTree(driverNet, clkInputNet, sdcClockName, parent)`.
#[allow(clippy::too_many_arguments)]
pub fn init_one_clock_tree(d: &mut dyn ClockDesign, log: &mut Log, s: &Setup, clocks: &[SdcClock], f: &mut Found, net: &str, input_net: &str, clk: &str, parent: Option<usize>) -> Res<()> {
    let mut builder = None;
    if d.net_is_special(net) {
        log.info("CTS", 116, format!("Special net \"{net}\" skipped."));
    } else if s.skip_nets.iter().any(|n| n == net) {
        log.warn("CTS", 44, format!("Skipping net {net}, specified by the user..."));
    } else {
        builder = init_clock(d, log, s, f, net, input_net, clk, parent)?;
    }
    if let Some(b) = builder {
        f.net2builder.entry(input_net.to_string()).or_insert(b);
    }
    // Gated clocks are separate trees.
    f.visited.insert(net.to_string());
    let driver = d.net_first_output(net);
    for t in d.net_iterms(net) {
        if Some(&t) == driver.as_ref() || !d.iterm_is_input_signal(&t, true) || is_sink(d, &t) {
            continue;
        }
        let Some(out) = get_single_output(d, f, &t) else { continue };
        let Some(out_net) = d.iterm_net(&out) else { continue };
        if f.visited.contains(&out_net) || is_leaf_pin_clock(clocks, &Pin::Term(out)) {
            continue;
        }
        let up = match (builder, f.net2builder.get(input_net)) {
            (None, Some(&b)) => Some(b),
            _ => builder,
        };
        init_one_clock_tree(d, log, s, clocks, f, &out_net, input_net, clk, up)?;
    }
    Ok(())
}

/// `separateMacroRegSinks`: the placed input sinks split into those with insertion delay (macros,
/// non-sinks, inverters that drive a macro) and registers; `None` when the net already has a
/// clock buffer (CTS-0105 / CTS-0110).
fn separate_macro_reg_sinks(d: &dyn ClockDesign, log: &mut Log, s: &Setup, net: &str) -> Res<Option<(Vec<ITerm>, Vec<ITerm>)>> {
    let mut masters: HashSet<&str> = s.buffer_list.iter().map(String::as_str).collect();
    masters.insert(s.root_buffer);
    let (mut regs, mut macros) = (Vec::new(), Vec::new());
    let mut skipped_timing_buf: Option<String> = None;
    for t in d.net_iterms(net) {
        let inst = &t.0;
        let timing_buffer = masters.contains(d.inst_master(inst).as_str()) && d.inst_is_timing_source(inst);
        if d.iterm_is_output_signal(&t, true) && timing_buffer {
            log.warn("CTS", 105, format!("Net \"{net}\" already has clock buffer {inst}. Skipping..."));
            return Ok(None);
        }
        if d.iterm_is_input_signal(&t, true) && timing_buffer {
            skipped_timing_buf = Some(inst.clone());
            continue;
        }
        if d.iterm_is_input_signal(&t, true) && d.inst_is_placed(inst) {
            let mut non_sink_macro = !is_sink(d, &t);
            if let Some(cell) = d.liberty_cell(&d.inst_master(inst)) {
                if cell.is_inverter() {
                    let out = d.inst_iterms(inst).into_iter().find(|o| d.iterm_is_output_signal(o, true));
                    let out_net = out.and_then(|o| d.iterm_net(&o));
                    let first_in = out_net.and_then(|n| d.net_iterms(&n).into_iter().find(|x| d.iterm_is_input_signal(x, false)));
                    match first_in {
                        Some(x) => non_sink_macro &= d.inst_is_block(&x.0),
                        None => log.warn("CTS", 136, format!("Inverter {inst} output net has no signal input; treating as a dangling clock inverter.")),
                    }
                }
            }
            if has_insertion_delay(d, s, &t)? || non_sink_macro || d.inst_is_block(inst) {
                macros.push(t);
            } else {
                regs.push(t);
            }
        }
    }
    if let Some(b) = skipped_timing_buf {
        if regs.len() + macros.len() < 2 {
            log.warn("CTS", 110, format!("Net \"{net}\" already has clock buffer {b}. Skipping..."));
            return Ok(None);
        }
    }
    Ok(Some((regs, macros)))
}

fn sink_name(t: &ITerm) -> String {
    format!("{}/{}", t.0, t.1)
}

/// `initClock` and `initClockTreeForMacrosAndRegs`: the driver (an output terminal, else a port,
/// else CTS-0122), then the sinks — one tree, or a macro tree and a register tree.
#[allow(clippy::too_many_arguments)]
fn init_clock(d: &mut dyn ClockDesign, log: &mut Log, s: &Setup, f: &mut Found, net: &str, input_net: &str, clk: &str, parent: Option<usize>) -> Res<Option<usize>> {
    if d.net_first_output(net).is_none() && d.net_first_bterm(net).is_none() {
        log.info("CTS", 122, format!("Clock net \"{net}\" is skipped for CTS because it is not connected to any output instance pin or input block terminal."));
        return Ok(None);
    }
    let Some((regs, macros)) = separate_macro_reg_sinks(d, log, s, net)? else { return Ok(None) };
    if !s.insertion_delay || macros.is_empty() || regs.is_empty() {
        let sinks: Vec<String> = d.net_iterms(net).iter().filter(|t| d.iterm_is_input_signal(t, true) && d.inst_is_placed(&t.0)).map(sink_name).collect();
        if sinks.len() < 2 {
            log.warn("CTS", 41, format!("Net \"{net}\" has {} sinks. Skipping...", sinks.len()));
            return Ok(None);
        }
        log.info("CTS", 10, format!(" Clock net \"{net}\" has {} sinks.", sinks.len()));
        f.builders.push(Builder { net: net.into(), input_net: input_net.into(), sdc_clock: clk.into(), sinks, parent, tree_type: None, top_buffer: None });
        return Ok(Some(f.builders.len() - 1));
    }
    let first = add_clock_sinks(log, f, net, input_net, clk, &macros, parent, "macros");
    f.builders[first].tree_type = Some("MacroTree");
    let top_buffer = format!("clkbuf_regs_{}_{clk}", f.reg_tree_root_buf_index);
    f.reg_tree_root_buf_index += 1;
    let second_net = d.fork_register_net(net, &regs, s.root_buffer, &top_buffer)?;
    let second = add_clock_sinks(log, f, &second_net, input_net, clk, &regs, Some(first), "registers");
    f.builders[second].tree_type = Some("RegisterTree");
    f.builders[second].top_buffer = Some(top_buffer);
    Ok(Some(first))
}

/// `addClockSinks`: the sinks, CTS-0011, a builder.
#[allow(clippy::too_many_arguments)]
fn add_clock_sinks(log: &mut Log, f: &mut Found, net: &str, input_net: &str, clk: &str, sinks: &[ITerm], parent: Option<usize>, kind: &str) -> usize {
    let sinks: Vec<String> = sinks.iter().map(sink_name).collect();
    log.info("CTS", 11, format!(" Clock net \"{net}\" for {kind} has {} sinks.", sinks.len()));
    f.builders.push(Builder { net: net.into(), input_net: input_net.into(), sdc_clock: clk.into(), sinks, parent, tree_type: None, top_buffer: None });
    f.builders.len() - 1
}
