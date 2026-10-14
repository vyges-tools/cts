// SPDX-License-Identifier: Apache-2.0
//! The H-tree (`HTreeBuilder`): the sink region, one topology level after another until a stop
//! criterion, the branching points legalized, then the clock buffers and subnets of the tree.
//!
//! Each stage is its own function, in the reference's order and named after it; [`run`] is only
//! the call sequence. Distances are in wire-segment units (DBU / the characterization unit).
//!
//! Modelled: a tree without sink clustering (which only engages above 200 sinks with
//! `-sink_clustering_enable`), plain or simple segments (`-distance_between_buffers`, with or
//! without vertex buffers), the obstruction-aware legalizer with the blockages it is given.

use crate::clustering::Clustering;
use crate::log::{refuse, Log, Res};
use crate::techchar::{Lut, WireSegment};
use std::collections::{HashMap, HashSet};

/// `Point<double>`: `==` is fuzzy (1e-6 on each coordinate); ordering and map keys are exact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct P {
    pub x: f64,
    pub y: f64,
}

impl P {
    pub fn new(x: f64, y: f64) -> P {
        P { x, y }
    }
    /// `Point::computeDist`: Manhattan.
    pub fn dist(self, o: P) -> f64 {
        let dx = if self.x > o.x { self.x - o.x } else { o.x - self.x };
        let dy = if self.y > o.y { self.y - o.y } else { o.y - self.y };
        dx + dy
    }
    /// `Point::operator==` (`equal`, epsilon 1e-6).
    pub fn same(self, o: P) -> bool {
        (self.x - o.x).abs() < 1e-6 && (self.y - o.y).abs() < 1e-6
    }
    /// The key an ordered map compares by (`operator<` is exact; `-0.0 == 0.0`).
    pub fn key(self) -> (u64, u64) {
        ((self.x + 0.0).to_bits(), (self.y + 0.0).to_bits())
    }
}

/// `operator<<(Point)`: `[(x, y)]`, three fixed decimals.
pub fn fmt_p(p: P) -> String {
    format!("[({:.3}, {:.3})]", p.x, p.y)
}

/// An `std::ostream` default float (`%g`, six significant digits).
pub fn fmt_ostream(v: f64) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    let s = format!("{v:.5e}");
    let (m, e) = s.split_once('e').expect("exponent form");
    let exp: i32 = e.parse().expect("an exponent");
    let strip = |t: String| if t.contains('.') { t.trim_end_matches('0').trim_end_matches('.').to_string() } else { t };
    if (-4..6).contains(&exp) {
        let decimals = (5 - exp) as usize;
        strip(format!("{v:.decimals$}"))
    } else {
        format!("{}e{}{:02}", strip(m.to_string()), if exp < 0 { '-' } else { '+' }, exp.abs())
    }
}

/// `cts::fuzzyEqual`.
fn fuzzy_equal(a: f64, b: f64, eps: f64) -> bool {
    (a - b).abs() < eps
}
fn fuzzy_equal_or_greater(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6 || a > b
}
fn fuzzy_equal_or_smaller(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6 || a < b
}

/// `Box<double>` with its cached centre (`getCenter` computes once, `setCenter` overrides).
#[derive(Debug, Clone, Copy, Default)]
pub struct Region {
    pub min_x: f64,
    pub min_y: f64,
    pub width: f64,
    pub height: f64,
    center: Option<P>,
}

impl Region {
    fn from_corners(x0: f64, y0: f64, x1: f64, y1: f64) -> Region {
        Region { min_x: x0, min_y: y0, width: x1 - x0, height: y1 - y0, center: None }
    }
    fn max_x(&self) -> f64 {
        self.min_x + self.width
    }
    fn max_y(&self) -> f64 {
        self.min_y + self.height
    }
    pub fn center(&mut self) -> P {
        if let Some(c) = self.center {
            return c;
        }
        let c = P::new(self.min_x + self.width / 2.0, self.min_y + self.height / 2.0);
        self.center = Some(c);
        c
    }
    fn set_center(&mut self, p: P) {
        self.center = Some(p);
    }
}

/// `TreeType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TreeType {
    Regular,
    Macro,
    Register,
}

/// A clock instance: a sink (an instance pin) or a clock buffer the tree adds.
#[derive(Debug, Clone, PartialEq)]
pub struct ClockInst {
    pub name: String,
    pub master: String,
    pub x: i32,
    pub y: i32,
    pub insertion_delay: f64,
    /// The sink pin's liberty capacitance (0 for a buffer the tree adds).
    pub input_cap: f32,
}

/// Which instance of the clock a subnet holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum InstRef {
    Sink(usize),
    Buffer(usize),
}

/// `ClockSubNet`: the driver first, then its sinks, in the order added.
#[derive(Debug, Clone, PartialEq)]
pub struct SubNet {
    pub name: String,
    pub insts: Vec<InstRef>,
    pub leaf: bool,
}

/// `Clock`: the net, its sinks, and what the tree adds (every name suffixed `_<net>`).
#[derive(Debug, Clone, PartialEq)]
pub struct Clock {
    /// The net name with `/` escaped (`sta::escapeChars`).
    pub name: String,
    pub sinks: Vec<ClockInst>,
    pub buffers: Vec<ClockInst>,
    pub subnets: Vec<SubNet>,
    pub max_level: u32,
}

impl Clock {
    pub fn new(net: &str) -> Clock {
        let mut name = String::new();
        for c in net.chars() {
            if c == '/' {
                name.push('\\');
            }
            name.push(c);
        }
        Clock { name, sinks: Vec::new(), buffers: Vec::new(), subnets: Vec::new(), max_level: 0 }
    }
    /// `addClockBuffer`.
    fn add_clock_buffer(&mut self, name: &str, master: &str, x: i32, y: i32) -> usize {
        self.buffers.push(ClockInst { name: format!("{name}_{}", self.name), master: master.into(), x, y, insertion_delay: 0.0, input_cap: 0.0 });
        self.buffers.len() - 1
    }
    /// `addSubNet`.
    fn add_sub_net(&mut self, name: &str) -> usize {
        self.subnets.push(SubNet { name: format!("{name}_{}", self.name), insts: Vec::new(), leaf: false });
        self.subnets.len() - 1
    }
    pub fn inst(&self, r: InstRef) -> &ClockInst {
        match r {
            InstRef::Sink(i) => &self.sinks[i],
            InstRef::Buffer(i) => &self.buffers[i],
        }
    }
    /// `computeSinkRegion`: the sinks' box without the 1 % outliers on each side.
    fn compute_sink_region(&self) -> (i32, i32, i32, i32) {
        let mut xs: Vec<i32> = self.sinks.iter().map(|s| s.x).collect();
        let mut ys: Vec<i32> = self.sinks.iter().map(|s| s.y).collect();
        xs.sort_unstable();
        ys.sort_unstable();
        let n = xs.len() as u32;
        let outliers = (0.01 * f64::from(n)) as u32 as usize;
        let n = n as usize;
        (xs[outliers], ys[outliers], xs[n - outliers - 1], ys[n - outliers - 1])
    }
}

/// What the tree reads of the options (`CtsOptions`), as the run left them.
#[derive(Debug, Clone)]
pub struct HOptions {
    pub root_buffer: String,
    pub sink_buffer: String,
    /// `-tree_buf`.
    pub tree_buffer: Option<String>,
    pub sink_clustering: bool,
    pub max_diameter: f64,
    pub max_diameter_set: bool,
    pub sink_clustering_size: u32,
    pub sink_clustering_size_set: bool,
    pub macro_max_diameter: f64,
    pub macro_max_diameter_set: bool,
    pub macro_sink_clustering_size: u32,
    pub macro_sink_clustering_size_set: bool,
    pub num_static_layers: u32,
    pub clustering_power: u32,
    pub clustering_capacity: f64,
    /// `maxFanout_` (0: none).
    pub max_fanout: u32,
    /// `maxWl_` (DBU; 0: none).
    pub max_wl: i32,
    /// `wireSegmentUnit_` as the characterization left it (DBU).
    pub wire_segment_unit: u32,
    pub db_units: i32,
    pub max_slew: u32,
    pub num_max_leaf_sinks: u32,
    pub clock_tree_max_depth: u32,
    pub fake_lut_entries: bool,
    pub obstruction_aware: bool,
    /// `getBufferDistance()` (DBU).
    pub buffer_distance: i32,
    /// `getSinkBufferInputCap()` and the characterization's capacitance per DBU (clustering).
    pub sink_buffer_input_cap: f64,
    pub cap_per_dbu: f64,
    /// `-sink_clustering_levels`.
    pub sink_clustering_levels: u32,
    /// `isSimpleSegmentEnabled` (`-distance_between_buffers` given).
    pub simple_segments: bool,
    /// `isVertexBuffersEnabled` and `getVertexBufferDistance()` (DBU).
    pub vertex_buffers: bool,
    pub vertex_buffer_distance: i32,
}

/// What the builder reads of the design.
pub struct HDesign {
    /// `block->getCoreArea()`.
    pub core: (i32, i32, i32, i32),
    /// The merged blockages (`mergeBlockages`), in DBU.
    pub blockages: Vec<(i32, i32, i32, i32)>,
    /// Every buffer master's width and height (DBU).
    pub master_size: HashMap<String, (i32, i32)>,
}

/// `LevelTopology`.
#[derive(Debug, Clone)]
pub struct Level {
    pub length: f64,
    pub output_slew: u32,
    pub output_cap: u32,
    pub remaining_length: u32,
    pub curr_wl: i32,
    pub wire_segments: Vec<u32>,
    pub points: Vec<P>,
    pub parents: Vec<u32>,
    pub sinks: Vec<Vec<P>>,
    pub driving: Vec<Option<usize>>,
}

const NO_PARENT: u32 = u32::MAX;

impl Level {
    fn new(length: f64) -> Level {
        Level { length, output_slew: 0, output_cap: 0, remaining_length: 0, curr_wl: 0, wire_segments: Vec::new(), points: Vec::new(), parents: Vec::new(), sinks: Vec::new(), driving: Vec::new() }
    }
    fn add_branching_point(&mut self, p: P, parent: u32) -> usize {
        self.points.push(p);
        self.parents.push(parent);
        self.sinks.resize(self.points.len(), Vec::new());
        self.driving.resize(self.points.len(), None);
        self.points.len() - 1
    }
    /// `getLargestSinkRegionHPWL`.
    fn largest_sink_region_hpwl(&self, wire_segment_unit: i32) -> f64 {
        let mut best = 0.0;
        for sinks in &self.sinks {
            if sinks.len() < 2 {
                continue;
            }
            let (mut min_x, mut min_y) = (sinks[0].x, sinks[0].y);
            let (mut max_x, mut max_y) = (min_x, min_y);
            for s in &sinks[1..] {
                min_x = min_x.min(s.x);
                max_x = max_x.max(s.x);
                min_y = min_y.min(s.y);
                max_y = max_y.max(s.y);
            }
            let hpwl = (max_x - min_x) + (max_y - min_y);
            if hpwl > best {
                best = hpwl;
            }
        }
        best * f64::from(wire_segment_unit)
    }
}

/// The builder's state (`TreeBuilder` + `HTreeBuilder` members).
pub struct HTree<'a> {
    o: &'a HOptions,
    d: &'a HDesign,
    pub lut: &'a mut Lut,
    pub clock: Clock,
    pub tree_type: TreeType,
    sink_region: Region,
    pub topology: Vec<Level>,
    /// `mapLocationToSink_` (exact keys).
    map_location_to_sink: HashMap<(u64, u64), InstRef>,
    top_level_sinks_clustered: Vec<(f32, f32)>,
    wire_segment_unit: i32,
    min_input_cap: u32,
    num_max_leaf_sinks: u32,
    min_length_sink_region: u32,
    clock_tree_max_depth: u32,
    /// `occupiedLocations_` (exact keys: see the divergence register).
    occupied: HashSet<(u64, u64)>,
    buffer_width: f64,
    buffer_height: f64,
    pub tree_buf_levels: u32,
    pub tree_level_buffers: Vec<usize>,
    pub first_level_sink_drivers: Vec<usize>,
    pub second_level_sink_drivers: Vec<usize>,
    pub top_buffer_name: String,
}

/// The min-clustering thresholds (`min_clustering_sinks_`, `min_clustering_macro_sinks_`).
const MIN_CLUSTERING_SINKS: usize = 200;
const MIN_CLUSTERING_MACRO_SINKS: usize = 10;

impl<'a> HTree<'a> {
    pub fn new(o: &'a HOptions, d: &'a HDesign, lut: &'a mut Lut, clock: Clock, tree_type: TreeType) -> HTree<'a> {
        HTree {
            o,
            d,
            lut,
            clock,
            tree_type,
            sink_region: Region::default(),
            topology: Vec::new(),
            map_location_to_sink: HashMap::new(),
            top_level_sinks_clustered: Vec::new(),
            wire_segment_unit: 0,
            min_input_cap: 0,
            num_max_leaf_sinks: 0,
            min_length_sink_region: 0,
            clock_tree_max_depth: 0,
            occupied: HashSet::new(),
            buffer_width: 0.0,
            buffer_height: 0.0,
            tree_buf_levels: 0,
            tree_level_buffers: Vec::new(),
            first_level_sink_drivers: Vec::new(),
            second_level_sink_drivers: Vec::new(),
            top_buffer_name: String::new(),
        }
    }

    fn is_macro(&self) -> bool {
        self.tree_type == TreeType::Macro
    }

    // ---- TreeBuilder -------------------------------------------------------------------------

    /// `initBlockages`: the tree buffer's (else the root buffer's) size, then the blockages
    /// (already merged by the caller), CTS-0201.
    pub fn init_blockages(&mut self, log: &mut Log) -> Res<()> {
        let buffer = self.o.tree_buffer.clone().unwrap_or_else(|| self.o.root_buffer.clone());
        let unit = f64::from(self.lut.length_unit);
        match self.d.master_size.get(&buffer) {
            Some(&(w, h)) => {
                self.buffer_width = f64::from(w) / unit;
                self.buffer_height = f64::from(h) / unit;
                log.debug("CTS", "legalizer", 3, || format!("buf width= {:.3} buf ht= {:.3} scalingUnit={}", self.buffer_width, self.buffer_height, self.lut.length_unit));
            }
            None => return log.error("CTS", 77, format!("No physical master cell found for cell {buffer}.")),
        }
        log.info("CTS", 201, format!("{} blockages from hard placement blockages and placed macros will be used.", self.d.blockages.len()));
        Ok(())
    }

    /// `computeDist`: the distance plus each end's insertion delay (none is modelled: a sink with
    /// one is refused upstream).
    fn compute_dist(&self, a: P, b: P) -> f64 {
        a.dist(b) + 0.0 + 0.0
    }

    fn is_inside_bbox(x: f64, y: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
        x > x1 && x < x2 && y > y1 && y < y2
    }

    fn is_along_bbox(x: f64, y: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> bool {
        if (fuzzy_equal(x, x1, 1e-6) || fuzzy_equal(x, x2, 1e-6)) && fuzzy_equal_or_greater(y, y1) && fuzzy_equal_or_smaller(y, y2) {
            return true;
        }
        (fuzzy_equal(y, y1, 1e-6) || fuzzy_equal(y, y2, 1e-6)) && fuzzy_equal_or_greater(x, x1) && fuzzy_equal_or_smaller(x, x2)
    }

    /// `findBlockage`: the first blockage containing the point (strictly), scaled down.
    fn find_blockage(&self, p: P, scaling: f64) -> Option<(f64, f64, f64, f64)> {
        find_blockage(&self.d.blockages, p, scaling)
    }

    /// `checkLegalitySpecial`.
    fn check_legality_special(&self, loc: P, x1: f64, y1: f64, x2: f64, y2: f64, scaling: i32) -> bool {
        if !self.is_occupied_loc(loc) && Self::is_along_bbox(loc.x, loc.y, x1, y1, x2, y2) {
            return true;
        }
        self.check_legality_loc(loc, scaling)
    }

    /// `TreeBuilder::legalizeOneBuffer`: a buffer on a blockage moves to the nearest free candidate
    /// around its edges.
    fn tree_legalize_one_buffer(&self, loc: P, buffer: &str) -> P {
        self.legalizer_view().tree_legalize_one_buffer(loc, buffer)
    }

    /// The legalizer's state as a view the one-buffer legalizer reads.
    fn legalizer_view(&self) -> LegalizerView<'_> {
        LegalizerView { obstruction_aware: self.o.obstruction_aware, length_unit: self.lut.length_unit, wire_segment_unit: self.wire_segment_unit, blockages: &self.d.blockages, master_size: &self.d.master_size, occupied: &self.occupied, map_location_to_sink: &self.map_location_to_sink }
    }

    /// A copy of the legalizer's state after the tree is built: later stages (the latency
    /// balancer's delay buffers) legalize through the root tree's (`root_->legalizeOneBuffer`).
    pub fn legalizer(&self) -> Legalizer {
        Legalizer { obstruction_aware: self.o.obstruction_aware, length_unit: self.lut.length_unit, wire_segment_unit: self.wire_segment_unit, blockages: self.d.blockages.clone(), master_size: self.d.master_size.clone(), occupied: self.occupied.clone(), map_location_to_sink: self.map_location_to_sink.keys().copied().collect() }
    }

    /// `checkLegalityLoc`: not occupied, inside the core, on no blockage.
    fn check_legality_loc(&self, p: P, scaling: i32) -> bool {
        if self.occupied.contains(&p.key()) {
            return false;
        }
        let (lx, ly, ux, uy) = self.d.core;
        let x = (p.x * f64::from(scaling)) as i32;
        let y = (p.y * f64::from(scaling)) as i32;
        if !(lx <= x && x <= ux && ly <= y && y <= uy) {
            return false;
        }
        self.find_blockage(p, f64::from(scaling)).is_none()
    }

    fn is_occupied_loc(&self, p: P) -> bool {
        self.occupied.contains(&p.key())
    }
    fn commit_loc(&mut self, p: P) {
        self.occupied.insert(p.key());
    }
    fn commit_move_loc(&mut self, old: P, new: P) {
        self.occupied.remove(&old.key());
        self.occupied.insert(new.key());
    }

    // ---- HTreeBuilder ------------------------------------------------------------------------

    /// `HTreeBuilder::legalizeOneBuffer`: the tree legalizer, then the collision jitter.
    fn legalize_one_buffer(&self, loc: P, buffer: &str) -> P {
        let legal = self.tree_legalize_one_buffer(loc, buffer);
        self.resolve_location_collision(legal)
    }

    /// `resolveLocationCollision`: shift right by 1/unit steps until no sink sits at the point (as
    /// floats).
    fn resolve_location_collision(&self, legal: P) -> P {
        self.legalizer_view().resolve_location_collision(legal)
    }

    /// `run`: the call sequence.
    pub fn run(&mut self, log: &mut Log) -> Res<()> {
        let (cluster_diameter, cluster_size, use_max_diameter, use_max_size) = if self.is_macro() {
            (self.o.macro_max_diameter, self.o.macro_sink_clustering_size, self.o.macro_max_diameter_set, self.o.macro_sink_clustering_size_set)
        } else {
            (self.o.max_diameter, self.o.sink_clustering_size, self.o.max_diameter_set, self.o.sink_clustering_size_set)
        };
        let use_max_cap = !(use_max_size && use_max_diameter);
        log.info("CTS", 27, format!("Generating H-Tree topology for net {}.", self.clock.name));
        log.info("CTS", 28, format!(" Total number of sinks: {}.", self.clock.sinks.len()));
        if self.o.sink_clustering {
            let kind = if self.is_macro() { "Macro " } else { "Register" };
            if !use_max_cap {
                log.info("CTS", 29, format!(" {kind} sinks will be clustered in groups of up to {cluster_size} and with maximum cluster diameter of {cluster_diameter:.1} um."));
            } else if use_max_diameter && !use_max_size {
                log.info("CTS", 59, format!(" {kind} sinks will be clustered with maximum cluster diameter of {cluster_diameter:.1} um and based on buffer max cap."));
            } else if !use_max_diameter && use_max_size {
                log.info("CTS", 60, format!(" {kind} sinks will be clustered in groups of up to {cluster_size} and based on buffer max cap."));
            } else {
                log.info("CTS", 90, " Sinks will be clustered based on buffer max cap.");
            }
        }
        log.info("CTS", 30, format!(" Number of static layers: {}.", self.o.num_static_layers));
        self.clock_tree_max_depth = self.o.clock_tree_max_depth;
        self.min_input_cap = self.lut.actual_min_input_cap;
        self.num_max_leaf_sinks = self.o.num_max_leaf_sinks;
        self.min_length_sink_region = self.lut.min_segment_length * 2;

        self.init_sink_region(log)?;

        let mut prev_max_hpwl = f64::MAX;
        for level in 1..=self.clock_tree_max_depth {
            let num_sinks_per_sub_region = self.compute_number_of_sinks_per_sub_region(level);
            let (w, h) = self.compute_sub_region_size(level);
            if self.is_sub_region_too_small(w, h) {
                if self.o.fake_lut_entries {
                    create_fake_entries(log, self.lut, self.min_length_sink_region, 1);
                    self.min_length_sink_region = 1;
                } else {
                    log.info("CTS", 31, format!(" Stop criterion found. Min length of sink region is ({}).", self.min_length_sink_region));
                    break;
                }
            }
            self.compute_level_topology(log, level, w, h)?;
            if self.is_number_of_sinks_too_small(num_sinks_per_sub_region) {
                let max_sinks = if self.o.max_fanout != 0 { self.o.max_fanout } else { self.num_max_leaf_sinks };
                if self.o.max_wl != 0 {
                    let max_hpwl = self.topology.last().expect("a level").largest_sink_region_hpwl(self.wire_segment_unit);
                    if max_hpwl < f64::from(self.o.max_wl) {
                        log.info("CTS", 38, format!(" Stop criterion found. Sink region hpwl is smaller than max wirelength ({:.3} um) and max number of sinks is {max_sinks}.", f64::from(self.o.max_wl) / f64::from(self.o.db_units)));
                        break;
                    }
                    if max_hpwl >= prev_max_hpwl {
                        log.info("CTS", 54, format!(" Stop criterion found. Sink region hpwl ({:.3} um) did not improve and max number of sinks is {max_sinks}.", max_hpwl / f64::from(self.o.db_units)));
                        break;
                    }
                    prev_max_hpwl = max_hpwl;
                } else {
                    log.info("CTS", 32, format!(" Stop criterion found. Max number of sinks is {max_sinks}."));
                    break;
                }
            }
        }

        if self.topology.is_empty() {
            self.create_single_buffer_clock_net(log);
            self.tree_buf_levels += 1;
            return Ok(());
        }
        self.clock.max_level = self.topology.len() as u32;
        if self.o.obstruction_aware {
            self.legalize(log);
        }
        self.probe_legal_points(log);
        self.create_clock_sub_nets(log)
    }

    /// `isNumberOfSinksTooSmall`.
    fn is_number_of_sinks_too_small(&self, n: u32) -> bool {
        if self.o.max_fanout != 0 {
            return n < self.o.max_fanout;
        }
        n < self.num_max_leaf_sinks
    }

    /// `isSubRegionTooSmall`.
    fn is_sub_region_too_small(&self, w: f64, h: f64) -> bool {
        w < f64::from(self.min_length_sink_region) || h < f64::from(self.min_length_sink_region)
    }

    /// `isVertical`: odd levels run along the longer side.
    fn is_vertical(&self, level: u32) -> bool {
        (level % 2) == u32::from(self.sink_region.height >= self.sink_region.width)
    }
    fn is_horizontal(&self, level: u32) -> bool {
        !self.is_vertical(level)
    }
    fn grid_size_x(level: u32) -> u32 {
        2f64.powi(level.div_ceil(2) as i32) as u32
    }
    fn grid_size_y(level: u32) -> u32 {
        2f64.powi((level / 2) as i32) as u32
    }

    /// `initSinkRegion`.
    fn init_sink_region(&mut self, log: &mut Log) -> Res<()> {
        let unit_dbu = self.lut.length_unit;
        let db_units = self.o.db_units;
        self.wire_segment_unit = unit_dbu as i32;
        let (cluster_diameter, cluster_size, min_clustering) = if self.is_macro() {
            (self.o.macro_max_diameter, self.o.macro_sink_clustering_size, MIN_CLUSTERING_MACRO_SINKS)
        } else {
            (self.o.max_diameter, self.o.sink_clustering_size, MIN_CLUSTERING_SINKS)
        };
        log.info("CTS", 20, format!(" Wire segment unit: {}  dbu ({} um).", self.wire_segment_unit, unit_dbu as i32 / db_units));
        if self.o.simple_segments {
            // `int32 / unsigned`: unsigned division.
            let remaining = self.o.buffer_distance as u32 / (unit_dbu * 2);
            log.info("CTS", 21, format!(" Distance between buffers: {remaining} units ({} um).", self.o.buffer_distance / db_units));
            if self.o.vertex_buffers {
                let vertex = self.o.vertex_buffer_distance as u32 / (unit_dbu * 2);
                log.info("CTS", 22, format!(" Branch length for Vertex Buffer: {vertex} units ({} um).", self.o.vertex_buffer_distance / db_units));
            }
        }
        let (top, insts) = self.init_top_level_sinks();
        let max_diameter = ((cluster_diameter * f64::from(db_units)) / f64::from(self.wire_segment_unit)) as f32;
        log.debug("CTS", "clustering", 1, || format!("maxDiameter={max_diameter:.3} = origMaxDiam={} * dbUnits={db_units} / wireSegmentUnit_={}", fmt_ostream(cluster_diameter), self.wire_segment_unit));
        self.pre_sink_clustering(log, &top, &insts, max_diameter, cluster_size, false)?;
        if top.len() <= min_clustering || !self.o.sink_clustering {
            let (x0, y0, x1, y1) = self.clock.compute_sink_region();
            log.info("CTS", 23, format!(" Original sink region: [({x0}, {y0}), ({x1}, {y1})]."));
            // `Box<int>::normalize`: the corners (max = min + extent) times 1/unit.
            let f = 1.0 / f64::from(self.wire_segment_unit);
            let (w, h) = (x1 - x0, y1 - y0);
            self.sink_region = Region::from_corners(f64::from(x0) * f, f64::from(y0) * f, f64::from(x0 + w) * f, f64::from(y0 + h) * f);
        } else {
            if self.top_level_sinks_clustered.len() > 400 && self.o.sink_clustering_levels > 0 {
                let (locs, insts) = self.init_second_level_sinks();
                let size = f64::from(cluster_size).sqrt().ceil() as u32;
                self.pre_sink_clustering(log, &locs, &insts, max_diameter * 4.0, size, true)?;
            }
            // `computeSinkRegionClustered`: the clustered locations' float box.
            let (mut x0, mut x1, mut y0, mut y1) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
            for &(x, y) in &self.top_level_sinks_clustered {
                x0 = if x < x0 { x } else { x0 };
                x1 = if x1 < x { x } else { x1 };
                y0 = if y < y0 { y } else { y0 };
                y1 = if y1 < y { y } else { y1 };
            }
            self.sink_region = Region::from_corners(f64::from(x0), f64::from(y0), f64::from(x1), f64::from(y1));
        }
        let r = self.sink_region;
        log.info("CTS", 24, format!(" Normalized sink region: [({}, {}), ({}, {})].", fmt_ostream(r.min_x), fmt_ostream(r.min_y), fmt_ostream(r.max_x()), fmt_ostream(r.max_y())));
        log.info("CTS", 25, format!("    Width:  {:.4}.", r.width));
        log.info("CTS", 26, format!("    Height: {:.4}.", r.height));
        log.trace(format!("VYGHTR|{}|{:016x}|{:016x}|{:016x}|{:016x}", self.wire_segment_unit, r.min_x.to_bits(), r.min_y.to_bits(), r.width.to_bits(), r.height.to_bits()));
        Ok(())
    }

    /// `initTopLevelSinks`: each sink's location in units, as floats.
    fn init_top_level_sinks(&self) -> (Vec<(f32, f32)>, Vec<InstRef>) {
        let u = self.wire_segment_unit as f32;
        let locs = self.clock.sinks.iter().map(|s| (s.x as f32 / u, s.y as f32 / u)).collect();
        (locs, (0..self.clock.sinks.len()).map(InstRef::Sink).collect())
    }

    /// `initSecondLevelSinks`: the first level's cluster buffers, by their locations.
    fn init_second_level_sinks(&self) -> (Vec<(f32, f32)>, Vec<InstRef>) {
        let mut locs = Vec::new();
        let mut insts = Vec::new();
        for &(x, y) in &self.top_level_sinks_clustered {
            locs.push((x, y));
            let k = P::new(f64::from(x), f64::from(y)).key();
            insts.push(*self.map_location_to_sink.get(&k).expect("a clustered location maps to its buffer"));
        }
        (locs, insts)
    }

    /// `preSinkClustering`: (first level) every sink mapped by its location; without clustering the
    /// sinks are the top level's. With it, the sinks are matched into clusters, each driven by a
    /// sink buffer at its centre, and the cluster buffers become the top level's sinks.
    #[allow(clippy::too_many_arguments)]
    fn pre_sink_clustering(&mut self, log: &mut Log, points: &[(f32, f32)], insts: &[InstRef], max_diameter: f32, cluster_size: u32, second_level: bool) -> Res<()> {
        let (max_diameter_set, cluster_size_set, min_clustering) = if self.is_macro() {
            (self.o.macro_max_diameter_set, self.o.macro_sink_clustering_size_set, MIN_CLUSTERING_MACRO_SINKS)
        } else {
            (self.o.max_diameter_set, self.o.sink_clustering_size_set, MIN_CLUSTERING_SINKS)
        };
        if !second_level {
            let u = self.wire_segment_unit as f32;
            for (i, s) in self.clock.sinks.iter().enumerate() {
                let norm = P::new(f64::from(s.x as f32 / u), f64::from(s.y as f32 / u));
                // A later sink at the same location replaces an earlier one.
                self.map_location_to_sink.insert(norm.key(), InstRef::Sink(i));
                if !fuzzy_equal(s.insertion_delay, 0.0, 1e-6) {
                    return refuse("a sink with insertion delay is not modelled");
                }
            }
        }
        if points.len() <= min_clustering || !self.o.sink_clustering {
            self.top_level_sinks_clustered = points.to_vec();
            return Ok(());
        }
        let mut matching = crate::sink_clustering::SinkClustering::new(self.o.sink_buffer_input_cap, self.o.cap_per_dbu, self.o.db_units, max_diameter_set, cluster_size_set);
        for (i, &(x, y)) in points.iter().enumerate() {
            matching.add_point(f64::from(x), f64::from(y));
            let cap = self.clock.inst(insts[i]).input_cap;
            // A cluster buffer's cap is unset (the second level): the sink buffer's stands in.
            matching.add_cap(if cap == 0.0 { self.o.sink_buffer_input_cap as f32 } else { cap });
        }
        log.debug("CTS", "clustering", 1, || format!("**** match.run({cluster_size}, {}, {}) ****", crate::log::fmt_f32(max_diameter), self.wire_segment_unit));
        // No insertion delay is modelled: the lookup always misses.
        let (mut best_size, mut best_diameter) = (0u32, 0.0f32);
        if let Some((s, dia)) = matching.run_with_insertion_delay(cluster_size, max_diameter, self.wire_segment_unit, &mut |_, _| 0.0) {
            best_size = s;
            best_diameter = dia;
        }
        let (mut size_log, mut diameter_log) = ("", "");
        if !max_diameter_set {
            diameter_log = "max ";
            best_diameter = matching.max_diameter() as f32;
        }
        if !cluster_size_set {
            size_log = "max ";
            best_size = matching.max_size() as u32;
        }
        let shown = (best_diameter / self.o.db_units as f32 * self.wire_segment_unit as f32).round();
        if cluster_size_set || max_diameter_set {
            log.info("CTS", 204, format!("A clustering solution was found from {size_log}clustering size of {best_size} and {diameter_log}clustering diameter of {shown:.0}."));
            log.info("CTS", 205, "Better solution may be possible if either -sink_clustering_size, -sink_clustering_max_diameter, or both options are omitted to enable automatic clustering.");
        } else {
            log.info("CTS", 206, format!("Best clustering solution was found from max clustering size of {best_size} and max clustering diameter of {shown:.0}."));
        }
        let mut cluster_count = 0u32;
        let mut new_locations: Vec<(f32, f32)> = Vec::new();
        let sink_buffer = self.o.sink_buffer.clone();
        let solution = matching.solution().to_vec();
        for cluster in &solution {
            if !cluster.is_empty() {
                let mut members: Vec<InstRef> = Vec::new();
                for &pi in cluster {
                    let (px, py) = (f64::from(points[pi as usize].0), f64::from(points[pi as usize].1));
                    let Some(&m) = self.map_location_to_sink.get(&P::new(px, py).key()) else {
                        return log.error("CTS", 79, "Sink not found.");
                    };
                    members.push(m);
                }
                let member_points: Vec<(f32, f32)> = cluster.iter().map(|&pi| points[pi as usize]).collect();
                let (cx, cy) = cluster_center(&member_points);
                let center = P::new(f64::from(cx), f64::from(cy));
                let at = self.legalize_one_buffer(center, &sink_buffer);
                self.commit_move_loc(center, at);
                let base = if second_level { "clkbuf_leaf2_" } else { "clkbuf_leaf_" };
                let unit = f64::from(self.wire_segment_unit);
                let b = self.clock.add_clock_buffer(&format!("{base}{cluster_count}"), &sink_buffer, (at.x * unit) as i32, (at.y * unit) as i32);
                if !center.same(at) {
                    log.debug("CTS", "legalizer", 2, || format!("preSinkClustering legalizeOneBuffer {base}{cluster_count}: {} => {}", fmt_p(center), fmt_p(at)));
                }
                if !second_level {
                    self.first_level_sink_drivers.push(b);
                } else {
                    self.second_level_sink_drivers.push(b);
                }
                let net_base = if second_level { "clknet_leaf2_" } else { "clknet_leaf_" };
                let sub = self.clock.add_sub_net(&format!("{net_base}{cluster_count}"));
                self.clock.subnets[sub].insts.push(InstRef::Buffer(b));
                self.clock.subnets[sub].insts.extend(members);
                if !second_level {
                    self.clock.subnets[sub].leaf = true;
                }
                let point = (at.x as f32, at.y as f32);
                new_locations.push(point);
                self.map_location_to_sink.insert(P::new(f64::from(point.0), f64::from(point.1)).key(), InstRef::Buffer(b));
            }
            cluster_count += 1;
        }
        self.top_level_sinks_clustered = new_locations;
        if cluster_count != 0 {
            self.tree_buf_levels += 1;
        }
        log.info("CTS", 19, format!(" Total number of sinks after clustering: {}.", self.top_level_sinks_clustered.len()));
        Ok(())
    }

    /// `computeNumberOfSinksPerSubRegion`.
    fn compute_number_of_sinks_per_sub_region(&self, level: u32) -> u32 {
        let min_clustering = if self.is_macro() { MIN_CLUSTERING_MACRO_SINKS } else { MIN_CLUSTERING_SINKS };
        let total = if self.clock.sinks.len() > min_clustering && self.o.sink_clustering { self.top_level_sinks_clustered.len() } else { self.clock.sinks.len() };
        let roots = 2f64.powi(level as i32) as u32;
        let per_root = total as f64 / f64::from(roots);
        per_root.ceil() as u32
    }

    /// `computeSubRegionSize`.
    fn compute_sub_region_size(&self, level: u32) -> (f64, f64) {
        let (gx, gy) = if self.is_vertical(1) { (Self::grid_size_y(level), Self::grid_size_x(level)) } else { (Self::grid_size_x(level), Self::grid_size_y(level)) };
        (self.sink_region.width / f64::from(gx), self.sink_region.height / f64::from(gy))
    }

    /// `computeLevelTopology`: the level's segment length, its wire segments, its branching
    /// points.
    fn compute_level_topology(&mut self, log: &mut Log, level: u32, width: f64, height: f64) -> Res<()> {
        let n = self.compute_number_of_sinks_per_sub_region(level);
        log.report(format!(" Level {level}"));
        log.report(format!("    Direction: {}", if self.is_vertical(level) { "Vertical" } else { "Horizontal" }));
        log.report(format!("    Sinks per sub-region: {n}"));
        log.report(format!("    Sub-region size: {width:.4} X {height:.4}"));
        let min_length = self.min_length_sink_region;
        let clamped = min_length.max(1);
        let side = if self.is_vertical(level) { height } else { width };
        let mut segment_length = ((side / f64::from(clamped)).round() * f64::from(min_length) / 2.0) as u32;
        segment_length = segment_length.max(1);
        let mut topology = Level::new(f64::from(segment_length));
        log.info("CTS", 34, format!("    Segment length (rounded): {segment_length}."));

        let unit = self.lut.length_unit as i32;
        let vertex_buffer_length = (self.o.vertex_buffer_distance as u32 / (self.lut.length_unit * 2)) as i32;
        let mut remaining_length = self.o.buffer_distance / unit;
        let mut curr_wl = 0i32;
        let mut input_cap = self.min_input_cap;
        let mut input_slew = 1u32;
        if level > 1 {
            let prev = &self.topology[(level - 2) as usize];
            input_cap = prev.output_cap;
            input_slew = prev.output_slew;
            remaining_length = prev.remaining_length as i32;
            curr_wl = prev.curr_wl;
        }
        let slew_threshold = self.o.max_slew;
        let init_tolerance = 1u32;
        let wirelength_threshold: i32 = if self.o.max_wl == 0 {
            (slew_threshold * self.lut.max_segment_length) as i32
        } else {
            (self.o.max_wl as u32 / self.o.wire_segment_unit) as i32
        };
        log.debug("CTS", "tech char", 1, || format!("slew threshold = {slew_threshold}"));
        log.debug("CTS", "tech char", 1, || format!("wirelength threshold = {wirelength_threshold}"));
        let mut length = 0u32;
        let mut char_seg_length = self.lut.max_segment_length as i32;
        while char_seg_length >= 1 {
            let num_wires = (segment_length - length) / char_seg_length as u32;
            if num_wires >= 1 {
                for wire_count in 0..num_wires {
                    log.debug("CTS", "tech char", 1, || format!("curr wl = {curr_wl}"));
                    let (mut out_cap, mut out_slew) = (0u32, 0u32);
                    let csl = char_seg_length as u32;
                    let key = if self.o.simple_segments {
                        remaining_length -= char_seg_length;
                        // `segmentLength >= vertexBufferLength` compares unsigned.
                        if segment_length >= vertex_buffer_length as u32 && wire_count + 1 >= num_wires && self.o.vertex_buffers {
                            remaining_length = 0;
                            let k = compute_min_delay_segment_forced(self.lut, csl, true, remaining_length);
                            remaining_length += self.o.buffer_distance / unit;
                            k
                        } else if remaining_length <= 0 {
                            let k = compute_min_delay_segment_forced(self.lut, csl, true, remaining_length);
                            remaining_length += self.o.buffer_distance / unit;
                            k
                        } else {
                            compute_min_delay_segment_forced(self.lut, csl, false, remaining_length)
                        }
                    } else {
                        compute_min_delay_segment(self.lut, csl, input_slew, input_cap, slew_threshold, wirelength_threshold, init_tolerance, &mut out_slew, &mut out_cap, &mut curr_wl)
                    };
                    if key == u32::MAX {
                        continue;
                    }
                    length += char_seg_length as u32;
                    report_segment(log, self.lut, key);
                    input_cap = out_cap.max(self.min_input_cap);
                    input_slew = out_slew;
                    topology.wire_segments.push(key);
                    topology.remaining_length = remaining_length as u32;
                }
                if length == segment_length {
                    break;
                }
            }
            char_seg_length -= 1;
        }
        topology.output_slew = input_slew;
        topology.output_cap = input_cap;
        topology.curr_wl = curr_wl;
        self.compute_branching_points(level, &mut topology);
        self.topology.push(topology);
        self.probe_level(log, level);
        Ok(())
    }

    fn probe_level(&self, log: &mut Log, level: u32) {
        let t = self.topology.last().expect("a level");
        let w: Vec<String> = t.wire_segments.iter().map(|k| k.to_string()).collect();
        log.trace(format!("VYGHTL|{level}|{:016x}|{}|{}|{}|{}|{}", t.length.to_bits(), t.output_slew, t.output_cap, t.remaining_length, t.curr_wl, w.join(",")));
        for (i, p) in t.points.iter().enumerate() {
            let s: Vec<String> = t.sinks[i].iter().map(|q| format!("{:016x}:{:016x}", q.x.to_bits(), q.y.to_bits())).collect();
            log.trace(format!("VYGHTB|{level}|{i}|{:016x}|{:016x}|{}|{}", p.x.to_bits(), p.y.to_bits(), t.parents[i], s.join(";")));
        }
    }

    fn probe_legal_points(&mut self, log: &mut Log) {
        let c = self.sink_region.center();
        log.trace(format!("VYGHTG|0|0|{:016x}|{:016x}", c.x.to_bits(), c.y.to_bits()));
        for (l, t) in self.topology.iter().enumerate() {
            for (i, p) in t.points.iter().enumerate() {
                log.trace(format!("VYGHTG|{}|{i}|{:016x}|{:016x}", l + 1, p.x.to_bits(), p.y.to_bits()));
            }
        }
    }

    /// `computeBranchingPoints`: two points a segment length either side of each parent, refined
    /// by clustering that parent's sinks.
    fn compute_branching_points(&mut self, level: u32, topology: &mut Level) {
        let horizontal = self.is_horizontal(level);
        let split = |root: P, len: f64| -> (P, P) {
            let (mut low, mut high) = (root, root);
            if horizontal {
                low.x -= len;
                high.x += len;
            } else {
                low.y -= len;
                high.y += len;
            }
            (low, high)
        };
        if level == 1 {
            let root = self.sink_region.center();
            let (low, high) = split(root, topology.length);
            let i1 = topology.add_branching_point(low, NO_PARENT);
            let i2 = topology.add_branching_point(high, NO_PARENT);
            let sinks = self.top_level_sinks_clustered.clone();
            self.refine_branching_points_with_clustering(topology, level, i1, i2, root, &sinks);
            return;
        }
        let parent = self.topology[(level - 2) as usize].clone();
        for (idx, &root) in parent.points.iter().enumerate() {
            let (low, high) = split(root, topology.length);
            let i1 = topology.add_branching_point(low, idx as u32);
            let i2 = topology.add_branching_point(high, idx as u32);
            let sinks = compute_branch_sinks(&parent, idx);
            self.refine_branching_points_with_clustering(topology, level, i1, i2, root, &sinks);
        }
    }

    /// `refineBranchingPointsWithClustering`: two-means of the sinks from the two points; past the
    /// static layers the points move to the means; each cluster's sinks join its point.
    fn refine_branching_points_with_clustering(&self, topology: &mut Level, level: u32, i1: usize, i2: usize, root: P, sinks: &[(f32, f32)]) {
        let mut engine = Clustering::new(sinks, root.x as f32, root.y as f32);
        let mut means = vec![(topology.points[i1].x as f32, topology.points[i1].y as f32), (topology.points[i2].x as f32, topology.points[i2].y as f32)];
        let cap = if self.o.max_fanout != 0 { (sinks.len() as f64 * 0.5) as u32 } else { (sinks.len() as f64 * self.o.clustering_capacity) as u32 };
        engine.iter_kmeans(1, means.len() as u32, cap, 5, self.o.clustering_power, &mut means);
        if (self.o.num_static_layers as i32) - (level as i32) < 0 {
            topology.points[i1] = P::new(f64::from(means[0].0), f64::from(means[0].1));
            topology.points[i2] = P::new(f64::from(means[1].0), f64::from(means[1].1));
        }
        for (ci, cluster) in engine.get_clusters().iter().enumerate() {
            for &s in cluster {
                let loc = P::new(f64::from(sinks[s as usize].0), f64::from(sinks[s as usize].1));
                topology.sinks[if ci == 0 { i1 } else { i2 }].push(loc);
            }
        }
    }

    /// `findSibling`: the other point of the same parent (itself when none).
    fn find_sibling(t: &Level, i: usize, par: u32) -> usize {
        (0..t.points.len()).find(|&idx| idx != i && t.parents[idx] == par).unwrap_or(i)
    }

    /// `weightedDistance`.
    fn weighted_distance(&self, new: P, old: P, sinks: &[P]) -> f64 {
        let mut dist = 0.0;
        for &s in sinks {
            dist += self.compute_dist(new, s);
            dist += self.compute_dist(new, old);
        }
        dist
    }

    /// `scalePosition`.
    fn scale_position(&self, loc: &mut P, par: P, leng: f64, scale: f64) {
        let (px, py) = (par.x, par.y);
        let d = self.compute_dist(*loc, par);
        let (x, y);
        if d > 0.0 {
            let delta = d * scale;
            let mut dx = loc.x - px;
            let mut dy = loc.y - py;
            dx += if dx > 0.0 { delta } else { -delta };
            dy += if dy > 0.0 { -delta } else { delta };
            let s = leng / d;
            x = px + dx * s;
            y = py + dy * s;
        } else {
            x = px + leng / 2.0;
            y = py + leng / 2.0;
        }
        loc.x = x;
        loc.y = y;
    }

    /// `findLegalLocations`: the blockage's corners, the point's straight-line projections onto
    /// its edges, and where the parent→point line meets them.
    #[allow(clippy::too_many_arguments)]
    fn find_legal_locations(&self, log: &mut Log, parent: P, branch: P, x1: f64, y1: f64, x2: f64, y2: f64) -> Vec<P> {
        let mut points = Vec::new();
        let mut add = |x: f64, y: f64| {
            if !P::new(x, y).same(parent) && Self::is_along_bbox(x, y, x1, y1, x2, y2) {
                points.push(P::new(x, y));
            }
        };
        add(x1, y1);
        add(x1, y2);
        add(x2, y2);
        add(x2, y1);
        let (bx, by) = (branch.x, branch.y);
        add(bx, y1);
        add(bx, y2);
        add(x1, by);
        add(x2, by);
        let dx = parent.x - bx;
        let dy = parent.y - by;
        let m = dy / dx;
        add(x1, m * (x1 - bx) + by);
        add(x2, m * (x2 - bx) + by);
        add((y1 - by) / m + bx, y1);
        add((y2 - by) / m + bx, y2);
        if log.debug_on("CTS", "legalizer", 3) {
            log.report(format!("    branchPt:{} is not legal, parentPt:{} blockages:({x1:.3} {y1:.3}) ({x2:.3} {y2:.3})", fmt_p(branch), fmt_p(parent)));
            for p in &points {
                log.report(format!("      FLL candiate {}", fmt_p(*p)));
            }
        }
        points
    }

    /// `findBestLegalLocation`: the candidate closest to the target path length, then adjusted.
    #[allow(clippy::too_many_arguments)]
    fn find_best_legal_location(&self, log: &mut Log, target: f64, branch: P, parent: P, locs: &[P], sinks: &[P], x1: f64, y1: f64, x2: f64, y2: f64, scaling: i32, dir: Dir) -> P {
        let mut best = P::new(0.0, 0.0);
        let mut min_diff = f64::MAX;
        for &loc in locs {
            let mut dist = self.compute_dist(loc, parent);
            dist += self.compute_dist(loc, branch);
            let diff = (dist - target).abs();
            if log.debug_on("CTS", "legalizer", 3) {
                log.report(format!("      Loc {}: curr dist={dist:.3} target dist={target:.3} sink dist={:.3}", fmt_p(loc), self.weighted_distance(loc, branch, sinks)));
            }
            if diff < min_diff {
                min_diff = diff;
                best = loc;
            }
        }
        self.adjust_best_legal_location(log, target, best, parent, sinks, x1, y1, x2, y2, scaling, dir)
    }

    /// `adjustBestLegalLocation`: along the blockage first, then beyond it.
    #[allow(clippy::too_many_arguments)]
    fn adjust_best_legal_location(&self, log: &mut Log, target: f64, curr: P, parent: P, sinks: &[P], x1: f64, y1: f64, x2: f64, y2: f64, scaling: i32, dir: Dir) -> P {
        if fuzzy_equal(target, self.compute_dist(curr, parent), 1e-6) {
            return curr;
        }
        if let Some(best) = self.adjust_along_blockage(log, target, curr, parent, sinks, x1, y1, x2, y2, scaling) {
            return best;
        }
        self.adjust_beyond_blockage(log, curr, parent, target, sinks, scaling, dir)
    }

    /// `adjustAlongBlockage`: four moves along x or y that restore the target distance.
    #[allow(clippy::too_many_arguments)]
    fn adjust_along_blockage(&self, log: &mut Log, target: f64, curr: P, parent: P, sinks: &[P], x1: f64, y1: f64, x2: f64, y2: f64, scaling: i32) -> Option<P> {
        log.debug("CTS", "legalizer", 3, || format!("{} currDist={:.3} != targetDist={target:.3}, adjustAlongBlockage...", fmt_p(curr), self.compute_dist(curr, parent)));
        let (x, y, px, py) = (curr.x, curr.y, parent.x, parent.y);
        let no_best = P::new(f64::MIN_POSITIVE, f64::MIN_POSITIVE);
        let mut best = no_best;
        let mut best_sink_dist = f64::MAX;
        let mut candidates = Vec::new();
        let new_x = x;
        candidates.push(P::new(new_x, py - target + (px - new_x).abs()));
        candidates.push(P::new(new_x, py + target - (px - new_x).abs()));
        let new_y = y;
        candidates.push(P::new(px - target + (py - new_y).abs(), new_y));
        candidates.push(P::new(px + target - (py - new_y).abs(), new_y));
        for c in candidates {
            if fuzzy_equal(self.compute_dist(c, parent), target, 1e-6) && self.check_legality_special(c, x1, y1, x2, y2, scaling) {
                let sink_dist = self.weighted_distance(c, curr, sinks);
                if sink_dist < best_sink_dist {
                    best = c;
                    best_sink_dist = sink_dist;
                }
                log.debug("CTS", "legalizer", 3, || format!("adjustBestLegalLoc: branchPt move:{}=>{} is legal, dist={target:.3}, sinkDist={sink_dist:.3}", fmt_p(curr), fmt_p(c)));
            }
            log.debug("CTS", "legalizer", 3, || format!("adjustBestLegalLoc: branchPt move:{}=>{} is illegal or dist {:.3} != {target:.3}", fmt_p(curr), fmt_p(c), self.compute_dist(c, parent)));
        }
        (!best.same(no_best)).then_some(best)
    }

    /// `adjustBeyondBlockage`: points on the Manhattan square of the target radius around the
    /// parent, on the branch's side, plus where that square meets a blockage, plus small offsets;
    /// the legal one nearest the sinks.
    #[allow(clippy::too_many_arguments)]
    fn adjust_beyond_blockage(&self, log: &mut Log, branch: P, parent: P, target: f64, sinks: &[P], scaling: i32, dir: Dir) -> P {
        let (px, py) = (parent.x, parent.y);
        let mut candidates = Vec::new();
        let l50 = target * 0.5;
        let l25 = target * 0.25;
        let l75 = target * 0.75;
        match dir {
            Dir::North => {
                candidates.push(P::new(px, py + target));
                candidates.push(P::new(px + l50, py + l50));
                candidates.push(P::new(px - l50, py + l50));
                candidates.push(P::new(px + l25, py + l75));
                candidates.push(P::new(px - l25, py + l75));
            }
            Dir::East => {
                candidates.push(P::new(px + target, py));
                candidates.push(P::new(px + l50, py + l50));
                candidates.push(P::new(px + l50, py - l50));
                candidates.push(P::new(px + l75, py + l25));
                candidates.push(P::new(px + l75, py - l25));
            }
            Dir::South => {
                candidates.push(P::new(px, py - target));
                candidates.push(P::new(px + l50, py - l50));
                candidates.push(P::new(px - l50, py - l50));
                candidates.push(P::new(px + l25, py - l75));
                candidates.push(P::new(px - l25, py - l75));
            }
            Dir::West => {
                candidates.push(P::new(px - target, py));
                candidates.push(P::new(px - l50, py - l50));
                candidates.push(P::new(px - l50, py + l50));
                candidates.push(P::new(px - l75, py - l25));
                candidates.push(P::new(px - l75, py + l25));
            }
        }
        let corner = match dir {
            Dir::North => P::new(px, py + target),
            Dir::East => P::new(px + target, py),
            Dir::South => P::new(px, py - target),
            Dir::West => P::new(px - target, py),
        };
        self.add_candidate_points_along_blockage(log, corner, parent, target, scaling, &mut candidates, dir);

        let (bx, by) = (branch.x, branch.y);
        let no_best = P::new(f64::MIN_POSITIVE, f64::MIN_POSITIVE);
        let mut best = no_best;
        let mut best_sink_dist = f64::MAX;
        let min_x = 10.0 * self.buffer_width;
        let min_y = 10.0 * self.buffer_height;
        // Small offsets in x, then in y (each kept only on its side).
        let mut nx = bx + min_x;
        if dir == Dir::South {
            candidates.push(P::new(nx, py - target + (px - nx).abs()));
        }
        if dir == Dir::North {
            candidates.push(P::new(nx, py + target - (px - nx).abs()));
        }
        nx = bx - min_x;
        if dir == Dir::South {
            candidates.push(P::new(nx, py - target + (px - nx).abs()));
        }
        if dir == Dir::North {
            candidates.push(P::new(nx, py + target - (px - nx).abs()));
        }
        let mut ny = by + min_y;
        if dir == Dir::West {
            candidates.push(P::new(px - target + (py - ny).abs(), ny));
        }
        if dir == Dir::East {
            candidates.push(P::new(px + target - (py - ny).abs(), ny));
        }
        ny = by - min_y;
        if dir == Dir::West {
            candidates.push(P::new(px - target + (py - ny).abs(), ny));
        }
        if dir == Dir::East {
            candidates.push(P::new(px + target - (py - ny).abs(), ny));
        }
        for c in candidates {
            // `checkLegalityAndCost`.
            if fuzzy_equal(self.compute_dist(c, parent), target, 1e-6) && self.check_legality_loc(c, scaling) {
                let sink_dist = self.weighted_distance(c, branch, sinks);
                if sink_dist < best_sink_dist {
                    best = c;
                    best_sink_dist = sink_dist;
                }
                log.debug("CTS", "legalizer", 3, || format!("adjustBeyondBlockage: branchPt move:{}=>{} is legal, dist={target:.3}, sinkDist={sink_dist:.3}", fmt_p(branch), fmt_p(c)));
            } else {
                log.debug("CTS", "legalizer", 3, || format!("adjustBeyondBlockage: branchPt move:{}=>{} is illegal or dist {:.3} != {target:.3}", fmt_p(branch), fmt_p(c), self.compute_dist(c, parent)));
            }
        }
        if !best.same(no_best) {
            return best;
        }
        branch
    }

    /// `addCandidatePointsAlongBlockage`: where the square's side meets the blockage the corner
    /// sits in.
    #[allow(clippy::too_many_arguments)]
    fn add_candidate_points_along_blockage(&self, log: &mut Log, point: P, parent: P, target: f64, scaling: i32, candidates: &mut Vec<P>, dir: Dir) {
        let Some((x1, y1, x2, y2)) = self.find_blockage(point, f64::from(scaling)) else { return };
        let (px, py) = (parent.x, parent.y);
        log.debug("CTS", "legalizer", 3, || format!("  {} corner {} of Manhattan sqaure is inside blockage ({x1:.3} {y1:.3}) ({x2:.3} {y2:.3})", dir.name(), fmt_p(point)));
        let mut p2 = point;
        match dir {
            Dir::North => {
                p2.x = -target - py + px + y1;
                p2.y = y1;
                candidates.push(p2);
                p2.y = target + py + px - y1;
                candidates.push(p2);
            }
            Dir::East => {
                p2.x = x1;
                p2.y = target + py + px - x1;
                candidates.push(p2);
                p2.y = -target + py - px + x1;
                candidates.push(p2);
            }
            Dir::South => {
                p2.x = -target + py + px - y2;
                p2.y = y2;
                candidates.push(p2);
                p2.x = target - py + px + y2;
                candidates.push(p2);
            }
            Dir::West => {
                p2.x = x2;
                p2.y = target + py - px + x2;
                candidates.push(p2);
                p2.y = -target + py + px - x2;
                candidates.push(p2);
            }
        }
    }

    /// `legalize`: the top buffer, then each level's points in order (moved off a blockage,
    /// re-placed when the parent moved or the point is taken, else committed), then the dummies.
    fn legalize(&mut self, log: &mut Log) {
        if log.debug_on("CTS", "legalizer", 3) {
            log.report("HTree before legalization -------");
        }
        let old_top = self.sink_region.center();
        let new_top = self.legalize_one_buffer(old_top, &self.o.root_buffer.clone());
        self.sink_region.set_center(new_top);
        self.commit_move_loc(old_top, new_top);
        log.debug("CTS", "legalizer", 3, || format!("legalize: top buf loc:{}->{}", fmt_p(old_top), fmt_p(new_top)));
        let scaling = self.wire_segment_unit;
        for level_idx in 0..self.topology.len() {
            for buf_idx in 0..self.topology[level_idx].points.len() {
                let branch = self.topology[level_idx].points[buf_idx];
                let parent_idx = self.topology[level_idx].parents[buf_idx];
                let parent = if level_idx == 0 { new_top } else { self.topology[level_idx - 1].points[parent_idx as usize] };
                let dir = if self.is_horizontal(level_idx as u32 + 1) {
                    if branch.x - parent.x > 0.0 { Dir::East } else { Dir::West }
                } else if branch.y - parent.y > 0.0 {
                    Dir::North
                } else {
                    Dir::South
                };
                let sinks = self.topology[level_idx].sinks[buf_idx].clone();
                let length = self.topology[level_idx].length;
                let leng = self.compute_dist(branch, parent);
                if log.debug_on("CTS", "legalizer", 3) {
                    log.report(format!("  HTree level*{level_idx}* bufId*{buf_idx}*, parent:{}, branch:{}, leng:{leng:.3}, sinks:{}", fmt_p(parent), fmt_p(branch), sinks.len()));
                }
                let blockage = if !self.is_occupied_loc(branch) { self.find_blockage(branch, f64::from(scaling)) } else { None };
                if let Some((x1, y1, x2, y2)) = blockage {
                    let locs = self.find_legal_locations(log, parent, branch, x1, y1, x2, y2);
                    let legal = self.find_best_legal_location(log, length, branch, parent, &locs, &sinks, x1, y1, x2, y2, scaling, dir);
                    log.debug("CTS", "legalizer", 1, || format!("findBestLegalLocation branchPt:{}=>{} parentPt:{} new branchPt is {} blockage", fmt_p(branch), fmt_p(legal), fmt_p(parent), if Self::is_inside_bbox(legal.x, legal.y, x1, y1, x2, y2) { "inside" } else { "outside" }));
                    self.commit_move_loc(branch, legal);
                    self.topology[level_idx].points[buf_idx] = legal;
                } else if self.is_occupied_loc(branch) || !fuzzy_equal(leng, length, 0.01) {
                    let new = self.adjust_beyond_blockage(log, branch, parent, length, &sinks, scaling, dir);
                    log.debug("CTS", "legalizer", 3, || format!("adjustBeyondBlockage applied to legal branchPt:{}=>{} parentPt:{} newDist={:.3}", fmt_p(branch), fmt_p(new), fmt_p(parent), self.compute_dist(new, parent)));
                    self.commit_move_loc(branch, new);
                    self.topology[level_idx].points[buf_idx] = new;
                } else {
                    self.commit_loc(branch);
                }
            }
        }
        self.legalize_dummy(log);
        if log.debug_on("CTS", "legalizer", 3) {
            log.report("HTree after legalization -------");
        }
    }

    /// `legalizeDummy`: a point driving no sinks is mirrored from its sibling and pulled in; one on
    /// its sibling or parent is pulled in; either is then moved off a blockage.
    fn legalize_dummy(&mut self, log: &mut Log) {
        let top = self.sink_region.center();
        for level_idx in 0..self.topology.len() {
            for idx in 0..self.topology[level_idx].points.len() {
                let parent_idx = self.topology[level_idx].parents[idx];
                let parent = if level_idx == 0 { top } else { self.topology[level_idx - 1].points[parent_idx as usize] };
                let sinks = self.topology[level_idx].sinks[idx].clone();
                let leng = self.topology[level_idx].length;
                let sib = Self::find_sibling(&self.topology[level_idx], idx, parent_idx);
                let mut branch = self.topology[level_idx].points[idx];
                let sib_loc = self.topology[level_idx].points[sib];
                let d1 = self.compute_dist(branch, sib_loc);
                let d2 = self.compute_dist(branch, parent);
                let overlap = d1 == 0.0 || d2 == 0.0;
                if sinks.is_empty() {
                    // `setSiblingPosition(sibLoc, branchPoint, parent)`.
                    branch = P::new(2.0 * parent.x - sib_loc.x, 2.0 * parent.y - sib_loc.y);
                    self.scale_position(&mut branch, parent, leng, 0.1);
                } else if overlap {
                    self.scale_position(&mut branch, parent, leng, 0.1);
                } else {
                    continue;
                }
                self.topology[level_idx].points[idx] = branch;
                let scaling = self.wire_segment_unit;
                if self.is_occupied_loc(branch) {
                    continue;
                }
                if let Some((x1, _y1, x2, y2)) = self.find_blockage(branch, f64::from(scaling)) {
                    let locs = self.find_legal_locations(log, parent, branch, x1, _y1, x2, y2);
                    // The reference passes (x1, y2, x2, y2) here.
                    let legal = self.find_best_legal_location(log, leng, branch, parent, &locs, &sinks, x1, y2, x2, y2, scaling, Dir::North);
                    let d = self.compute_dist(legal, parent);
                    log.debug("CTS", "legalizer", 1, || format!("legalizeDummy level index {level_idx}: {}->{} d={d:.3}, leng={leng:.3},ratio={:.3}", fmt_p(branch), fmt_p(legal), d / leng));
                    self.commit_move_loc(branch, legal);
                    self.topology[level_idx].points[idx] = legal;
                }
            }
        }
    }

    /// `createClockSubNets`: the root buffer at the (legalized) centre, each level's segments
    /// buffered from the parent point to each branching point that has sinks, then the sinks on
    /// the leaf subnets.
    fn create_clock_sub_nets(&mut self, log: &mut Log) -> Res<()> {
        let root = self.o.root_buffer.clone();
        let center = self.sink_region.center();
        let legal_center = self.legalize_one_buffer(center, &root);
        self.sink_region.set_center(legal_center);
        self.commit_move_loc(center, legal_center);
        let unit = f64::from(self.wire_segment_unit);
        let (cx, cy) = ((legal_center.x * unit) as i32, (legal_center.y * unit) as i32);
        let root_buffer = self.clock.add_clock_buffer("clkbuf_0", &root, cx, cy);
        if self.top_buffer_name.is_empty() {
            self.top_buffer_name = self.clock.buffers[root_buffer].name.clone();
        }
        if !center.same(legal_center) {
            log.debug("CTS", "legalizer", 2, || format!("createClockSubNets: root clkbuf_0: {} => {}", fmt_p(center), fmt_p(legal_center)));
        } else {
            log.debug("CTS", "legalizer", 2, || format!("createClockSubNets: root clkbuf_0: {}", fmt_p(center)));
        }
        self.tree_level_buffers.push(root_buffer);
        let root_sub_net = self.clock.add_sub_net("clknet_0");
        self.clock.subnets[root_sub_net].insts.push(InstRef::Buffer(root_buffer));
        self.tree_buf_levels += 1;

        let levels = self.topology.len();
        for level_idx in 0..levels {
            let mut is_first_point = true;
            for idx in 0..self.topology[level_idx].points.len() {
                if self.topology[level_idx].sinks[idx].is_empty() {
                    continue;
                }
                let branch = self.topology[level_idx].points[idx];
                let (from, driving) = if level_idx == 0 {
                    (legal_center, root_sub_net)
                } else {
                    let parent_idx = self.topology[level_idx].parents[idx] as usize;
                    let parent = &self.topology[level_idx - 1];
                    (parent.points[parent_idx], parent.driving[parent_idx].expect("a parent with sinks has a subnet"))
                };
                let legal = self.legalize_one_buffer(branch, &root);
                self.commit_move_loc(branch, legal);
                let prefix = format!("clkbuf_{}_{idx}_", level_idx + 1);
                if level_idx == 0 {
                    if !branch.same(legal) {
                        log.debug("CTS", "legalizer", 2, || format!("createClockSubNets level 1 clk_buf_1_{idx}_ : {} => {}", fmt_p(branch), fmt_p(legal)));
                    } else {
                        log.debug("CTS", "legalizer", 2, || format!("createClockSubNets level 1 clk_buf_1_{idx}_ : {}", fmt_p(branch)));
                    }
                } else if !branch.same(legal) {
                    log.debug("CTS", "legalizer", 2, || format!("createClockSubNets level {} {prefix} : {} => {}", level_idx + 1, fmt_p(branch), fmt_p(legal)));
                } else {
                    log.debug("CTS", "legalizer", 2, || format!("createClockSubNets level {} {prefix} : {}", level_idx + 1, fmt_p(branch)));
                }
                let wires = self.topology[level_idx].wire_segments.clone();
                let mut b = SegmentBuilder { inst_prefix: prefix, net_prefix: format!("clknet_{}_{idx}_", level_idx + 1), root: from, target: legal, wires, driving, num_buffer_levels: 0 };
                let force = self.o.tree_buffer.clone().unwrap_or_default();
                b.build(self, log, &force);
                if level_idx == levels - 1 {
                    b.force_buffer_in_segment(self, log, &root);
                }
                if is_first_point {
                    self.tree_buf_levels += b.num_buffer_levels;
                    is_first_point = false;
                }
                self.topology[level_idx].driving[idx] = Some(b.driving);
            }
        }

        let mut num_sinks = 0u32;
        let leaf = self.topology.last().expect("a level").clone();
        for idx in 0..leaf.points.len() {
            let Some(sub_net) = leaf.driving[idx] else { continue };
            self.clock.subnets[sub_net].leaf = true;
            for loc in &leaf.sinks[idx] {
                let Some(&sink) = self.map_location_to_sink.get(&loc.key()) else {
                    return log.error("CTS", 80, "Sink not found.");
                };
                self.clock.subnets[sub_net].insts.push(sink);
                num_sinks += 1;
            }
        }
        log.info("CTS", 35, format!(" Number of sinks covered: {num_sinks}."));
        Ok(())
    }

    /// `createSingleBufferClockNet`: one root buffer driving every sink.
    fn create_single_buffer_clock_net(&mut self, log: &mut Log) {
        log.report(" Building single-buffer clock net.");
        let root = self.o.root_buffer.clone();
        let center = self.sink_region.center();
        let legal_center = self.legalize_one_buffer(center, &root);
        self.sink_region.set_center(legal_center);
        self.commit_move_loc(center, legal_center);
        let unit = f64::from(self.wire_segment_unit);
        let b = self.clock.add_clock_buffer("clkbuf_0", &root, (legal_center.x * unit) as i32, (legal_center.y * unit) as i32);
        if self.top_buffer_name.is_empty() {
            self.top_buffer_name = self.clock.buffers[b].name.clone();
        }
        if !center.same(legal_center) {
            log.debug("CTS", "legalizer", 2, || format!("createSingleBufferClockNet legalizeOneBuffer clkbuf_0: {} => {}", fmt_p(center), fmt_p(legal_center)));
        }
        self.tree_level_buffers.push(b);
        let n = self.clock.add_sub_net("clknet_0");
        self.clock.subnets[n].insts.push(InstRef::Buffer(b));
        for i in 0..self.clock.sinks.len() {
            self.clock.subnets[n].insts.push(InstRef::Sink(i));
        }
    }

    /// The tree as built (buffers, subnets, levels): the probe the reference's trace is scored on.
    pub fn probe_tree(&self, log: &mut Log) {
        for b in &self.clock.buffers {
            log.trace(format!("VYGHTI|{}|{}|{}|{}", b.name, b.master, b.x, b.y));
        }
        for n in &self.clock.subnets {
            let sinks: Vec<&str> = n.insts[1..].iter().map(|&r| self.clock.inst(r).name.as_str()).collect();
            log.trace(format!("VYGHTN|{}|{}|{}|{}", n.name, u8::from(n.leaf), self.clock.inst(n.insts[0]).name, sinks.join(",")));
        }
        log.trace(format!("VYGHTE|{}|{}|{}", self.clock.name, self.tree_buf_levels, self.clock.max_level));
    }
}

/// What `HTreeBuilder::legalizeOneBuffer` reads: the tree legalizer's blockages and occupied
/// locations, then the sink map the collision jitter avoids.
struct LegalizerView<'a> {
    obstruction_aware: bool,
    length_unit: u32,
    wire_segment_unit: i32,
    blockages: &'a [(i32, i32, i32, i32)],
    master_size: &'a HashMap<String, (i32, i32)>,
    occupied: &'a HashSet<(u64, u64)>,
    map_location_to_sink: &'a HashMap<(u64, u64), InstRef>,
}

impl LegalizerView<'_> {
    fn find_blockage(&self, p: P, scaling: f64) -> Option<(f64, f64, f64, f64)> {
        find_blockage(self.blockages, p, scaling)
    }

    /// `TreeBuilder::legalizeOneBuffer`: a buffer on a blockage moves to the nearest free candidate
    /// around its edges.
    fn tree_legalize_one_buffer(&self, loc: P, buffer: &str) -> P {
        if self.obstruction_aware {
            let unit = f64::from(self.length_unit);
            if let Some((x1, y1, x2, y2)) = self.find_blockage(loc, unit) {
                let &(mw, mh) = self.master_size.get(buffer).expect("a buffer master in the database");
                let (bx, by) = (loc.x, loc.y);
                let w = f64::from(mw) / unit;
                let h = f64::from(mh) / unit;
                let new_x = x1 - w;
                let new_y = y1 - h;
                let candidates = [
                    P::new(new_x, by),
                    P::new(new_x, by + h),
                    P::new(new_x, by - h),
                    P::new(x2, by),
                    P::new(x2, by + h),
                    P::new(x2, by - h),
                    P::new(bx, new_y),
                    P::new(bx + w, new_y),
                    P::new(bx - w, new_y),
                    P::new(bx, y2),
                    P::new(bx + w, y2),
                    P::new(bx - w, y2),
                ];
                let mut min_dist = f64::MAX;
                let no_best = P::new(-1e6, -1e6);
                let mut best = no_best;
                for c in candidates {
                    if !self.occupied.contains(&c.key()) {
                        // `computeDist` (no insertion delay is modelled).
                        let dist = c.dist(loc) + 0.0 + 0.0;
                        if dist < min_dist {
                            min_dist = dist;
                            best = c;
                        }
                    }
                }
                if !best.same(no_best) {
                    return best;
                }
            }
        }
        loc
    }

    /// `resolveLocationCollision`: shift right by 1/unit steps until no sink sits at the point (as
    /// floats).
    fn resolve_location_collision(&self, legal: P) -> P {
        let mut resolved = legal;
        let mut jitter = 0u32;
        loop {
            let check = P::new(f64::from(resolved.x as f32), f64::from(resolved.y as f32));
            if !self.map_location_to_sink.contains_key(&check.key()) {
                break;
            }
            jitter += 1;
            let offset = f64::from(jitter) / f64::from(self.wire_segment_unit);
            resolved.x = legal.x + offset;
        }
        resolved
    }
}

/// `findBlockage`: the first blockage containing the point (strictly), scaled down.
fn find_blockage(blockages: &[(i32, i32, i32, i32)], p: P, scaling: f64) -> Option<(f64, f64, f64, f64)> {
    let (bx, by) = (p.x * scaling, p.y * scaling);
    for &(x1, y1, x2, y2) in blockages {
        let (x1, y1, x2, y2) = (f64::from(x1), f64::from(y1), f64::from(x2), f64::from(y2));
        if x1 < bx && bx < x2 && y1 < by && by < y2 {
            return Some((x1 / scaling, y1 / scaling, x2 / scaling, y2 / scaling));
        }
    }
    None
}

/// A tree's legalizer state kept after it is built (see [`HTree::legalizer`]).
#[derive(Debug, Clone, Default)]
pub struct Legalizer {
    obstruction_aware: bool,
    length_unit: u32,
    wire_segment_unit: i32,
    blockages: Vec<(i32, i32, i32, i32)>,
    master_size: HashMap<String, (i32, i32)>,
    occupied: HashSet<(u64, u64)>,
    map_location_to_sink: HashSet<(u64, u64)>,
}

impl Legalizer {
    /// `HTreeBuilder::legalizeOneBuffer` on the kept state (nothing committed).
    pub fn legalize_one_buffer(&self, loc: P, buffer: &str) -> P {
        let map: HashMap<(u64, u64), InstRef> = self.map_location_to_sink.iter().map(|k| (*k, InstRef::Sink(0))).collect();
        let v = LegalizerView { obstruction_aware: self.obstruction_aware, length_unit: self.length_unit, wire_segment_unit: self.wire_segment_unit, blockages: &self.blockages, master_size: &self.master_size, occupied: &self.occupied, map_location_to_sink: &map };
        let legal = v.tree_legalize_one_buffer(loc, buffer);
        v.resolve_location_collision(legal)
    }
}

/// `odb::Direction2D` as the legalizer uses it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    North,
    East,
    South,
    West,
}

impl Dir {
    fn name(self) -> &'static str {
        match self {
            Dir::North => "NORTH",
            Dir::East => "EAST",
            Dir::South => "SOUTH",
            Dir::West => "WEST",
        }
    }
}

/// `SegmentBuilder`: the buffers of one segment, placed along an L from the root (first along
/// x, then y) at each characterized wire segment's buffer locations.
struct SegmentBuilder {
    inst_prefix: String,
    net_prefix: String,
    root: P,
    target: P,
    wires: Vec<u32>,
    driving: usize,
    num_buffer_levels: u32,
}

impl SegmentBuilder {
    /// `build`.
    fn build(&mut self, t: &mut HTree, log: &mut Log, force_buffer: &str) {
        let length_x = (self.root.x - self.target.x).abs();
        let low_to_hi_x = self.root.x < self.target.x;
        let low_to_hi_y = self.root.y < self.target.y;
        let unit = f64::from(t.wire_segment_unit);
        let mut connection_length = 0.0f64;
        for &w in &self.wires {
            let seg: WireSegment = t.lut.segments[w as usize].clone();
            let seg_len = u32::from(seg.length);
            for (buffer, &loc) in seg.buffer_locations.iter().enumerate() {
                let location = loc * f64::from(seg_len);
                connection_length += location;
                let (x, y) = if connection_length < length_x {
                    (if low_to_hi_x { self.root.x + connection_length } else { self.root.x - connection_length }, self.root.y)
                } else {
                    (self.target.x, if low_to_hi_y { self.root.y + (connection_length - length_x) } else { self.root.y - (connection_length - length_x) })
                };
                let master = if force_buffer.is_empty() { seg.buffer_masters[buffer].clone() } else { force_buffer.to_string() };
                let at = P::new(x, y);
                let legal = t.legalize_one_buffer(at, &master);
                t.commit_move_loc(at, legal);
                let name = format!("{}{}", self.inst_prefix, self.num_buffer_levels);
                let b = t.clock.add_clock_buffer(&name, &master, (legal.x * unit) as i32, (legal.y * unit) as i32);
                if !at.same(legal) {
                    connection_length -= t.compute_dist(at, legal);
                    log.debug("CTS", "legalizer", 2, || format!(" SegmentBuilder::build {name} TCId:{w} bufId:{buffer} connLen:{connection_length:.1}: {} => {}", fmt_p(at), fmt_p(legal)));
                } else {
                    log.debug("CTS", "legalizer", 2, || format!(" SegmentBuilder::build {name} TCId:{w} bufId:{buffer} connLen:{connection_length:.1}: {}", fmt_p(at)));
                }
                t.tree_level_buffers.push(b);
                t.clock.subnets[self.driving].insts.push(InstRef::Buffer(b));
                self.driving = t.clock.add_sub_net(&format!("{}{}", self.net_prefix, self.num_buffer_levels));
                t.clock.subnets[self.driving].insts.push(InstRef::Buffer(b));
                self.num_buffer_levels += 1;
            }
            connection_length += f64::from(seg_len);
        }
    }

    /// `forceBufferInSegment`: a segment with no buffer gets one at its (unlegalized) target.
    fn force_buffer_in_segment(&mut self, t: &mut HTree, log: &mut Log, master: &str) {
        if self.num_buffer_levels != 0 {
            return;
        }
        let unit = f64::from(t.wire_segment_unit);
        let name = format!("{}_f", self.inst_prefix);
        let b = t.clock.add_clock_buffer(&name, master, (self.target.x * unit) as i32, (self.target.y * unit) as i32);
        t.tree_level_buffers.push(b);
        log.debug("CTS", "legalizer", 2, || format!("  forceBufferInSegment {name}: {}", fmt_p(self.target)));
        t.clock.subnets[self.driving].insts.push(InstRef::Buffer(b));
        self.driving = t.clock.add_sub_net(&format!("{}_leaf", self.net_prefix));
        t.clock.subnets[self.driving].insts.push(InstRef::Buffer(b));
        self.num_buffer_levels += 1;
    }
}

/// A cluster's geometric centre as `preSinkClustering` computes it: each coordinate added to a
/// FLOAT sum as a double (`float += double`: summed in double, rounded back to float each time),
/// then divided by the count as a float.
fn cluster_center(points: &[(f32, f32)]) -> (f32, f32) {
    let (mut x, mut y) = (0.0f32, 0.0f32);
    for &(px, py) in points {
        x = (f64::from(x) + f64::from(px)) as f32;
        y = (f64::from(y) + f64::from(py)) as f32;
    }
    let n = points.len() as f32;
    (x / n, y / n)
}

/// `computeBranchSinks`: a branching point's sinks, as floats.
fn compute_branch_sinks(t: &Level, idx: usize) -> Vec<(f32, f32)> {
    t.sinks[idx].iter().map(|p| (p.x as f32, p.y as f32)).collect()
}

/// `TechChar::forEachWireSegment(length, load, slew)`: the segments under one key, in the order
/// they were created (the arguments are `uint8_t`).
fn segments_at(lut: &Lut, length: u32, load: u32, slew: u32) -> Vec<usize> {
    let key = Lut::key(length as u8, load as u8, slew as u8);
    lut.key_to_segments.get(&key).cloned().unwrap_or_default()
}

/// `computeMinDelaySegment` (the wire-length-bounded overload): the fastest segment of this length
/// whose input matches within `tolerance`; a buffered one when the input slew is at the threshold;
/// the tolerance widens up to 10.
#[allow(clippy::too_many_arguments)]
fn compute_min_delay_segment(lut: &Lut, length: u32, input_slew: u32, input_cap: u32, slew_threshold: u32, wirelength_threshold: i32, tolerance: u32, output_slew: &mut u32, output_cap: &mut u32, curr_wl: &mut i32) -> u32 {
    let (mut min_key, mut min_delay, mut min_buf_key, mut min_buf_delay) = (u32::MAX, u32::MAX, u32::MAX, u32::MAX);
    // `wirelengthThreshold - getMinSegmentLength()` is unsigned arithmetic.
    let unbuffered_threshold = (wirelength_threshold as u32).wrapping_sub(lut.min_segment_length);
    for load in 1..=lut.max_capacitance as i32 {
        for out_slew in 1..=lut.max_slew as i32 {
            for idx in segments_at(lut, length, load as u32, out_slew as u32) {
                let seg = &lut.segments[idx];
                if (i32::from(seg.input_cap) - input_cap as i32).unsigned_abs() > tolerance || (i32::from(seg.input_slew) - input_slew as i32).unsigned_abs() > tolerance {
                    continue;
                }
                let buffered = !seg.buffer_locations.is_empty();
                if buffered && *curr_wl + seg.wl2_first_buffer > wirelength_threshold {
                    continue;
                }
                if !buffered && ((*curr_wl + seg.wl2_first_buffer) as u32) > unbuffered_threshold {
                    continue;
                }
                if seg.delay < min_delay {
                    min_delay = seg.delay;
                    min_key = idx as u32;
                }
                if buffered && seg.delay < min_buf_delay {
                    min_buf_delay = seg.delay;
                    min_buf_key = idx as u32;
                }
            }
        }
    }
    const MAX_TOLERANCE: u32 = 10;
    if input_slew >= slew_threshold {
        if min_buf_key < u32::MAX {
            let s = &lut.segments[min_buf_key as usize];
            *output_slew = u32::from(s.output_slew);
            *output_cap = u32::from(s.load);
            *curr_wl = s.last_wl;
            return min_buf_key;
        }
        if tolerance < MAX_TOLERANCE {
            return compute_min_delay_segment(lut, length, input_slew, input_cap, slew_threshold, wirelength_threshold, tolerance + 1, output_slew, output_cap, curr_wl);
        }
    }
    if min_key == u32::MAX {
        if tolerance >= MAX_TOLERANCE {
            return min_key;
        }
        return compute_min_delay_segment(lut, length, input_slew, input_cap, slew_threshold, wirelength_threshold, tolerance + 1, output_slew, output_cap, curr_wl);
    }
    let s = &lut.segments[min_key as usize];
    if !s.buffer_locations.is_empty() {
        *output_slew = u32::from(s.output_slew);
        *curr_wl = s.last_wl;
    } else {
        *output_slew = u32::from(s.output_slew).max(input_slew + 1);
        *curr_wl += s.last_wl;
    }
    *output_cap = u32::from(s.load);
    min_key
}

/// `computeMinDelaySegment` (the forced-buffer overload, simple segments): the fastest unbuffered
/// segment, or — when forced — the fastest single-buffer one whose buffer sits within 10 % of
/// `(length + expected) / length`, else the fastest single-buffer one.
///
/// ⛔ This overload never writes the output slew and load: the caller's zeros stand, so the next
/// wire starts from slew 0 and the minimum input cap.
fn compute_min_delay_segment_forced(lut: &Lut, length: u32, force_buffer: bool, expected_length: i32) -> u32 {
    let (mut min_key, mut min_delay, mut min_buf_key, mut min_buf_delay) = (u32::MAX, u32::MAX, u32::MAX, u32::MAX);
    let (mut fallback_key, mut fallback_delay) = (u32::MAX, u32::MAX);
    let normal = f64::from(length);
    let target = (normal + f64::from(expected_length)) / normal;
    for load in 1..=lut.max_capacitance as i32 {
        for out_slew in 1..=lut.max_slew as i32 {
            for idx in segments_at(lut, length, load as u32, out_slew as u32) {
                let seg = &lut.segments[idx];
                let buffered = !seg.buffer_locations.is_empty();
                if !buffered && seg.delay < min_delay {
                    min_delay = seg.delay;
                    min_key = idx as u32;
                }
                if buffered && seg.delay < min_buf_delay && seg.buffer_locations.len() == 1 {
                    let loc = seg.buffer_locations[0];
                    if loc > target * 0.9 && loc < target * 1.1 {
                        min_buf_delay = seg.delay;
                        min_buf_key = idx as u32;
                    }
                    if seg.delay < fallback_delay {
                        fallback_delay = seg.delay;
                        fallback_key = idx as u32;
                    }
                }
            }
        }
    }
    if force_buffer {
        if min_buf_key != u32::MAX {
            return min_buf_key;
        }
        if fallback_key != u32::MAX {
            return fallback_key;
        }
    }
    min_key
}

/// `TechChar::reportSegment` (a `tech char` 1 debug line).
fn report_segment(log: &mut Log, lut: &Lut, key: u32) {
    let s = &lut.segments[key as usize];
    log.debug("CTS", "tech char", 1, || {
        format!(
            "    Key: {key} inSlew: {} inCap: {} outSlew: {} load: {} length: {} wl2fistyBuf: {} lastWL: {} delay: {}",
            s.input_slew, s.input_cap, s.output_slew, s.load, s.length, s.wl2_first_buffer, s.last_wl, s.delay
        )
    });
    for (loc, master) in s.buffer_locations.iter().zip(&s.buffer_masters) {
        log.debug("CTS", "tech char", 1, || format!("      location: {} buffer: {master}", crate::log::fmt_f64(*loc)));
    }
}

/// `TechChar::createFakeEntries(length, fakeLength)`: every segment of `length` copied as one of
/// `fake_length`.
///
/// ⛔ The reference reads the source segment's buffers AFTER appending the copy to the same
/// vector; when that append reallocates, the source has been moved from and reads as unbuffered.
/// The vector grows by doubling from empty, so its capacity is the next power of two.
pub fn create_fake_entries(log: &mut Log, lut: &mut Lut, length: u32, fake_length: u32) {
    if length == fake_length {
        return;
    }
    if log.debug_on("CTS", "tech char", 1) {
        log.warn("CTS", 45, "Creating fake entries in the LUT.");
    }
    for load in 1..=lut.max_capacitance {
        for out_slew in 1..=lut.max_slew {
            for idx in segments_at(lut, length, load, out_slew) {
                let src = lut.segments[idx].clone();
                let size = lut.segments.len();
                let reallocates = size == 0 || size.is_power_of_two();
                let mut fake = WireSegment {
                    length: fake_length as u8,
                    load: load as u8,
                    output_slew: out_slew as u8,
                    power: src.power,
                    delay: src.delay,
                    input_cap: src.input_cap,
                    input_slew: src.input_slew,
                    buffer_locations: Vec::new(),
                    buffer_masters: Vec::new(),
                    wl2_first_buffer: src.wl2_first_buffer,
                    last_wl: src.last_wl,
                };
                if !reallocates {
                    fake.buffer_locations = src.buffer_locations.clone();
                    fake.buffer_masters = src.buffer_masters.clone();
                }
                let key = Lut::key(fake_length as u8, load as u8, out_slew as u8);
                lut.key_to_segments.entry(key).or_default().push(lut.segments.len());
                lut.segments.push(fake);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`preSinkClustering`): the centre is the float sum of the members divided by their
    /// count as a float (not a running mean).
    #[test]
    fn a_cluster_centre_is_the_float_mean() {
        assert_eq!(cluster_center(&[(1.0, 2.0), (2.0, 4.0)]), (1.5, 3.0));
        assert_eq!(cluster_center(&[(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)]).0, (0.1f32 + 0.2 + 0.3) / 3.0);
    }

    /// Rule (`operator<<` of a double on a default stream): six significant digits, `%g`.
    #[test]
    fn ostream_floats_are_six_significant_digits() {
        assert_eq!(fmt_ostream(0.266428571428), "0.266429");
        assert_eq!(fmt_ostream(1.6235714285), "1.62357");
        assert_eq!(fmt_ostream(1285.9523809), "1285.95");
        assert_eq!(fmt_ostream(1.5), "1.5");
        assert_eq!(fmt_ostream(0.0), "0");
        assert_eq!(fmt_ostream(1234567.0), "1.23457e+06");
        assert_eq!(fmt_ostream(0.00001234), "1.234e-05");
    }

    /// Rule (`Point::operator==`): within 1e-6 on each axis; map keys are exact.
    #[test]
    fn points_compare_fuzzily_but_key_exactly() {
        let a = P::new(1.0, 2.0);
        let b = P::new(1.0 + 1e-7, 2.0);
        assert!(a.same(b));
        assert_ne!(a.key(), b.key());
        assert_eq!(P::new(-0.0, 0.0).key(), P::new(0.0, 0.0).key());
    }

    /// Rule (`Clock::computeSinkRegion`): the 1 % outliers are dropped from each end, per axis.
    #[test]
    fn sink_region_drops_one_percent_per_side() {
        let mut c = Clock::new("clk");
        for i in 0..200 {
            c.sinks.push(ClockInst { name: format!("s{i}"), master: String::new(), x: i, y: 1000 - i, insertion_delay: 0.0, input_cap: 0.0 });
        }
        assert_eq!(c.compute_sink_region(), (2, 803, 197, 998));
        let mut c = Clock::new("a/b");
        c.sinks.push(ClockInst { name: "s".into(), master: String::new(), x: 5, y: 7, insertion_delay: 0.0, input_cap: 0.0 });
        assert_eq!(c.compute_sink_region(), (5, 7, 5, 7));
        assert_eq!(c.name, "a\\/b", "a hierarchical net name is escaped");
    }

    /// Rule (`SegmentBuilder::build`): buffers go along x from the root, then along y; a
    /// segment's buffer offsets ACCUMULATE with the lengths of the segments before it.
    #[test]
    fn segment_buffers_walk_an_l_from_the_root() {
        let seg = |locs: Vec<f64>| WireSegment { length: 4, load: 1, output_slew: 1, power: 0.0, delay: 1, input_cap: 1, input_slew: 1, buffer_masters: vec!["B".into(); locs.len()], buffer_locations: locs, wl2_first_buffer: 0, last_wl: 0 };
        let mut lut = Lut { segments: vec![seg(vec![0.5]), seg(vec![0.25])], length_unit: 10, ..Lut::default() };
        let o = test_options();
        let d = HDesign { core: (0, 0, 1000, 1000), blockages: Vec::new(), master_size: HashMap::new() };
        let mut t = HTree::new(&o, &d, &mut lut, Clock::new("clk"), TreeType::Regular);
        t.wire_segment_unit = 10;
        let net = t.clock.add_sub_net("n");
        let mut b = SegmentBuilder { inst_prefix: "b_".into(), net_prefix: "n_".into(), root: P::new(0.0, 0.0), target: P::new(3.0, -10.0), wires: vec![0, 1], driving: net, num_buffer_levels: 0 };
        let mut log = Log::default();
        b.build(&mut t, &mut log, "");
        // First buffer at 0.5·4 = 2 along x; the second at 2 + 4 + 0.25·4 = 7, past x's 3 by 4 in y.
        let at: Vec<(i32, i32)> = t.clock.buffers.iter().map(|b| (b.x, b.y)).collect();
        assert_eq!(at, [(20, 0), (30, -40)]);
        assert_eq!(t.clock.subnets.iter().map(|n| n.name.as_str()).collect::<Vec<_>>(), ["n_clk", "n_0_clk", "n_1_clk"]);
    }

    /// Rule (`createFakeEntries`): a copy appended where the vector reallocates loses its buffers.
    #[test]
    fn a_fake_entry_made_on_reallocation_reads_as_unbuffered() {
        let seg = WireSegment { length: 2, load: 1, output_slew: 1, power: 0.0, delay: 5, input_cap: 1, input_slew: 1, buffer_locations: vec![0.5], buffer_masters: vec!["B".into()], wl2_first_buffer: 1, last_wl: 1 };
        let mut lut = Lut { max_capacitance: 1, max_slew: 1, ..Lut::default() };
        for _ in 0..2 {
            lut.key_to_segments.entry(Lut::key(2, 1, 1)).or_default().push(lut.segments.len());
            lut.segments.push(seg.clone());
        }
        create_fake_entries(&mut Log::default(), &mut lut, 2, 1);
        // Size 2 (a power of two) reallocates; size 3 does not.
        assert!(lut.segments[2].buffer_locations.is_empty());
        assert_eq!(lut.segments[3].buffer_locations, [0.5]);
        assert_eq!(lut.key_to_segments[&Lut::key(1, 1, 1)], [2, 3]);
    }

    fn test_options() -> HOptions {
        HOptions {
            root_buffer: "B".into(),
            sink_buffer: "B".into(),
            tree_buffer: None,
            sink_clustering: false,
            max_diameter: 50.0,
            max_diameter_set: false,
            sink_clustering_size: 30,
            sink_clustering_size_set: false,
            macro_max_diameter: 50.0,
            macro_max_diameter_set: false,
            macro_sink_clustering_size: 4,
            macro_sink_clustering_size_set: true,
            num_static_layers: 0,
            clustering_power: 4,
            clustering_capacity: 0.6,
            max_fanout: 0,
            max_wl: 0,
            wire_segment_unit: 10,
            db_units: 1000,
            max_slew: 4,
            num_max_leaf_sinks: 15,
            clock_tree_max_depth: 100,
            fake_lut_entries: true,
            obstruction_aware: true,
            buffer_distance: 100_000,
            sink_buffer_input_cap: 0.0,
            cap_per_dbu: 0.0,
            sink_clustering_levels: 0,
            simple_segments: false,
            vertex_buffers: false,
            vertex_buffer_distance: 240_000,
        }
    }
}
