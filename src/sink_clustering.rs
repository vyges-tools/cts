// SPDX-License-Identifier: Apache-2.0
//! Sink clustering ahead of the H-tree: sinks are ordered along a space-filling curve and cut
//! into consecutive runs ("clusters") that respect a size, a diameter and a capacitance limit,
//! one candidate solution per starting offset; the cheapest candidate is kept, and its
//! single-sink clusters are repaired by merging them into, or pulling a sink from, a nearby
//! cluster.
//!
//! The functions below are in the reference's call order and carry its names (snake-cased):
//! [`SinkClustering::run`] is the sequencer — `normalize_points`, the per-sink insertion delays,
//! `compute_all_thetas`, `sort_points`, `find_best_matching` (which calls
//! `repair_clustering_solution` and `is_limit_exceeded`).
//!
//! Exactness notes, held at each site:
//! - Points and every distance and cost are `f64`; the per-sink caps, the internal (normalized)
//!   diameter limit and the cap per normalized unit are `f32`, widened where they meet an `f64`.
//! - `max`/`min` are the C++ library's: `max(a, b)` is `a < b ? b : a` and `min(a, b)` is
//!   `b < a ? b : a`. They are spelled out ([`cpp_max`], [`cpp_min`]) rather than taken from
//!   `f64::max`, which differs on NaN and on which zero it returns.
//! - The sort is over `(theta, index)` pairs compared lexicographically. The index makes every key
//!   distinct, so no two elements tie and every correct sort algorithm returns the same order —
//!   the reference's library sort (an unstable one) needs no transcription here.
//! - The reference is built with floating-point contraction OFF, so `a * b + c` is a rounded
//!   multiply and a rounded add, exactly as Rust evaluates it.

use vyges_stt::flute::lut::Lut;

/// The cap limit is this many sink-buffer input caps (the reference's constant is a `double`).
const MAX_CAP_FACTOR: f64 = 10.0;

/// The routing alpha a Steiner builder starts with when nothing set it; [`SinkClustering::get_wire_length`]
/// takes the builder's current alpha, which is this unless the user changed it.
pub const DEFAULT_ROUTING_ALPHA: f32 = 0.3;

/// `std::max(a, b)`: `b` only when `a < b` — so `a` wins ties and NaN comparisons.
#[inline]
fn cpp_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// `std::min(a, b)`: `b` only when `b < a` — so `a` wins ties and NaN comparisons.
#[inline]
fn cpp_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `Point<double>::computeDist`: Manhattan distance, each axis as the larger minus the smaller.
#[inline]
fn compute_dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = if a.0 > b.0 { a.0 - b.0 } else { b.0 - a.0 };
    let dy = if a.1 > b.1 { a.1 - b.1 } else { b.1 - a.1 };
    dx + dy
}

/// The sink clusterer. Build it, [`add_point`](Self::add_point) and [`add_cap`](Self::add_cap)
/// once per sink in the same order, then [`run`](Self::run).
#[derive(Clone, Debug)]
pub struct SinkClustering {
    /// The sink buffer's input cap; the cap limit is ten of these.
    sink_buffer_input_cap: f64,
    /// Wire capacitance per database unit, as the characterization computed it.
    cap_per_dbu: f64,
    /// Database units per micron.
    db_units: i32,
    points: Vec<(f64, f64)>,
    points_cap: Vec<f32>,
    /// Per-sink insertion delay, looked up once at the first run on the NORMALIZED points.
    points_ins_delay: Vec<f64>,
    theta_index_vector: Vec<(f64, u32)>,
    best_solution: Vec<Vec<u32>>,
    max_internal_diameter: f32,
    cap_per_unit: f32,
    use_max_diameter: bool,
    use_max_size: bool,
    use_max_cap_limit: bool,
    scale_factor: i32,
    first_run: bool,
    x_span: f64,
    y_span: f64,
    max_diameter: f64,
    max_size: f64,
    /// Persists across runs: a later run only replaces the solution when it is strictly cheaper.
    best_solution_cost: f64,
}

impl SinkClustering {
    /// The constructor. `max_diameter_set` and `cluster_size_set` are the caller's choice of the
    /// macro-tree or the register-tree flags (whether `-macro_clustering_max_diameter` /
    /// `-sink_clustering_max_diameter`, and the matching size option, were given); the cap limit
    /// is used unless BOTH are set.
    pub fn new(sink_buffer_input_cap: f64, cap_per_dbu: f64, db_units: i32, max_diameter_set: bool, cluster_size_set: bool) -> Self {
        SinkClustering {
            sink_buffer_input_cap,
            cap_per_dbu,
            db_units,
            points: Vec::new(),
            points_cap: Vec::new(),
            points_ins_delay: Vec::new(),
            theta_index_vector: Vec::new(),
            best_solution: Vec::new(),
            max_internal_diameter: 10.0,
            cap_per_unit: 0.0,
            use_max_diameter: max_diameter_set,
            use_max_size: cluster_size_set,
            use_max_cap_limit: !(cluster_size_set && max_diameter_set),
            scale_factor: 1,
            first_run: true,
            x_span: 0.0,
            y_span: 0.0,
            max_diameter: 0.0,
            max_size: 0.0,
            best_solution_cost: f64::MAX,
        }
    }

    /// `addPoint`. The caller's coordinates are `float`s widened to `double`.
    pub fn add_point(&mut self, x: f64, y: f64) {
        self.points.push((x, y));
    }

    /// `addCap`, stored as `float`.
    pub fn add_cap(&mut self, cap: f32) {
        self.points_cap.push(cap);
    }

    /// `getNumPoints`.
    pub fn num_points(&self) -> u32 {
        self.points.len() as u32
    }

    /// `getMaxDiameter`: the chosen solution's largest closed-cluster cost, in caller units.
    pub fn max_diameter(&self) -> f64 {
        self.max_diameter
    }

    /// `getMaxSize`: the chosen solution's largest closed cluster.
    pub fn max_size(&self) -> f64 {
        self.max_size
    }

    /// The sinks as the clusterer holds them: the caller's locations before the first run, the
    /// unit-square ones after it (what the reference's own debug plot and wirelength read).
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// `getScaleFactor`.
    pub fn scale_factor(&self) -> i32 {
        self.scale_factor
    }

    /// `sinkClusteringSolution`: per cluster, the sink indices (the order of `add_point`) in the
    /// order they joined. ⚠️ A cluster can be EMPTY (see `find_best_matching`); the caller skips
    /// those but still counts them when it numbers the clusters.
    pub fn solution(&self) -> &[Vec<u32>] {
        &self.best_solution
    }

    /// The span the internal diameter is scaled by: the smaller nonzero span of the sinks' box.
    /// The reference repeats this inline in `normalizePoints` and `findBestMatching`.
    fn span(&self) -> f64 {
        if self.x_span == 0.0 && self.y_span == 0.0 {
            1.0 // arbitrary
        } else if self.x_span == 0.0 {
            self.y_span
        } else if self.y_span == 0.0 {
            self.x_span
        } else {
            cpp_min(self.x_span, self.y_span)
        }
    }

    /// `normalizePoints`: on the first run, maps the sinks into the unit square (an axis with no
    /// spread maps to 0); every run, sets the internal diameter limit and the cap per unit.
    fn normalize_points(&mut self, max_diameter: f32) {
        if self.first_run {
            let mut x_max = f64::NEG_INFINITY;
            let mut x_min = f64::INFINITY;
            let mut y_max = f64::NEG_INFINITY;
            let mut y_min = f64::INFINITY;
            for p in &self.points {
                x_max = cpp_max(p.0, x_max);
                y_max = cpp_max(p.1, y_max);
                x_min = cpp_min(p.0, x_min);
                y_min = cpp_min(p.1, y_min);
            }
            let x_span = x_max - x_min;
            let y_span = y_max - y_min;
            self.x_span = x_span;
            self.y_span = y_span;
            for p in &mut self.points {
                // A span is "true" when nonzero, NaN included.
                let x_norm = if x_span != 0.0 { (p.0 - x_min) / x_span } else { 0.0 };
                let y_norm = if y_span != 0.0 { (p.1 - y_min) / y_span } else { 0.0 };
                *p = (x_norm, y_norm);
            }
        }
        let span = self.span();
        // float / double, rounded back to float.
        self.max_internal_diameter = (max_diameter as f64 / span) as f32;
        // (double * int) * double, rounded to float.
        self.cap_per_unit = (self.cap_per_dbu * self.scale_factor as f64 * span) as f32;
    }

    /// `computeAllThetas` (first run only): each sink's position along the curve.
    fn compute_all_thetas(&mut self) {
        if self.first_run {
            self.theta_index_vector.reserve(self.points.len());
            for idx in 0..self.points.len() as u32 {
                let p = self.points[idx as usize];
                let theta = Self::compute_theta(p.0, p.1);
                self.theta_index_vector.push((theta, idx));
            }
        }
    }

    /// `sortPoints` (first run only): by theta, then by index. Keys are distinct (see the module
    /// note), so the algorithm does not matter.
    fn sort_points(&mut self) {
        if self.first_run {
            self.theta_index_vector.sort_by(|a, b| {
                if a.0 < b.0 {
                    std::cmp::Ordering::Less
                } else if b.0 < a.0 {
                    std::cmp::Ordering::Greater
                } else {
                    a.1.cmp(&b.1)
                }
            });
        }
    }

    /// `isOne`: within one machine epsilon below 1 (or above it). The comparison is signed on
    /// purpose — anything above 1 counts as one — so it takes no absolute value.
    #[allow(clippy::float_equality_without_abs)]
    fn is_one(pos: f64) -> bool {
        (1.0 - pos) < f64::EPSILON
    }

    /// `isZero`; the reference defines it but nothing calls it.
    #[allow(dead_code)]
    fn is_zero(pos: f64) -> bool {
        pos < f64::EPSILON
    }

    /// `computeTheta`: the recursive position along the curve. Terminates because the fold
    /// `2|v - 1/2|` reaches 1 within a mantissa's worth of steps.
    fn compute_theta(x: f64, y: f64) -> f64 {
        if Self::is_one(x) && Self::is_one(y) {
            return 0.5;
        }
        // Truncating double -> unsigned conversions.
        let quad = Self::num_vertex(((2.0 * x) as u32).min(1), ((2.0 * y) as u32).min(1));
        let mut t = Self::compute_theta(2.0 * (x - 0.5).abs(), 2.0 * (y - 0.5).abs());
        if quad % 2 == 1 {
            t = 1.0 - t;
        }
        // modf's fractional part: exact, like `v - trunc(v)`.
        ((quad as f64 + t) / 4.0 + 7.0 / 8.0).fract()
    }

    /// `numVertex`: the quadrant number of a corner. The reference raises an error on anything
    /// else; the arguments are clamped to 0/1 so it cannot happen.
    fn num_vertex(x: u32, y: u32) -> u32 {
        match (x, y) {
            (0, 0) => 0,
            (0, 1) => 1,
            (1, 1) => 2,
            (1, 0) => 3,
            _ => 4,
        }
    }

    /// `run` with no sink carrying an insertion delay. See [`run_with_insertion_delay`](Self::run_with_insertion_delay).
    pub fn run(&mut self, group_size: u32, max_diameter: f32, scale_factor: i32) -> Option<(u32, f32)> {
        self.run_with_insertion_delay(group_size, max_diameter, scale_factor, &mut |_, _| 0.0)
    }

    /// `run`. Returns `(best size, best diameter)` — which are just `group_size` and
    /// `max_diameter` — when a solution was chosen, `None` when not (the reference then leaves the
    /// caller's outputs untouched).
    ///
    /// `ins_delay` is the H-tree's per-sink insertion-delay lookup. ⚠️ The reference calls it with
    /// the NORMALIZED (unit-square) coordinates, once per sink at the first run, while the H-tree
    /// stores the delays under the sinks' un-normalized locations; a lookup only hits when the two
    /// coincide. The closure receives exactly what the reference looks up.
    pub fn run_with_insertion_delay(&mut self, group_size: u32, max_diameter: f32, scale_factor: i32, ins_delay: &mut dyn FnMut(f64, f64) -> f64) -> Option<(u32, f32)> {
        self.scale_factor = scale_factor;
        self.normalize_points(max_diameter);
        if self.first_run {
            self.points_ins_delay = self.points.iter().map(|p| ins_delay(p.0, p.1)).collect();
        }
        self.compute_all_thetas();
        self.sort_points();
        let best_solution_found = self.find_best_matching(group_size);
        self.first_run = false;
        if best_solution_found {
            Some((group_size, max_diameter))
        } else {
            None
        }
    }

    /// The distance cost between two sinks: Manhattan distance plus both insertion delays, added
    /// in that order.
    #[inline]
    fn dist_cost(&self, idx_a: u32, a: (f64, f64), idx_b: u32, b: (f64, f64)) -> f64 {
        compute_dist(a, b) + self.points_ins_delay[idx_a as usize] + self.points_ins_delay[idx_b as usize]
    }

    /// `repairClusteringSolution`: each single-sink cluster, in index order, is merged into the
    /// cluster whose farthest member is nearest (strictly under the diameter limit and within the
    /// limits), or failing that takes the nearest sink of a cluster of more than two. A merge
    /// swaps the emptied cluster with the last and pops it, then re-examines the same index when
    /// the cluster swapped in is itself a single.
    #[allow(clippy::too_many_arguments)]
    fn repair_clustering_solution(
        &self,
        group_size: u32,
        solution_points: &mut Vec<Vec<(f64, f64)>>,
        solution_points_idx: &mut Vec<Vec<u32>>,
        solutions: &mut Vec<Vec<u32>>,
        single_cluster_count: &mut i32,
        solved_cluster_count: &mut i32,
    ) {
        let mut cluster_num = solutions.len() as i32;
        let mut c: i32 = 0;
        while c < cluster_num {
            let cu = c as usize;
            if solution_points[cu].len() == 1 {
                *single_cluster_count += 1;
                let idx = solution_points_idx[cu][0];
                let p = self.points[idx as usize];
                let mut min_dist_to_add = self.max_internal_diameter as f64;
                let mut min_sink_idx_to_add: i32 = -1;
                let mut min_cluster_idx_to_add: i32 = -1;
                let mut min_dist_to_merge = self.max_internal_diameter as f64;
                let mut cluster_idx_to_merge: i32 = -1;
                for k in 0..cluster_num {
                    if k == c {
                        continue;
                    }
                    let ku = k as usize;
                    let mut point_idx: u32 = 0;
                    let mut distance_cost = 0.0f64;
                    let mut cap_cost = self.points_cap[idx as usize] as f64;
                    // The reference keeps its own `unsigned` counter beside the range loop; so do we.
                    #[allow(clippy::explicit_counter_loop)]
                    for comparison_point in &solution_points[ku] {
                        let other = solution_points_idx[ku][point_idx as usize];
                        let cost = self.dist_cost(idx, p, other, *comparison_point);
                        let pair_cap_cost = self.points_cap[idx as usize] as f64
                            + (cost * self.cap_per_unit as f64 + self.points_cap[other as usize] as f64);
                        if solution_points[ku].len() > 2 && cost < min_dist_to_add && !self.is_limit_exceeded(2, cost, pair_cap_cost, group_size) {
                            min_dist_to_add = cost;
                            min_sink_idx_to_add = point_idx as i32;
                            min_cluster_idx_to_add = k;
                        }
                        if self.use_max_cap_limit {
                            cap_cost += cost * self.cap_per_unit as f64 + self.points_cap[other as usize] as f64;
                        }
                        point_idx += 1;
                        distance_cost = cpp_max(cost, distance_cost);
                    }
                    if !self.is_limit_exceeded(solution_points[ku].len() as u32, distance_cost, cap_cost, group_size) && distance_cost < min_dist_to_merge {
                        cluster_idx_to_merge = k;
                        min_dist_to_merge = distance_cost;
                    }
                }
                if cluster_idx_to_merge != -1 {
                    *solved_cluster_count += 1;
                    let m = cluster_idx_to_merge as usize;
                    solution_points[m].push(p);
                    solution_points_idx[m].push(idx);
                    solutions[m].push(idx);
                    let last = (cluster_num - 1) as usize;
                    solution_points.swap(cu, last);
                    solution_points.pop();
                    solution_points_idx.swap(cu, last);
                    solution_points_idx.pop();
                    solutions.swap(cu, last);
                    solutions.pop();
                    cluster_num -= 1;
                    if c < cluster_num && solution_points[cu].len() == 1 {
                        c -= 1;
                    }
                } else if min_cluster_idx_to_add != -1 {
                    *solved_cluster_count += 1;
                    let a = min_cluster_idx_to_add as usize;
                    let s = min_sink_idx_to_add as usize;
                    let idx_p = solution_points_idx[a][s];
                    solution_points[cu].push(self.points[idx_p as usize]);
                    solution_points_idx[cu].push(idx_p);
                    solutions[cu].push(idx_p);
                    let old_size = solution_points[a].len();
                    solution_points[a].swap(s, old_size - 1);
                    solution_points[a].pop();
                    solution_points_idx[a].swap(s, old_size - 1);
                    solution_points_idx[a].pop();
                    solutions[a].swap(s, old_size - 1);
                    solutions[a].pop();
                }
            }
            c += 1;
        }
    }

    /// `findBestMatching`: one candidate per starting offset `j` in `0..group_size`, each walking
    /// the sorted sinks from offset `j` (wrapping) and closing the current cluster when adding the
    /// next sink would break a limit. Closing a cluster adds its cost (its largest member distance,
    /// or the diameter limit when that was 0) — the LAST cluster of a candidate is never closed,
    /// so it adds nothing. ⚠️ The cheapest candidate is searched from `j = 1`: candidate 0 is built
    /// but can never be chosen, and `group_size <= 1` never yields a solution.
    fn find_best_matching(&mut self, group_size: u32) -> bool {
        let g = group_size as usize;
        let mut clusters = vec![0u32; g];
        let mut costs = vec![0.0f64; g];
        let mut previous_costs = vec![0.0f64; g];
        let mut max_diameter = vec![0.0f64; g];
        let mut max_size = vec![0u32; g];
        let mut solution_points: Vec<Vec<Vec<(f64, f64)>>> = vec![Vec::new(); g];
        let mut solution_points_idx: Vec<Vec<Vec<u32>>> = vec![Vec::new(); g];
        let mut solutions: Vec<Vec<Vec<u32>>> = vec![Vec::new(); g];

        let sinks_num = self.theta_index_vector.len() as u32;
        if sinks_num == 0 {
            return false;
        }
        for j in 0..group_size {
            let ju = j as usize;
            for i in 0..sinks_num {
                if solutions[ju].is_empty() {
                    solutions[ju].push(Vec::new());
                    solution_points[ju].push(Vec::new());
                    solution_points_idx[ju].push(Vec::new());
                }
                let idx = self.theta_index_vector[(i.wrapping_add(j) % sinks_num) as usize].1;
                let p = self.points[idx as usize];
                let mut distance_cost = 0.0f64;
                let mut cap_cost = self.points_cap[idx as usize] as f64;
                let cur = clusters[ju] as usize;
                for (point_idx, comparison_point) in solution_points[ju][cur].iter().enumerate() {
                    let other = solution_points_idx[ju][cur][point_idx];
                    let cost = self.dist_cost(idx, p, other, *comparison_point);
                    if self.use_max_cap_limit {
                        cap_cost += cost * self.cap_per_unit as f64 + self.points_cap[other as usize] as f64;
                    }
                    distance_cost = cpp_max(cost, distance_cost);
                }
                let cur_size = solution_points[ju][cur].len() as u32;
                if self.is_limit_exceeded(cur_size, distance_cost, cap_cost, group_size) {
                    max_diameter[ju] = cpp_max(max_diameter[ju], previous_costs[ju]);
                    max_size[ju] = max_size[ju].max(cur_size);
                    if previous_costs[ju] == 0.0 {
                        previous_costs[ju] = self.max_internal_diameter as f64;
                    }
                    costs[ju] += previous_costs[ju];
                    clusters[ju] += 1;
                    previous_costs[ju] = 0.0;
                } else {
                    previous_costs[ju] = cpp_max(distance_cost, previous_costs[ju]);
                }
                if solutions[ju].len() < (clusters[ju] + 1) as usize {
                    solutions[ju].push(Vec::new());
                    solution_points[ju].push(Vec::new());
                    solution_points_idx[ju].push(Vec::new());
                }
                let cur = clusters[ju] as usize;
                solution_points[ju][cur].push(p);
                solution_points_idx[ju][cur].push(idx);
                solutions[ju][cur].push(idx);
            }
        }

        let mut best_solution = 0usize;
        let mut best_solution_found = false;
        #[allow(clippy::needless_range_loop)]
        for j in 1..g {
            if costs[j] < self.best_solution_cost {
                best_solution = j;
                self.best_solution_cost = costs[j];
                best_solution_found = true;
            }
        }
        if best_solution_found {
            let mut single_cluster_count = 0;
            let mut solved_cluster_count = 0;
            let mut sp = std::mem::take(&mut solution_points[best_solution]);
            let mut spi = std::mem::take(&mut solution_points_idx[best_solution]);
            let mut sol = std::mem::take(&mut solutions[best_solution]);
            self.repair_clustering_solution(group_size, &mut sp, &mut spi, &mut sol, &mut single_cluster_count, &mut solved_cluster_count);
            self.best_solution = sol;
            let span = self.span();
            self.max_diameter = max_diameter[best_solution] * span;
            self.max_size = max_size[best_solution] as f64;
        }
        best_solution_found
    }

    /// `isLimitExceeded`: the cap limit (unless both size and diameter were given), the size limit
    /// (`>=`: a cluster already AT the size takes no more) and the diameter limit (strict `>`).
    fn is_limit_exceeded(&self, size: u32, cost: f64, cap_cost: f64, size_limit: u32) -> bool {
        (self.use_max_cap_limit && (cap_cost > self.sink_buffer_input_cap * MAX_CAP_FACTOR))
            || (self.use_max_size && (size >= size_limit))
            || (self.use_max_diameter && (cost > self.max_internal_diameter as f64))
    }

    /// `getWireLength`: a Steiner tree over the points scaled to database units (each coordinate
    /// truncated to an integer) with their centroid as the driver, its length back in caller
    /// units. `alpha` is the Steiner builder's current routing alpha ([`DEFAULT_ROUTING_ALPHA`]
    /// unless the user changed it); no per-net alpha applies, since no net is given. The reference
    /// only uses this for a debug report; `None` for no points or when no tree comes back.
    pub fn get_wire_length(&self, lut: &Lut, points: &[(f64, f64)], alpha: f32) -> Option<f64> {
        if points.is_empty() {
            return None;
        }
        let mut driver_x = 0.0f64;
        let mut driver_y = 0.0f64;
        for p in points {
            driver_x += p.0;
            driver_y += p.1;
        }
        driver_x /= points.len() as f64;
        driver_y /= points.len() as f64;
        let mut vec_x: Vec<i32> = vec![(driver_x * self.db_units as f64) as i32];
        let mut vec_y: Vec<i32> = vec![(driver_y * self.db_units as f64) as i32];
        for p in points {
            vec_x.push((p.0 * self.db_units as f64) as i32);
            vec_y.push((p.1 * self.db_units as f64) as i32);
        }
        let tree = vyges_stt::make_steiner_tree(lut, &vec_x, &vec_y, 0, alpha).0?;
        let wl = tree.length as i32;
        Some(wl as f64 / self.db_units as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Every expected value below was printed by a driver built from the reference's own source,
    // with the reference binary's compiler, standard library and flags.

    fn build(points: &[(f32, f32)], cap: f32, sbcap: f64, cap_per_dbu: f64, dia_set: bool, size_set: bool) -> SinkClustering {
        let mut sc = SinkClustering::new(sbcap, cap_per_dbu, 1000, dia_set, size_set);
        for &(x, y) in points {
            sc.add_point(x as f64, y as f64);
            sc.add_cap(cap);
        }
        sc
    }

    const FIVE: [(f32, f32); 5] = [(0.0, 0.0), (10.0, 10.0), (20.0, 0.0), (5.0, 5.0), (30.0, 30.0)];

    #[test]
    fn candidate_zero_is_never_chosen_and_the_last_cluster_is_never_closed() {
        // Rule: the cheapest candidate is searched from offset 1, so a group size of 1 builds one
        // candidate and chooses none. And only a cluster that is CLOSED (the next sink broke a
        // limit) contributes to the max size / max diameter, so a solution that is one open
        // cluster reports 0 for both.
        let mut one = build(&FIVE, 1e-15, 1e-15, 1e-19, false, false);
        assert_eq!(one.run(1, 100.0, 1), None);
        assert!(one.solution().is_empty());

        let mut four = build(&FIVE, 1e-15, 1e-15, 1e-19, false, false);
        assert_eq!(four.run(4, 100.0, 1), Some((4, 100.0)));
        // Curve order of the five sinks, all in one cluster.
        assert_eq!(four.solution(), &[vec![4, 2, 1, 3, 0]]);
        assert_eq!((four.max_size(), four.max_diameter()), (0.0, 0.0));

        // The wirelength: centroid driver, coordinates in database units, length back in units.
        let lut = vyges_stt::flute::lut::load_tables(vyges_stt::flute::lut::MAX_LUT_DEGREE).expect("flute tables");
        let raw: Vec<(f64, f64)> = FIVE.iter().map(|&(x, y)| (x as f64, y as f64)).collect();
        assert_eq!(four.get_wire_length(&lut, &raw, DEFAULT_ROUTING_ALPHA).map(f64::to_bits), Some(0x4051800000000000));
        let norm: Vec<(f64, f64)> = [4u32, 2, 1, 3, 0].iter().map(|&i| four.points()[i as usize]).collect();
        assert_eq!(four.get_wire_length(&lut, &norm, DEFAULT_ROUTING_ALPHA).map(f64::to_bits), Some(0x4002a9fbe76c8b44));
    }

    #[test]
    fn coincident_sinks_keep_index_order_and_a_single_takes_a_sink_from_a_larger_cluster() {
        // Rule: sinks on one location share a theta and are ordered by index. The size limit is
        // `>=`, so clusters close at 3; the lone sink at the origin cannot join a full cluster, so
        // it takes the nearest sink of a cluster of MORE than two, which is swapped with that
        // cluster's last member and popped.
        let mut pts = vec![(3.0f32, 3.0f32); 6];
        pts.push((0.0, 0.0));
        let mut sc = build(&pts, 1e-15, 1e-15, 0.0, false, true);
        assert_eq!(sc.run(3, 100.0, 1), Some((3, 100.0)));
        assert_eq!(sc.solution(), &[vec![3, 2], vec![4, 5, 6], vec![0, 1]]);
        assert_eq!((sc.max_size(), sc.max_diameter()), (3.0, 6.0));
    }

    #[test]
    fn a_single_merges_into_the_cluster_whose_farthest_member_is_nearest() {
        // Rule: a single-sink cluster joins the cluster whose LARGEST distance to it is the
        // smallest (strictly below the diameter limit, within the limits); the emptied cluster is
        // swapped with the last and popped, so cluster order changes.
        let pts = [(0.0f32, 0.0f32), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (5.0, 5.0), (6.0, 5.0), (10.0, 10.0)];
        let mut sc = build(&pts, 1e-15, 1e-15, 0.0, true, true);
        assert_eq!(sc.run(3, 4.0, 1), Some((3, 4.0)));
        assert_eq!(sc.solution(), &[vec![2, 3], vec![1, 0], vec![6], vec![5, 4]]);
        assert_eq!((sc.max_size(), sc.max_diameter()), (2.0, 1.0));
    }

    #[test]
    fn insertion_delays_are_looked_up_at_the_normalized_location() {
        // Rule: the per-sink insertion delay is fetched once, at the first run, with the sink's
        // UNIT-SQUARE coordinates, and adds to every distance that sink takes part in. Sink (1, 1)
        // of a 0..8 box sits at (0.125, 0.125); a delay there splits it off.
        let pts = [(0.0f32, 0.0f32), (8.0, 8.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)];
        let mut plain = build(&pts, 1e-15, 1e-15, 0.0, true, true);
        assert_eq!(plain.run(3, 3.0, 1), Some((3, 3.0)));
        assert_eq!(plain.solution(), &[vec![1], vec![0, 4], vec![3, 2]]);
        assert_eq!((plain.max_size(), plain.max_diameter()), (3.0, 2.0));

        let mut delayed = build(&pts, 1e-15, 1e-15, 0.0, true, true);
        let mut lookup = |x: f64, y: f64| if (x, y) == (0.125, 0.125) { 2.0 } else { 0.0 };
        assert_eq!(delayed.run_with_insertion_delay(3, 3.0, 1, &mut lookup), Some((3, 3.0)));
        assert_eq!(delayed.solution(), &[vec![1], vec![2], vec![4, 0, 3]]);
        assert_eq!((delayed.max_size(), delayed.max_diameter()), (1.0, 0.0));
    }
}
