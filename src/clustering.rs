// SPDX-License-Identifier: Apache-2.0
//! Capacitated k-means over clock sinks, as the H-tree builder uses it to refine a pair of
//! branching points.
//!
//! Each round pins the means to their segment lengths around the branching point, assigns sinks
//! to means by a min-cost flow (sink -> mean arcs cost `dist^power`, each mean takes at most `cap`
//! sinks, the first `sinks % means` take one more), sends any sink the flow left unassigned to the
//! nearest mean that still has room, and moves each mean to its cluster's centroid. Rounds stop
//! when the means move by less than half a unit in total or after `max + 1` rounds. The result is
//! scored by a silhouette, and the segment lengths are fixed once more at the end.
//!
//! The functions below are in the reference's call order and carry its names (snake-cased).
//!
//! Exactness notes, held at each site:
//! - Coordinates, distances, centroids and the silhouette are `f32`; the flow costs are `f64`
//!   (`pow` of a widened `f32` distance) truncated to `i32` when stored on the arcs.
//! - The flow is solved by [`crate::lemon::NetworkSimplex`], a transcription of the same solver,
//!   because ties between equal-cost assignments are common and the solver's own arc order and
//!   pivoting decide which optimum comes back. Its result is read even when it reports the
//!   problem infeasible (arcs dropped by the distance cut-off, or more supply than sinks).
//! - The constructor of the reference also reseeds the C library's random generator, but nothing
//!   in this algorithm draws from it, so there is nothing to port.

use crate::lemon::{ListDigraph, NetworkSimplex};

/// One sink: its location, the mean it is assigned to (`-1` for none), its input index.
#[derive(Clone, Debug)]
struct Sink {
    x: f32,
    y: f32,
    cluster_idx: i32,
    sink_idx: u32,
}

/// The flow round's outcome, kept only when [`Clustering::set_trace_flows`] asks for it.
#[doc(hidden)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FlowTrace {
    /// Whether the solver reported an optimal flow.
    pub optimal: bool,
    /// The flow on every arc, by arc id (creation order).
    pub flows: Vec<i32>,
}

/// The clustering engine: the sinks, the last round's clusters, and the branching point the
/// means are held around.
#[derive(Clone, Debug)]
pub struct Clustering {
    sinks: Vec<Sink>,
    /// Per mean, indices into `sinks` in sink order.
    clusters: Vec<Vec<usize>>,
    segment_length: f32,
    branching_point: Option<(f32, f32)>,
    trace_flows: bool,
    flow_trace: Vec<FlowTrace>,
}

impl Clustering {
    /// The engine without a branching point: the segment lengths are never fixed.
    pub fn new_unrooted(sinks: &[(f32, f32)]) -> Clustering {
        Clustering {
            sinks: sinks
                .iter()
                .enumerate()
                .map(|(i, &(x, y))| Sink { x, y, cluster_idx: -1, sink_idx: i as u32 })
                .collect(),
            clusters: Vec::new(),
            segment_length: 0.0,
            branching_point: None,
            trace_flows: false,
            flow_trace: Vec::new(),
        }
    }

    /// The engine the H-tree builder makes: sinks plus the branching point (the root location).
    pub fn new(sinks: &[(f32, f32)], x_branch: f32, y_branch: f32) -> Clustering {
        let mut c = Clustering::new_unrooted(sinks);
        c.branching_point = Some((x_branch, y_branch));
        c
    }

    /// Record every flow round's arc flows (a verification aid).
    #[doc(hidden)]
    pub fn set_trace_flows(&mut self, on: bool) {
        self.trace_flows = on;
    }

    /// The recorded flow rounds, in call order.
    #[doc(hidden)]
    pub fn flow_trace(&self) -> &[FlowTrace] {
        &self.flow_trace
    }

    /// Run `iter` k-means passes from `means` (each pass continues from the previous pass's
    /// means), keep the best by silhouette, then fix the segment lengths. `means` must hold at
    /// least two points; `n` is the number of clusters (the number of means).
    pub fn iter_kmeans(
        &mut self,
        iter: u32,
        n: u32,
        cap: u32,
        max: u32,
        power: u32,
        means: &mut Vec<(f32, f32)>,
    ) {
        // The segment length is the Manhattan distance between the two middle means.
        let mid = (means.len() / 2) as u32 as usize - 1;
        self.segment_length = (means[mid].0 - means[mid + 1].0).abs()
            + (means[mid].1 - means[mid + 1].1).abs();

        // A pass is kept only when its silhouette beats -1 strictly; otherwise the assignment
        // written back is the zero-filled one and `means` stays as given.
        let mut solution = vec![0i32; self.sinks.len()];
        let mut max_silh: f32 = -1.0;
        let mut tmp_means = means.clone();
        for _ in 0..iter {
            let silh = self.kmeans(n, cap, max, power, &mut tmp_means);
            if silh > max_silh {
                max_silh = silh;
                for (s, sink) in solution.iter_mut().zip(&self.sinks) {
                    *s = sink.cluster_idx;
                }
                *means = tmp_means.clone();
            }
        }
        for (sink, &s) in self.sinks.iter_mut().zip(&solution) {
            sink.cluster_idx = s;
        }
        self.fix_segment_lengths(means);
    }

    /// The middle two means are put at half a segment from the branching point, then each mean
    /// outward at one segment from its inner neighbour.
    fn fix_segment_lengths(&self, means: &mut [(f32, f32)]) {
        let Some(bp) = self.branching_point else {
            return;
        };
        let mid = (means.len() / 2) as u32 as usize - 1;
        // Half the segment is taken in double and handed over as a float.
        let half = (self.segment_length as f64 / 2.0) as f32;
        fix_segment(bp, half, &mut means[mid]);
        fix_segment(bp, half, &mut means[mid + 1]);
        let mut i = mid;
        while i > 0 {
            let fixed = means[i];
            fix_segment(fixed, self.segment_length, &mut means[i - 1]);
            i -= 1;
        }
        let mut i = mid + 1;
        while i < means.len() - 1 {
            let fixed = means[i];
            fix_segment(fixed, self.segment_length, &mut means[i + 1]);
            i += 1;
        }
    }

    /// One k-means pass; returns its silhouette.
    fn kmeans(&mut self, n: u32, cap: u32, max: u32, power: u32, means: &mut [(f32, f32)]) -> f32 {
        for sink in &mut self.sinks {
            sink.cluster_idx = -1;
        }
        let mut iter: u32 = 1;
        loop {
            self.fix_segment_lengths(means);
            self.min_cost_flow(means, cap, 5200.0, power);

            // Collect the clusters in sink order; a sink the flow left unassigned goes to the
            // nearest mean (first on ties) whose cluster is still under `cap`, else to mean 0.
            let mut clusters: Vec<Vec<usize>> = vec![Vec::new(); n as usize];
            for s in 0..self.sinks.len() {
                let sink = &self.sinks[s];
                let position = if sink.cluster_idx >= 0 && (sink.cluster_idx as u32) < n {
                    sink.cluster_idx as usize
                } else {
                    let mut minimum_dist = 0.0f32;
                    let mut minimum_idx: i64 = -1;
                    for (j, m) in means.iter().enumerate() {
                        if clusters[j].len() < cap as usize {
                            minimum_dist = calc_dist(*m, (sink.x, sink.y));
                            minimum_idx = j as i64;
                            break;
                        }
                    }
                    if minimum_idx == -1 {
                        minimum_idx = 0;
                    } else {
                        for (j, m) in means.iter().enumerate() {
                            if clusters[j].len() < cap as usize {
                                let d = calc_dist(*m, (sink.x, sink.y));
                                if d < minimum_dist {
                                    minimum_dist = d;
                                    minimum_idx = j as i64;
                                }
                            }
                        }
                    }
                    minimum_idx as usize
                };
                clusters[position].push(s);
            }

            // Move each non-empty cluster's mean to its centroid (float sums in sink order).
            let mut delta: f32 = 0.0;
            for i in 0..n as usize {
                let (mut sum_x, mut sum_y) = (0.0f32, 0.0f32);
                for &s in &clusters[i] {
                    sum_x += self.sinks[s].x;
                    sum_y += self.sinks[s].y;
                }
                let (pre_x, pre_y) = means[i];
                if !clusters[i].is_empty() {
                    let len = clusters[i].len() as f32;
                    means[i] = (sum_x / len, sum_y / len);
                    delta += (pre_x - means[i].0).abs() + (pre_y - means[i].1).abs();
                }
            }
            self.clusters = clusters;

            let stop = iter > max || delta < 0.5;
            iter = iter.wrapping_add(1);
            if stop {
                break;
            }
        }
        self.calc_silh(means)
    }

    /// The mean silhouette over sinks: (nearest other mean - own mean) / the larger of the two.
    fn calc_silh(&self, means: &[(f32, f32)]) -> f32 {
        let mut sum_silh: f32 = 0.0;
        for sink in &self.sinks {
            let (mut in_d, mut out_d) = (0.0f32, f32::MAX);
            for (j, &m) in means.iter().enumerate() {
                let d = calc_dist(m, (sink.x, sink.y));
                // A negative index widens to a huge unsigned value and matches no mean.
                if sink.cluster_idx as i64 as u64 == j as u64 {
                    in_d = d;
                } else {
                    // `min(d, out_d)` keeps `d` unless `out_d` is strictly smaller.
                    out_d = if out_d < d { out_d } else { d };
                }
            }
            // `max(out_d, in_d)` keeps `out_d` unless it is strictly smaller.
            let temp = if out_d < in_d { in_d } else { out_d };
            if temp == 0.0 {
                if out_d == 0.0 {
                    sum_silh += -1.0;
                }
                if in_d == 0.0 {
                    sum_silh += 1.0;
                }
            } else {
                sum_silh += (out_d - in_d) / temp;
            }
        }
        sum_silh / self.sinks.len() as f32
    }

    /// Assign sinks to means by min-cost flow: source -> each sink (cap 1, cost 0), sink -> mean
    /// when the distance is within `dist` and `dist^power` fits an `int` (cap 1, cost truncated),
    /// mean -> target (cap `cap`, one more for the first `sinks % means`), supply
    /// `means * cap + remainder`. Arcs are created in exactly that order.
    fn min_cost_flow(&mut self, means: &[(f32, f32)], cap: u32, dist: f32, power: u32) {
        let mut graph = ListDigraph::new();
        let src = graph.add_node();
        let target = graph.add_node();
        let sink_nodes: Vec<usize> = (0..self.sinks.len()).map(|_| graph.add_node()).collect();
        let cluster_nodes: Vec<usize> = (0..means.len()).map(|_| graph.add_node()).collect();

        let src_sink_edges: Vec<usize> =
            sink_nodes.iter().map(|&s| graph.add_arc(src, s)).collect();
        // (arc, sink, mean, cost)
        let mut sink_cluster_edges: Vec<(usize, usize, usize, f64)> = Vec::new();
        for (i, sink) in self.sinks.iter().enumerate() {
            for (j, &m) in means.iter().enumerate() {
                let mut d = calc_dist(m, (sink.x, sink.y)) as f64;
                if d <= dist as f64 {
                    // The double `pow` (the platform libm's), as the reference calls it.
                    d = d.powf(power as f64);
                    if d < i32::MAX as f64 {
                        let e = graph.add_arc(sink_nodes[i], cluster_nodes[j]);
                        sink_cluster_edges.push((e, i, j, d));
                    }
                }
            }
        }
        let cluster_sink_edges: Vec<usize> =
            cluster_nodes.iter().map(|&c| graph.add_arc(c, target)).collect();

        let remaining = (self.sinks.len() % means.len()) as i32;
        let mut flow = NetworkSimplex::new(&graph);
        for &e in &src_sink_edges {
            flow.set_arc(e, 0, 1, 0);
        }
        for &(e, _, _, cost) in &sink_cluster_edges {
            flow.set_arc(e, 0, 1, cost as i32);
        }
        for (i, &e) in cluster_sink_edges.iter().enumerate() {
            // Unsigned arithmetic stored into an `int` map.
            let upper = if (i as u64) < remaining as u64 { cap.wrapping_add(1) } else { cap };
            flow.set_arc(e, 0, upper as i32, 0);
        }
        let supply = (means.len() as u64)
            .wrapping_mul(cap as u64)
            .wrapping_add(remaining as i64 as u64) as i32;
        flow.st_supply(src, target, supply);
        let optimal = flow.run();

        if self.trace_flows {
            let n_arcs = src_sink_edges.len() + sink_cluster_edges.len() + cluster_sink_edges.len();
            self.flow_trace.push(FlowTrace { optimal, flows: (0..n_arcs).map(|a| flow.flow(a)).collect() });
        }

        // A sink's inflow is at most 1 (its one source arc), so at most one of its mean arcs
        // carries flow, even in an infeasible end state; the read order cannot matter.
        for &(e, i, j, _) in &sink_cluster_edges {
            if flow.flow(e) != 0 {
                self.sinks[i].cluster_idx = j as i32;
            }
        }
    }

    /// The last round's clusters, as input sink indices in sink order.
    pub fn get_clusters(&self) -> Vec<Vec<u32>> {
        self.clusters
            .iter()
            .map(|c| c.iter().map(|&s| self.sinks[s].sink_idx).collect())
            .collect()
    }
}

/// Move `movable` along the line from `fixed` so it lies `target_dist` (Manhattan) away; a point
/// on top of `fixed` stays put.
fn fix_segment(fixed: (f32, f32), target_dist: f32, movable: &mut (f32, f32)) {
    let actual = calc_dist(fixed, *movable);
    if actual == 0.0 {
        return;
    }
    let ratio = target_dist / actual;
    let dx = (movable.0 - fixed.0) * ratio;
    let dy = (movable.1 - fixed.1) * ratio;
    movable.0 = fixed.0 + dx;
    movable.1 = fixed.1 + dy;
}

/// Manhattan distance in `f32`.
fn calc_dist(a: (f32, f32), b: (f32, f32)) -> f32 {
    (a.0 - b.0).abs() + (a.1 - b.1).abs()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run the H-tree caller pattern (one pass, two means, max 5) on bit-given inputs.
    fn run(root: [u32; 2], means: [u32; 4], cap: u32, power: u32, sinks: &[u32]) -> Run {
        let f = f32::from_bits;
        let pts: Vec<(f32, f32)> = sinks.chunks(2).map(|p| (f(p[0]), f(p[1]))).collect();
        let mut eng = Clustering::new(&pts, f(root[0]), f(root[1]));
        eng.set_trace_flows(true);
        let mut m = vec![(f(means[0]), f(means[1])), (f(means[2]), f(means[3]))];
        eng.iter_kmeans(1, 2, cap, 5, power, &mut m);
        Run {
            means: m.iter().flat_map(|p| [p.0.to_bits(), p.1.to_bits()]).collect(),
            clusters: eng.get_clusters(),
            optimal: eng.flow_trace().iter().map(|t| t.optimal).collect(),
            last_flows: eng.flow_trace().last().unwrap().flows.clone(),
        }
    }

    struct Run {
        means: Vec<u32>,
        clusters: Vec<Vec<u32>>,
        optimal: Vec<bool>,
        last_flows: Vec<i32>,
    }

    // Every expected value below came from the reference implementation built on the test box
    // (clang, the pinned solver headers) and run on the same bit-exact inputs; the port matched it
    // bit for bit on 1,320 generated cases.

    /// Integer grid around the root: equal-cost assignments are common, and the solver's own
    /// pivoting picks which sink goes where.
    #[test]
    fn grid_ties_follow_the_solver() {
        let r = run(
            [0x41c80000, 0x40000000],
            [0x41c80000, 0x00000000, 0x41c80000, 0x40800000],
            1,
            4,
            &[0x41e00000, 0x41200000, 0x41f00000, 0x41000000, 0x41880000, 0xc0e00000],
        );
        assert_eq!(r.means, vec![0x41c00000, 0x3f800000, 0x41cc5d17, 0x405d1746]);
        assert_eq!(r.clusters, vec![vec![1, 2], vec![0]]);
        assert_eq!(r.optimal, vec![true; 6]);
        assert_eq!(r.last_flows, vec![1, 1, 1, 0, 1, 1, 0, 1, 0, 2, 1]);
    }

    /// Capacity above half the sinks: the supply exceeds the sink count, the solver reports the
    /// problem infeasible, and its flows are used anyway.
    #[test]
    fn supply_above_sink_count_is_read_infeasible() {
        let r = run(
            [0x43a26db7, 0x439d1933],
            [0x43a26db7, 0x439c7929, 0x43a26db7, 0x439db93d],
            4,
            4,
            &[
                0x43a1705c, 0x439eaea2, 0x43a2c369, 0x439c7cfe, 0x43a4ca22, 0x439afa07, 0x43a2ddf2,
                0x439ecff6,
            ],
        );
        assert_eq!(r.means, vec![0x43a2bd33, 0x439cc8a5, 0x43a256cb, 0x439da251]);
        assert_eq!(r.clusters, vec![vec![1, 2], vec![0, 3]]);
        assert_eq!(r.optimal, vec![false; 6]);
        assert_eq!(r.last_flows, vec![1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 2, 2]);
    }

    /// Sinks thousands of units out: `dist^4` overflows an `int`, so the first round has no
    /// sink-to-mean arcs and every sink takes the nearest-mean-with-room fallback.
    #[test]
    fn far_sinks_fall_back_to_nearest_mean() {
        let r = run(
            [0x43af400a, 0x43764b1b],
            [0x43af400a, 0xc50adcb1, 0x43af400a, 0x4529a615],
            4,
            4,
            &[
                0x45ab2c18, 0x454c064b, 0xc22ade16, 0xc54aeef9, 0x456abe58, 0xc55747c6, 0xc5993cbf,
                0xc517d2dd, 0xc5ab62b0, 0xc3e2388e, 0xc385cc29, 0x450cb26a, 0xc544c68f, 0x459746ce,
                0xc4fc2208, 0xc593f0c5,
            ],
        );
        assert_eq!(r.means, vec![0xc433cdcd, 0xc490027a, 0xc3506c6c, 0x4506b74d]);
        assert_eq!(r.clusters, vec![vec![1, 2, 3, 4], vec![0, 5, 6, 7]]);
        assert_eq!(r.optimal, vec![false; 6]);
        assert_eq!(r.last_flows, vec![0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1]);
    }

    /// Manhattan segment fixing: the point is moved along its own direction to the target length.
    #[test]
    fn fix_segment_scales_along_the_line() {
        let mut p = (4.0f32, 2.0f32);
        fix_segment((1.0, 1.0), 2.0, &mut p);
        assert_eq!(p, (2.5, 1.5));
        let mut q = (1.0f32, 1.0f32);
        fix_segment((1.0, 1.0), 2.0, &mut q);
        assert_eq!(q, (1.0, 1.0));
    }
}
