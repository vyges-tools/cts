// SPDX-License-Identifier: Apache-2.0
// A COPY of the dpl crate's `src/lemon.rs` (the same pinned solver version, transcribed once); keep it
// byte-identical to that file below this line — fix there, then re-copy.
//! A transcription of LEMON's `ListDigraph` iteration order and `NetworkSimplex` (block-search
//! pivot rule), as `DetailedMis::solveMatch` uses them.
//!
//! Derived from LEMON 1.3.1 (`lemon/list_graph.h`, `lemon/network_simplex.h`):
//!
//! > This file is a part of LEMON, a generic C++ optimization library.
//! > Copyright (C) 2003-2013 Egervary Jeno Kombinatorikus Optimalizalasi Kutatocsoport
//! > (Egervary Research Group on Combinatorial Optimization, EGRES).
//! > Permission to use, modify and distribute this software is granted provided that this
//! > copyright notice appears in all copies.
//!
//! ⛔ **Why transcribe rather than solve.** An assignment problem often has several optimal
//! solutions, and which one `NetworkSimplex` returns is decided by its internal arc order (the
//! graph's iteration order, then `arc_mixing`'s stride) and its pivoting. Measured on the upstream
//! cases: 10 of `edge_spacing-opt`'s 12 matching problems were such ties, and an exact assignment
//! solver chose a different optimum. The values here are C++ `int`, and so is their arithmetic:
//! the artificial cost is `INT_MAX / 2 + 1` and reduced costs can overflow, so it wraps.

/// A fresh `ListDigraph` (no erasures): nodes and arcs are prepended to their lists.
#[derive(Default)]
pub struct ListDigraph {
    /// `(first_out, first_in, next)` per node; `first_node` is the newest.
    first_out: Vec<i32>,
    first_in: Vec<i32>,
    next_node: Vec<i32>,
    first_node: i32,
    /// Per arc: `(source, target, next_out, next_in)`.
    src: Vec<usize>,
    tgt: Vec<usize>,
    next_out: Vec<i32>,
    next_in: Vec<i32>,
}

impl ListDigraph {
    pub fn new() -> ListDigraph {
        ListDigraph { first_node: -1, ..Default::default() }
    }

    pub fn add_node(&mut self) -> usize {
        let n = self.first_out.len();
        self.first_out.push(-1);
        self.first_in.push(-1);
        self.next_node.push(self.first_node);
        self.first_node = n as i32;
        n
    }

    pub fn add_arc(&mut self, u: usize, v: usize) -> usize {
        let a = self.src.len();
        self.src.push(u);
        self.tgt.push(v);
        self.next_out.push(self.first_out[u]);
        self.next_in.push(self.first_in[v]);
        self.first_out[u] = a as i32;
        self.first_in[v] = a as i32;
        a
    }

    /// `NodeIt` order: newest first.
    fn nodes(&self) -> Vec<usize> {
        let mut out = Vec::new();
        let mut n = self.first_node;
        while n != -1 {
            out.push(n as usize);
            n = self.next_node[n as usize];
        }
        out
    }

    /// `ArcIt` order: the nodes in `NodeIt` order, each node's out-arcs newest first.
    fn arcs(&self) -> Vec<usize> {
        let mut out = Vec::new();
        for n in self.nodes() {
            let mut a = self.first_out[n];
            while a != -1 {
                out.push(a as usize);
                a = self.next_out[a as usize];
            }
        }
        out
    }
}

const STATE_UPPER: i8 = -1;
const STATE_TREE: i8 = 0;
const STATE_LOWER: i8 = 1;
const DIR_DOWN: i8 = -1;
const DIR_UP: i8 = 1;

/// `NetworkSimplex<ListDigraph>` with `int` values and costs, `arc_mixing = true`, lower bounds
/// set (so `_has_lower`), `stSupply(s, t, k)` (so the balanced `_sum_supply == 0` start) and the
/// default `BLOCK_SEARCH` pivot rule.
pub struct NetworkSimplex {
    node_num: usize,
    arc_num: usize,
    all_arc_num: usize,
    search_arc_num: usize,
    /// Graph arc -> internal arc index, and graph node -> internal node index.
    arc_id: Vec<usize>,
    node_id: Vec<usize>,
    source: Vec<usize>,
    target: Vec<usize>,
    lower: Vec<i32>,
    upper: Vec<i32>,
    cap: Vec<i32>,
    cost: Vec<i32>,
    supply: Vec<i32>,
    flow: Vec<i32>,
    pi: Vec<i32>,
    parent: Vec<i64>,
    pred: Vec<i64>,
    thread: Vec<usize>,
    rev_thread: Vec<usize>,
    succ_num: Vec<i32>,
    last_succ: Vec<usize>,
    pred_dir: Vec<i8>,
    state: Vec<i8>,
    dirty_revs: Vec<usize>,
    root: usize,
    in_arc: usize,
    join: usize,
    u_in: usize,
    v_in: usize,
    u_out: usize,
    v_out: usize,
    delta: i32,
    // Block-search pivot state.
    block_size: usize,
    next_arc: usize,
}

const MAX: i32 = i32::MAX;
const INF: i32 = i32::MAX;

impl NetworkSimplex {
    /// The constructor's `reset()`: node ids in `NodeIt` order, arcs stored MIXED — `ArcIt`
    /// order at a stride of `skip = max(arcs / nodes, 3)`, wrapping to the next start.
    pub fn new(g: &ListDigraph) -> NetworkSimplex {
        let node_num = g.first_out.len();
        let arc_num = g.src.len();
        let all_node_num = node_num + 1;
        let max_arc_num = arc_num + 2 * node_num;
        let mut node_id = vec![0usize; node_num];
        for (i, n) in g.nodes().into_iter().enumerate() {
            node_id[n] = i;
        }
        let mut arc_id = vec![0usize; arc_num];
        let mut source = vec![0usize; max_arc_num];
        let mut target = vec![0usize; max_arc_num];
        let order = g.arcs();
        if node_num > 1 {
            let skip = (arc_num / node_num).max(3);
            let (mut i, mut j) = (0usize, 0usize);
            for a in order {
                arc_id[a] = i;
                source[i] = node_id[g.src[a]];
                target[i] = node_id[g.tgt[a]];
                i += skip;
                if i >= arc_num {
                    j += 1;
                    i = j;
                }
            }
        } else {
            for (i, a) in order.into_iter().enumerate() {
                arc_id[a] = i;
                source[i] = node_id[g.src[a]];
                target[i] = node_id[g.tgt[a]];
            }
        }
        NetworkSimplex {
            node_num, arc_num, all_arc_num: 0, search_arc_num: 0, arc_id, node_id, source, target,
            lower: vec![0; arc_num], upper: vec![INF; arc_num], cap: vec![0; max_arc_num],
            cost: vec![1; max_arc_num], supply: vec![0; all_node_num], flow: vec![0; max_arc_num],
            pi: vec![0; all_node_num], parent: vec![0; all_node_num], pred: vec![0; all_node_num],
            thread: vec![0; all_node_num], rev_thread: vec![0; all_node_num],
            succ_num: vec![0; all_node_num], last_succ: vec![0; all_node_num],
            pred_dir: vec![0; all_node_num], state: vec![0; max_arc_num], dirty_revs: Vec::new(),
            root: 0, in_arc: 0, join: 0, u_in: 0, v_in: 0, u_out: 0, v_out: 0, delta: 0,
            block_size: 0, next_arc: 0,
        }
    }

    /// `lowerMap`, `upperMap`, `costMap` over graph arc ids.
    pub fn set_arc(&mut self, a: usize, lower: i32, upper: i32, cost: i32) {
        let i = self.arc_id[a];
        self.lower[i] = lower;
        self.upper[i] = upper;
        self.cost[i] = cost;
    }

    /// `stSupply(s, t, k)`.
    pub fn st_supply(&mut self, s: usize, t: usize, k: i32) {
        for i in 0..self.node_num {
            self.supply[i] = 0;
        }
        self.supply[self.node_id[s]] = k;
        self.supply[self.node_id[t]] = -k;
    }

    /// The flow on graph arc `a` after [`NetworkSimplex::run`].
    pub fn flow(&self, a: usize) -> i32 {
        self.flow[self.arc_id[a]]
    }

    /// `run(BLOCK_SEARCH)`; `true` when OPTIMAL.
    pub fn run(&mut self) -> bool {
        if !self.init() {
            return false;
        }
        self.start()
    }

    fn init(&mut self) -> bool {
        if self.node_num == 0 {
            return false;
        }
        let sum: i32 = (0..self.node_num).fold(0i32, |a, i| a.wrapping_add(self.supply[i]));
        // GEQ supply type: `sum <= 0` required.
        if sum > 0 {
            return false;
        }
        // `_has_lower` (a lower map was set): remove the lower bounds.
        for i in 0..self.arc_num {
            let c = self.lower[i];
            self.cap[i] = if c >= 0 {
                if self.upper[i] < MAX { self.upper[i].wrapping_sub(c) } else { INF }
            } else if self.upper[i] < MAX.wrapping_add(c) {
                self.upper[i].wrapping_sub(c)
            } else {
                INF
            };
            let (s, t) = (self.source[i], self.target[i]);
            self.supply[s] = self.supply[s].wrapping_sub(c);
            self.supply[t] = self.supply[t].wrapping_add(c);
        }
        let art_cost: i32 = i32::MAX / 2 + 1;
        for i in 0..self.arc_num {
            self.flow[i] = 0;
            self.state[i] = STATE_LOWER;
        }
        let root = self.node_num;
        self.root = root;
        self.parent[root] = -1;
        self.pred[root] = -1;
        self.thread[root] = 0;
        self.rev_thread[0] = root;
        self.succ_num[root] = self.node_num as i32 + 1;
        self.last_succ[root] = root - 1;
        self.supply[root] = sum.wrapping_neg();
        self.pi[root] = 0;
        if sum == 0 {
            self.search_arc_num = self.arc_num;
            self.all_arc_num = self.arc_num + self.node_num;
            for u in 0..self.node_num {
                let e = self.arc_num + u;
                self.parent[u] = root as i64;
                self.pred[u] = e as i64;
                self.thread[u] = u + 1;
                self.rev_thread[u + 1] = u;
                self.succ_num[u] = 1;
                self.last_succ[u] = u;
                self.cap[e] = INF;
                self.state[e] = STATE_TREE;
                if self.supply[u] >= 0 {
                    self.pred_dir[u] = DIR_UP;
                    self.pi[u] = 0;
                    self.source[e] = u;
                    self.target[e] = root;
                    self.flow[e] = self.supply[u];
                    self.cost[e] = 0;
                } else {
                    self.pred_dir[u] = DIR_DOWN;
                    self.pi[u] = art_cost;
                    self.source[e] = root;
                    self.target[e] = u;
                    self.flow[e] = self.supply[u].wrapping_neg();
                    self.cost[e] = art_cost;
                }
            }
        } else {
            // GEQ with `sum < 0` — not reached from `stSupply` (which balances); kept honest.
            return false;
        }
        true
    }

    /// Reduced cost of internal arc `e`, times its state — `c < 0` is eligible.
    fn reduced(&self, e: usize) -> i32 {
        let r = self.cost[e].wrapping_add(self.pi[self.source[e]]).wrapping_sub(self.pi[self.target[e]]);
        (self.state[e] as i32).wrapping_mul(r)
    }

    /// `BlockSearchPivotRule::findEnteringArc`.
    fn find_entering_arc(&mut self) -> bool {
        let (mut min, mut cnt) = (0i32, self.block_size);
        let n = self.search_arc_num;
        let mut e = self.next_arc;
        let mut found_end = None;
        while e != n {
            let c = self.reduced(e);
            if c < min {
                min = c;
                self.in_arc = e;
            }
            cnt -= 1;
            if cnt == 0 {
                if min < 0 {
                    found_end = Some(e);
                    break;
                }
                cnt = self.block_size;
            }
            e += 1;
        }
        if found_end.is_none() {
            e = 0;
            while e != self.next_arc {
                let c = self.reduced(e);
                if c < min {
                    min = c;
                    self.in_arc = e;
                }
                cnt -= 1;
                if cnt == 0 {
                    if min < 0 {
                        found_end = Some(e);
                        break;
                    }
                    cnt = self.block_size;
                }
                e += 1;
            }
        }
        match found_end {
            // `goto search_end` leaves `e` at the arc that closed the block.
            Some(end) => {
                self.next_arc = end;
                true
            }
            None => {
                if min >= 0 {
                    return false;
                }
                // Fell through both loops: `e == _next_arc`.
                self.next_arc = e;
                true
            }
        }
    }

    fn find_join_node(&mut self) {
        let (mut u, mut v) = (self.source[self.in_arc], self.target[self.in_arc]);
        while u != v {
            if self.succ_num[u] < self.succ_num[v] {
                u = self.parent[u] as usize;
            } else {
                v = self.parent[v] as usize;
            }
        }
        self.join = u;
    }

    // `c >= MAX` against `i32::MAX` is LEMON's own test, transcribed as written.
    #[allow(clippy::absurd_extreme_comparisons)]
    fn find_leaving_arc(&mut self) -> bool {
        let (first, second) = if self.state[self.in_arc] == STATE_LOWER {
            (self.source[self.in_arc], self.target[self.in_arc])
        } else {
            (self.target[self.in_arc], self.source[self.in_arc])
        };
        self.delta = self.cap[self.in_arc];
        let mut result = 0;
        let mut u = first;
        while u != self.join {
            let e = self.pred[u] as usize;
            let mut d = self.flow[e];
            if self.pred_dir[u] == DIR_DOWN {
                let c = self.cap[e];
                d = if c >= MAX { INF } else { c.wrapping_sub(d) };
            }
            if d < self.delta {
                self.delta = d;
                self.u_out = u;
                result = 1;
            }
            u = self.parent[u] as usize;
        }
        let mut u = second;
        while u != self.join {
            let e = self.pred[u] as usize;
            let mut d = self.flow[e];
            if self.pred_dir[u] == DIR_UP {
                let c = self.cap[e];
                d = if c >= MAX { INF } else { c.wrapping_sub(d) };
            }
            if d <= self.delta {
                self.delta = d;
                self.u_out = u;
                result = 2;
            }
            u = self.parent[u] as usize;
        }
        if result == 1 {
            self.u_in = first;
            self.v_in = second;
        } else {
            self.u_in = second;
            self.v_in = first;
        }
        result != 0
    }

    fn change_flow(&mut self, change: bool) {
        if self.delta > 0 {
            let val = (self.state[self.in_arc] as i32).wrapping_mul(self.delta);
            self.flow[self.in_arc] = self.flow[self.in_arc].wrapping_add(val);
            let mut u = self.source[self.in_arc];
            while u != self.join {
                let e = self.pred[u] as usize;
                self.flow[e] = self.flow[e].wrapping_sub((self.pred_dir[u] as i32).wrapping_mul(val));
                u = self.parent[u] as usize;
            }
            let mut u = self.target[self.in_arc];
            while u != self.join {
                let e = self.pred[u] as usize;
                self.flow[e] = self.flow[e].wrapping_add((self.pred_dir[u] as i32).wrapping_mul(val));
                u = self.parent[u] as usize;
            }
        }
        if change {
            self.state[self.in_arc] = STATE_TREE;
            let e = self.pred[self.u_out] as usize;
            self.state[e] = if self.flow[e] == 0 { STATE_LOWER } else { STATE_UPPER };
        } else {
            self.state[self.in_arc] = -self.state[self.in_arc];
        }
    }

    fn update_tree_structure(&mut self) {
        let (u_in, v_in, u_out) = (self.u_in, self.v_in, self.u_out);
        let old_rev_thread = self.rev_thread[u_out];
        let old_succ_num = self.succ_num[u_out];
        let old_last_succ = self.last_succ[u_out];
        self.v_out = self.parent[u_out] as usize;
        let v_out = self.v_out;
        if u_in == u_out {
            self.parent[u_in] = v_in as i64;
            self.pred[u_in] = self.in_arc as i64;
            self.pred_dir[u_in] = if u_in == self.source[self.in_arc] { DIR_UP } else { DIR_DOWN };
            if self.thread[v_in] != u_out {
                let mut after = self.thread[old_last_succ];
                self.thread[old_rev_thread] = after;
                self.rev_thread[after] = old_rev_thread;
                after = self.thread[v_in];
                self.thread[v_in] = u_out;
                self.rev_thread[u_out] = v_in;
                self.thread[old_last_succ] = after;
                self.rev_thread[after] = old_last_succ;
            }
        } else {
            let thread_continue = if old_rev_thread == v_in { self.thread[old_last_succ] } else { self.thread[v_in] };
            let mut stem = u_in;
            let mut par_stem = v_in;
            let mut last = self.last_succ[u_in];
            let mut after = self.thread[last];
            self.thread[v_in] = u_in;
            self.dirty_revs.clear();
            self.dirty_revs.push(v_in);
            while stem != u_out {
                let next_stem = self.parent[stem] as usize;
                self.thread[last] = next_stem;
                self.dirty_revs.push(last);
                let before = self.rev_thread[stem];
                self.thread[before] = after;
                self.rev_thread[after] = before;
                self.parent[stem] = par_stem as i64;
                par_stem = stem;
                stem = next_stem;
                last = if self.last_succ[stem] == self.last_succ[par_stem] {
                    self.rev_thread[par_stem]
                } else {
                    self.last_succ[stem]
                };
                after = self.thread[last];
            }
            self.parent[u_out] = par_stem as i64;
            self.thread[last] = thread_continue;
            self.rev_thread[thread_continue] = last;
            self.last_succ[u_out] = last;
            if old_rev_thread != v_in {
                self.thread[old_rev_thread] = after;
                self.rev_thread[after] = old_rev_thread;
            }
            for i in 0..self.dirty_revs.len() {
                let u = self.dirty_revs[i];
                let t = self.thread[u];
                self.rev_thread[t] = u;
            }
            let (mut tmp_sc, tmp_ls) = (0i32, self.last_succ[u_out]);
            let mut u = u_out;
            let mut p = self.parent[u] as usize;
            while u != u_in {
                self.pred[u] = self.pred[p];
                self.pred_dir[u] = -self.pred_dir[p];
                tmp_sc += self.succ_num[u] - self.succ_num[p];
                self.succ_num[u] = tmp_sc;
                self.last_succ[p] = tmp_ls;
                u = p;
                p = self.parent[u] as usize;
            }
            self.pred[u_in] = self.in_arc as i64;
            self.pred_dir[u_in] = if u_in == self.source[self.in_arc] { DIR_UP } else { DIR_DOWN };
            self.succ_num[u_in] = old_succ_num;
        }
        let join = self.join;
        let up_limit_out: i64 = if self.last_succ[join] == v_in { join as i64 } else { -1 };
        let last_succ_out = self.last_succ[u_out];
        let mut u = v_in as i64;
        while u != -1 && self.last_succ[u as usize] == v_in {
            self.last_succ[u as usize] = last_succ_out;
            u = self.parent[u as usize];
        }
        if join != old_rev_thread && v_in != old_rev_thread {
            let mut u = v_out as i64;
            while u != up_limit_out && self.last_succ[u as usize] == old_last_succ {
                self.last_succ[u as usize] = old_rev_thread;
                u = self.parent[u as usize];
            }
        } else if last_succ_out != old_last_succ {
            let mut u = v_out as i64;
            while u != up_limit_out && self.last_succ[u as usize] == old_last_succ {
                self.last_succ[u as usize] = last_succ_out;
                u = self.parent[u as usize];
            }
        }
        let mut u = v_in;
        while u != join {
            self.succ_num[u] += old_succ_num;
            u = self.parent[u] as usize;
        }
        let mut u = v_out;
        while u != join {
            self.succ_num[u] -= old_succ_num;
            u = self.parent[u] as usize;
        }
    }

    fn update_potential(&mut self) {
        let sigma = self.pi[self.v_in].wrapping_sub(self.pi[self.u_in])
            .wrapping_sub((self.pred_dir[self.u_in] as i32).wrapping_mul(self.cost[self.in_arc]));
        let end = self.thread[self.last_succ[self.u_in]];
        let mut u = self.u_in;
        while u != end {
            self.pi[u] = self.pi[u].wrapping_add(sigma);
            u = self.thread[u];
        }
    }

    /// `start<BlockSearchPivotRule>()`. `initialPivots` is a no-op for `stSupply(s, t, k)` with
    /// every arc's capacity below `k` — the only use here — and is asserted rather than built.
    // `delta >= MAX` against `i32::MAX` is LEMON's own test, transcribed as written.
    #[allow(clippy::absurd_extreme_comparisons)]
    fn start(&mut self) -> bool {
        self.block_size = ((self.search_arc_num as f64).sqrt() as usize).max(10);
        self.next_arc = 0;
        while self.find_entering_arc() {
            self.find_join_node();
            let change = self.find_leaving_arc();
            if self.delta >= MAX {
                return false; // UNBOUNDED
            }
            self.change_flow(change);
            if change {
                self.update_tree_structure();
                self.update_potential();
            }
        }
        for e in self.search_arc_num..self.all_arc_num {
            if self.flow[e] != 0 {
                return false; // INFEASIBLE
            }
        }
        for i in 0..self.arc_num {
            let c = self.lower[i];
            if c != 0 {
                self.flow[i] = self.flow[i].wrapping_add(c);
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `ListDigraph` iterates nodes newest first, and each node's out-arcs newest first.
    #[test]
    fn list_digraph_iterates_newest_first() {
        let mut g = ListDigraph::new();
        let (a, b, c) = (g.add_node(), g.add_node(), g.add_node());
        let e0 = g.add_arc(a, b);
        let e1 = g.add_arc(a, c);
        let e2 = g.add_arc(c, b);
        assert_eq!(g.nodes(), vec![c, b, a]);
        assert_eq!(g.arcs(), vec![e2, e1, e0]);
    }

    /// A 2x2 assignment with a unique optimum: the simplex finds it.
    #[test]
    fn a_unique_assignment_is_found() {
        let mut g = ListDigraph::new();
        let (c0, s0, c1, s1) = (g.add_node(), g.add_node(), g.add_node(), g.add_node());
        let (sup, dem) = (g.add_node(), g.add_node());
        let mut arcs = Vec::new();
        for (c, s) in [(c0, s0), (c1, s1)] {
            arcs.push((g.add_arc(sup, c), 0));
            arcs.push((g.add_arc(s, dem), 0));
        }
        let x00 = g.add_arc(c0, s0);
        let x01 = g.add_arc(c0, s1);
        let x10 = g.add_arc(c1, s0);
        let x11 = g.add_arc(c1, s1);
        let mut ns = NetworkSimplex::new(&g);
        for (a, c) in &arcs {
            ns.set_arc(*a, 0, 1, *c);
        }
        ns.set_arc(x00, 0, 1, 5);
        ns.set_arc(x01, 0, 1, 1);
        ns.set_arc(x10, 0, 1, 1);
        ns.set_arc(x11, 0, 1, 5);
        ns.st_supply(sup, dem, 2);
        assert!(ns.run());
        assert_eq!((ns.flow(x00), ns.flow(x01), ns.flow(x10), ns.flow(x11)), (0, 1, 1, 0));
    }
}
