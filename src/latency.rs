// SPDX-License-Identifier: Apache-2.0
//! Latency balancing (`LatencyBalancer`): for a clock with sub-trees (a macro tree and its
//! register tree, the trees below clock gaters), the written trees are timed, the sinks' and the
//! leaf trees' clock arrivals read, and delay buffers inserted so the faster branches catch up with
//! the slowest.
//!
//! Each stage is its own function, in the reference's order and named after it. Delays are integers
//! in picoseconds (`dpUnit_`); the buffer delays come from the liberty tables directly.

use crate::discovery::{self, ITerm};
use crate::htree::{Legalizer, P};
use crate::log::{Log, Res};
use crate::write::{Tree, TreeDb};
use std::collections::{BTreeMap, HashMap};

/// `dpUnit_`: picoseconds.
const DP_UNIT: f64 = 1e12;

/// What the balancer reads beyond the database.
pub struct BOptions<'a> {
    /// `getDlyBufferList`.
    pub delay_buffers: &'a [String],
    /// The clock wire's capacitance and resistance per DBU.
    pub cap_per_dbu: f64,
    pub res_per_dbu: f64,
    pub wire_segment_unit: i32,
}

/// `GraphNode`.
#[derive(Debug, Clone)]
struct Node {
    name: String,
    /// The node's terminal: a sink's (or gater's, or leaf tree buffer's) input; the root's driver
    /// output (`None` when the root net has no driving instance).
    input_term: Option<ITerm>,
    children: Vec<usize>,
    arrival: Option<f64>,
    dly_needed: Option<i64>,
}

/// One balancer (`LatencyBalancer`): the graph from one root tree.
struct Balancer<'a> {
    o: &'a BOptions<'a>,
    trees: &'a [Tree],
    root: usize,
    graph: Vec<Node>,
    /// `inst2builder_`: a leaf tree's top buffer → the tree.
    inst_to_tree: HashMap<String, usize>,
    worse_delay: f32,
    dly_buffers: Vec<String>,
    buffers_delay: Vec<i64>,
    delay_buf_index: u32,
}

/// `balanceMacroRegisterLatencies`: each root tree with sub-trees, from the last built.
pub fn balance_macro_register_latencies(log: &mut Log, d: &mut dyn TreeDb, o: &BOptions, trees: &[Tree], legalizers: &[Legalizer], insertion_delay: bool) -> Res<i32> {
    if !insertion_delay {
        return Ok(0);
    }
    let mut total = 0i32;
    for i in (0..trees.len()).rev() {
        if trees[i].parent.is_none() && trees[i].has_children {
            let mut b = Balancer { o, trees, root: i, graph: Vec::new(), inst_to_tree: HashMap::new(), worse_delay: f32::MIN_POSITIVE, dly_buffers: Vec::new(), buffers_delay: Vec::new(), delay_buf_index: 0 };
            total += b.run(log, d, &legalizers[i])? as i32;
        }
    }
    if total != 0 {
        log.info("CTS", 37, format!("Total number of delay buffers: {total}"));
    }
    Ok(total)
}

/// `TechChar::computeBufferDelay`: the driver's rise→rise gate delay into `load_cap` (plus the
/// loads' rise input caps), its output slew cycled through the table twice more; the largest over
/// the arcs.
pub fn compute_buffer_delay(d: &dyn TreeDb, driver: &str, loads: &[f32], load_cap: f64) -> f32 {
    let Some(cell) = d.liberty_cell(driver) else { return 0.0 };
    let Some((input, output)) = cell.buffer_ports() else { return 0.0 };
    let mut load_cap = load_cap;
    for &c in loads {
        load_cap += f64::from(c);
    }
    let load = load_cap as f32;
    let mut max_rise = 0.0f32;
    for set in cell.arc_sets.iter().filter(|s| s.from == input.name && s.to == output.name) {
        for arc in &set.arcs {
            let vyges_sta::liberty::Model::Gate(model) = &arc.model else { continue };
            if arc.from_rf != vyges_sta::liberty::RISE || arc.to_rf != vyges_sta::liberty::RISE {
                continue;
            }
            let (_, s1) = model.gate_delay(0.0, load);
            let (_, s2) = model.gate_delay(s1, load);
            let (delay, _) = model.gate_delay(s2, load);
            // `delayGreater`: fuzzily greater.
            if delay > max_rise && !crate::write::sta_fuzzy_equal(delay, max_rise) {
                max_rise = delay;
            }
        }
    }
    max_rise
}

/// A buffer's input pin's rise capacitance (`bufferPorts` input, rise, max).
fn buffer_input_rise_cap(d: &dyn TreeDb, cell: &str) -> Option<f32> {
    let c = d.liberty_cell(cell)?;
    let (i, _) = c.buffer_ports()?;
    Some(i.capacitance[vyges_sta::liberty::RISE][vyges_sta::liberty::MAX])
}

/// A terminal's liberty port rise capacitance (none without a port).
fn pin_rise_cap(d: &dyn TreeDb, t: &ITerm) -> Option<f32> {
    let master = d.inst_master(&t.0);
    let c = d.liberty_cell(&master)?;
    c.ports.iter().find(|p| p.name == t.1).map(|p| p.capacitance[vyges_sta::liberty::RISE][vyges_sta::liberty::MAX])
}

/// `odb::Rect` after `mergeInit` and merges: `(xMin, yMin, xMax, yMax)`.
#[derive(Clone, Copy)]
struct Bbox(i32, i32, i32, i32);

impl Bbox {
    fn init() -> Bbox {
        Bbox(i32::MAX, i32::MAX, i32::MIN, i32::MIN)
    }
    fn merge(&mut self, (x, y): (i32, i32)) {
        self.0 = self.0.min(x);
        self.1 = self.1.min(y);
        self.2 = self.2.max(x);
        self.3 = self.3.max(y);
    }
    fn x_center(&self) -> i32 {
        (self.0 + self.2) / 2
    }
    fn y_center(&self) -> i32 {
        (self.1 + self.3) / 2
    }
    fn dx(&self) -> i32 {
        self.2 - self.0
    }
    fn dy(&self) -> i32 {
        self.3 - self.1
    }
}

/// `getAvgXY` with the database's messages logged in order; an unplaced pin leaves the caller's
/// coordinates (0, 0 here).
fn avg_xy(log: &mut Log, d: &mut dyn TreeDb, t: &ITerm) -> (i32, i32) {
    let (at, msgs) = d.iterm_avg_xy_logged(t);
    for m in msgs {
        log.report(m);
    }
    at.unwrap_or((0, 0))
}

impl Balancer<'_> {
    /// `run`.
    fn run(&mut self, log: &mut Log, d: &mut dyn TreeDb, legal: &Legalizer) -> Res<u32> {
        let clock = self.trees[self.root].sdc_clock.clone();
        log.info("CTS", 33, format!("Balancing latency for clock {clock}"));
        // `initSta`: the timer is built on the written design when first asked.
        self.find_leaf_builders(self.root);
        let top = self.trees[self.root].top_input_net.clone();
        self.build_graph(log, d, &top)?;
        self.compute_buffers_delay(log, d, 0.0);
        if self.buffers_delay.is_empty() {
            return Ok(0);
        }
        self.balance_latencies(log, d, legal, 0)?;
        log.info("CTS", 36, format!(" inserted {} delay buffers", self.delay_buf_index));
        Ok(self.delay_buf_index)
    }

    /// `findLeafBuilders`: every tree below the root without sub-trees (a macro tree's register tree
    /// does not count), by its top buffer.
    fn find_leaf_builders(&mut self, t: usize) {
        let children: Vec<usize> = (0..self.trees.len()).filter(|&c| self.trees[c].parent == Some(t)).collect();
        let leaf = if self.trees[t].tree_type == crate::htree::TreeType::Macro { children.len() == 1 } else { children.is_empty() };
        if leaf {
            self.inst_to_tree.insert(self.trees[t].top_buffer_name.clone(), t);
        }
        for c in children {
            self.find_leaf_builders(c);
        }
    }

    /// `buildGraph`: from the root net, depth first (a stack), every sink, gater and leaf tree's
    /// top buffer below it; sinks and leaf trees carry their clock arrivals.
    fn build_graph(&mut self, log: &mut Log, d: &mut dyn TreeDb, clk_input_net: &str) -> Res<()> {
        let root_out = d.net_first_output(clk_input_net);
        let name = match &root_out {
            None => clk_input_net.to_string(),
            Some(t) => t.0.clone(),
        };
        self.graph.push(Node { name, input_term: root_out, children: Vec::new(), arrival: None, dly_needed: None });
        let mut stack = vec![0usize];
        while let Some(driver) = stack.pop() {
            let driver_net = match &self.graph[driver].input_term {
                Some(t) => {
                    let Some(out) = d.inst_first_output(&t.0) else { continue };
                    match d.iterm_net(&out) {
                        Some(n) => n,
                        None => continue,
                    }
                }
                None => clk_input_net.to_string(),
            };
            for sink in d.net_iterms(&driver_net) {
                if d.iterm_io_type(&sink) != "INPUT" {
                    continue;
                }
                let is_sink = discovery::is_sink(d, &sink);
                if !is_sink && !propagate_clock(d, &sink) {
                    continue;
                }
                if !is_sink {
                    let Some(out) = d.inst_first_output(&sink.0) else { continue };
                    if d.iterm_net(&out).is_none() {
                        continue;
                    }
                }
                let id = self.graph.len();
                let inst = sink.0.clone();
                self.graph.push(Node { name: inst.clone(), input_term: Some(sink.clone()), children: Vec::new(), arrival: None, dly_needed: None });
                self.graph[driver].children.push(id);
                if let Some(&t) = self.inst_to_tree.get(&inst) {
                    let avg = self.compute_ave_sink_arrivals(log, d, t)?;
                    self.worse_delay = self.worse_delay.max(avg);
                    self.graph[id].arrival = Some(f64::from(avg));
                    continue;
                }
                if is_sink {
                    let arrival = self.vertex_clk_arrival(log, d, &sink, clk_input_net)?;
                    // Insertion delays are refused upstream: none here.
                    let ins_delay = 0.0f32;
                    self.worse_delay = self.worse_delay.max(arrival + ins_delay);
                    self.graph[id].arrival = Some(f64::from(arrival + ins_delay));
                    log.debug("CTS", "insertion delay", 2, || format!("Sink {}/{}: average sink arrival is {}", sink.0, sink.1, crate::log::fmt_e(f64::from(arrival), 3)));
                    continue;
                }
                stack.push(id);
            }
        }
        Ok(())
    }

    /// `getVertexClkArrival`: the first max path of a rising clock edge whose clock path starts on
    /// the root net; CTS-0179 and 0 when none does.
    fn vertex_clk_arrival(&self, log: &mut Log, d: &mut dyn TreeDb, t: &ITerm, top_net: &str) -> Res<f32> {
        match d.sink_clock_arrival(t, top_net)? {
            Some(a) => Ok(a),
            None => {
                log.warn("CTS", 179, format!("No paths found for pin {}/{}.", t.0, t.1));
                Ok(0.0)
            }
        }
    }

    /// `computeAveSinkArrivals`: the mean clock arrival over a leaf tree's sinks (through gaters).
    fn compute_ave_sink_arrivals(&self, log: &mut Log, d: &mut dyn TreeDb, t: usize) -> Res<f32> {
        let tree = &self.trees[t];
        let top = tree.top_input_net.clone();
        let mut sum = 0.0f32;
        let mut n = 0u32;
        for s in &tree.clock.sinks {
            let (i, p) = s.name.rsplit_once('/').unwrap_or((&s.name, ""));
            self.compute_sink_arrival_recur(log, d, &top, &(i.to_string(), p.to_string()), &mut sum, &mut n)?;
        }
        let avg = if n != 0 { sum / n as f32 } else { 0.0 };
        log.debug("CTS", "insertion delay", 2, || format!("{} {}: average sink arrival is {}", if tree.tree_type == crate::htree::TreeType::Macro { "macro tree" } else { "register tree" }, tree.clock.name, crate::log::fmt_e(f64::from(avg), 3)));
        Ok(avg)
    }

    /// `computeSinkArrivalRecur`.
    fn compute_sink_arrival_recur(&self, log: &mut Log, d: &mut dyn TreeDb, top: &str, t: &ITerm, sum: &mut f32, n: &mut u32) -> Res<()> {
        if discovery::is_sink(d, t) {
            let arrival = self.vertex_clk_arrival(log, d, t, top)?;
            *sum += arrival + 0.0;
            *n += 1;
            return Ok(());
        }
        let Some(out) = d.inst_first_output(&t.0) else { return Ok(()) };
        let out_net = d.iterm_net(&out);
        let propagate = propagate_clock(d, t);
        if let (Some(net), true) = (out_net, propagate) {
            for i in d.net_iterms(&net) {
                if d.iterm_io_type(&i) == "INPUT" {
                    self.compute_sink_arrival_recur(log, d, top, &i, sum, n)?;
                }
            }
        }
        Ok(())
    }

    /// `computeBuffersDelay`: each delay buffer with a master, its delay driving itself.
    fn compute_buffers_delay(&mut self, log: &mut Log, d: &dyn TreeDb, extra_out_cap: f64) {
        log.debug("CTS", "insertion delay", 3, || "Buffer list = [".into());
        for b in self.o.delay_buffers {
            if !d.has_master_cell(b) {
                log.debug("CTS", "insertion delay", 3, || format!("{b} : skipped, no master"));
                continue;
            }
            let loads: Vec<f32> = buffer_input_rise_cap(d, b).into_iter().collect();
            let delay = (f64::from(compute_buffer_delay(d, b, &loads, extra_out_cap)) * DP_UNIT).round() as i64;
            if delay <= 0 {
                log.debug("CTS", "insertion delay", 3, || format!("{b} : skipped, no delay"));
                continue;
            }
            log.debug("CTS", "insertion delay", 3, || format!("{b} : {delay}"));
            self.dly_buffers.push(b.clone());
            self.buffers_delay.push(delay);
        }
        log.debug("CTS", "insertion delay", 3, || "]".into());
        if self.buffers_delay.is_empty() {
            log.warn("CTS", 180, format!("No delay buffer has a measurable delay at {} resolution; latency balancing is skipped.", crate::log::fmt_f64(1.0 / DP_UNIT)));
        }
    }

    /// `computeWireLumpedDelay` to one buffer: half the wire's cap plus the buffer's rise input cap,
    /// through the wire's resistance. Returns the delay and the wire cap.
    fn wire_lumped_delay_to(&self, d: &dyn TreeDb, load: &str, wl: f64) -> (i64, f64) {
        let wire_cap = wl * self.o.cap_per_dbu;
        let mut total = wire_cap / 2.0;
        let res = wl * self.o.res_per_dbu;
        if !load.is_empty() {
            if let Some(c) = buffer_input_rise_cap(d, load) {
                total += f64::from(c);
            }
        }
        ((res * total * DP_UNIT).round() as i64, wire_cap)
    }

    /// `computeWireLumpedDelay` to terminals, with extra load cap.
    fn wire_lumped_delay_to_pins(&self, d: &dyn TreeDb, loads: &[ITerm], extra: f64, wl: f64) -> (i64, f64) {
        let wire_cap = wl * self.o.cap_per_dbu;
        let mut total = wire_cap / 2.0 + extra;
        let res = wl * self.o.res_per_dbu;
        for t in loads {
            if let Some(c) = pin_rise_cap(d, t) {
                total += f64::from(c);
            }
        }
        ((res * total * DP_UNIT).round() as i64, wire_cap)
    }

    /// `solveDP`: the fewest buffers whose chain delay (each driving the next, the last the sinks,
    /// with lumped wire delays) is nearest the target, overshooting by at most one buffer delay.
    // Index loops, as the reference's `dp[w][j]` reads.
    #[allow(clippy::needless_range_loop)]
    fn solve_dp(&self, d: &dyn TreeDb, target: i64, wl: f64, sinks: &[ITerm], hpwl: f64) -> (i64, Vec<String>) {
        let n = self.dly_buffers.len();
        let mut max_buf_delay = 0i64;
        let (sink_wire_dly, sink_wire_cap) = self.wire_lumped_delay_to_pins(d, sinks, hpwl * self.o.cap_per_dbu, wl);
        let sink_caps: Vec<f32> = sinks.iter().filter_map(|t| pin_rise_cap(d, t)).collect();
        let mut sink_delay = vec![0i64; n];
        for j in 0..n {
            let delay = (f64::from(compute_buffer_delay(d, &self.dly_buffers[j], &sink_caps, sink_wire_cap + hpwl * self.o.cap_per_dbu)) * DP_UNIT).round() as i64 + sink_wire_dly;
            sink_delay[j] = delay;
            max_buf_delay = max_buf_delay.max(delay);
        }
        let mut pair = vec![vec![0i64; n]; n];
        for i in 0..n {
            for j in 0..n {
                let (wire_dly, wire_cap) = self.wire_lumped_delay_to(d, &self.dly_buffers[j], wl);
                let load: Vec<f32> = buffer_input_rise_cap(d, &self.dly_buffers[j]).into_iter().collect();
                let delay = (f64::from(compute_buffer_delay(d, &self.dly_buffers[i], &load, wire_cap)) * DP_UNIT).round() as i64 + wire_dly;
                pair[i][j] = delay;
                max_buf_delay = max_buf_delay.max(delay);
            }
        }
        let max_w = target + max_buf_delay;
        const UNSET: i32 = -1;
        const DRIVES_SINKS: i32 = -1;
        const NO_NEXT: i32 = -2;
        let size = (max_w as usize + 1) * n;
        let mut dp = vec![UNSET; size];
        let mut nxt = vec![NO_NEXT; size];
        let st = |w: i64, j: usize| w as usize * n + j;
        for j in 0..n {
            let b = sink_delay[j];
            if b > 0 && b <= max_w {
                dp[st(b, j)] = 1;
                nxt[st(b, j)] = DRIVES_SINKS;
            }
        }
        for w in 0..=max_w {
            for j in 0..n {
                let len = dp[st(w, j)];
                if len == UNSET {
                    continue;
                }
                for i in 0..n {
                    let b = pair[i][j];
                    let nw = w + b;
                    if b <= 0 || nw > max_w {
                        continue;
                    }
                    let next = st(nw, i);
                    if dp[next] == UNSET || len + 1 < dp[next] {
                        dp[next] = len + 1;
                        nxt[next] = j as i32;
                    }
                }
            }
        }
        let (best_w, best_j) = pick_best_chain(&dp, n, max_w, target);
        if best_j == -1 {
            return (0, Vec::new());
        }
        let mut out = Vec::new();
        let (mut w, mut cur) = (best_w, best_j);
        let mut left = dp[st(best_w, best_j as usize)];
        while cur >= 0 && left > 0 {
            out.push(self.dly_buffers[cur as usize].clone());
            let next = nxt[st(w, cur as usize)];
            if next == DRIVES_SINKS {
                break;
            }
            w -= pair[cur as usize][next as usize];
            cur = next;
            left -= 1;
        }
        (best_w, out)
    }

    /// `backtrackCount`.
    fn backtrack_count(elements: &[i32], delays: &[i64], target: i64) -> i32 {
        let mut n = 0;
        let mut w = target;
        while w > 0 && elements[w as usize] != -1 {
            let b = delays[elements[w as usize] as usize];
            if b <= 0 {
                break;
            }
            n += 1;
            w -= b;
        }
        n
    }

    /// `computeNumberOfDelayBuffers`: an unconstrained combination first (its count), then the
    /// chain re-solved with each pass's wire length until the count stops falling.
    fn compute_number_of_delay_buffers(&self, log: &mut Log, d: &mut dyn TreeDb, delay_needed: i64, src: (i32, i32), sinks: &[ITerm]) -> Vec<String> {
        let target = delay_needed;
        log.debug("CTS", "insertion delay", 2, || format!("  target delay: {target}"));
        if target <= 0 || self.buffers_delay.is_empty() {
            return Vec::new();
        }
        let mut dp = vec![0i64; target as usize + 1];
        let mut elements = vec![-1i32; target as usize + 1];
        for w in 0..=target {
            for (i, &b) in self.buffers_delay.iter().enumerate() {
                let best_prev = if b > w { 0 } else { dp[(w - b) as usize] };
                if (dp[w as usize] - w).abs() >= (best_prev + b - w).abs() {
                    elements[w as usize] = i as i32;
                    dp[w as usize] = best_prev + b;
                }
            }
        }
        if dp[target as usize] == 0 {
            return Vec::new();
        }
        let mut n_bufs = Self::backtrack_count(&elements, &self.buffers_delay, target);
        log.debug("CTS", "insertion delay", 4, || format!("Initial best = {}", dp[target as usize]));
        log.debug("CTS", "insertion delay", 4, || format!("Initial n bufs = {n_bufs}"));
        let mut bbox = Bbox::init();
        for s in sinks {
            let at = avg_xy(log, d, s);
            bbox.merge(at);
        }
        let hpwl = f64::from(bbox.dx() + bbox.dy()) / 2.0;
        let mut prev_n = i32::MAX;
        let mut pass = 2;
        let mut result = (0i64, Vec::new());
        while n_bufs < prev_n {
            let off_x = f64::from(bbox.x_center() - src.0) / f64::from(n_bufs + 1);
            let off_y = f64::from(bbox.y_center() - src.1) / f64::from(n_bufs + 1);
            let wl = off_x.abs() + off_y.abs();
            result = self.solve_dp(d, target, wl, sinks, hpwl);
            log.debug("CTS", "insertion delay", 4, || format!("Pass {pass} best = {}", result.0));
            prev_n = n_bufs;
            n_bufs = result.1.len() as i32;
            log.debug("CTS", "insertion delay", 4, || format!("Pass {pass} n bufs = {n_bufs}"));
            if n_bufs == 0 {
                break;
            }
            pass += 1;
        }
        log.debug("CTS", "insertion delay", 2, || format!("  Max achievable delay {}", result.0));
        log.debug("CTS", "insertion delay", 2, || format!("  using buffers [{}]", result.1.join(", ")));
        result.1
    }

    /// `balanceLatencies`: depth first; a leaf needs the gap to the slowest arrival; a node groups
    /// its children by delay needed and, from the largest, puts the difference to the next group
    /// in front of the larger one with delay buffers.
    fn balance_latencies(&mut self, log: &mut Log, d: &mut dyn TreeDb, legal: &Legalizer, node: usize) -> Res<()> {
        if self.graph[node].children.is_empty() {
            if let Some(a) = self.graph[node].arrival {
                self.graph[node].dly_needed = Some(((f64::from(self.worse_delay) - a) * DP_UNIT).round() as i64);
            }
            return Ok(());
        }
        let src = match &self.graph[node].input_term {
            None => {
                let top = self.trees[self.root].top_input_net.clone();
                d.net_first_bterm_center(&top).ok_or_else(|| crate::log::Stop::Refused(format!("{top}: the root net has no driver and no port")))?
            }
            Some(t) => avg_xy(log, d, &t.clone()),
        };
        let mut by_need: BTreeMap<i64, Vec<ITerm>> = BTreeMap::new();
        for c in self.graph[node].children.clone() {
            self.balance_latencies(log, d, legal, c)?;
            let Some(need) = self.graph[c].dly_needed else { continue };
            by_need.entry(need).or_default().push(self.graph[c].input_term.clone().expect("a child has a terminal"));
        }
        let name = self.graph[node].name.clone();
        log.debug("CTS", "insertion delay", 1, || format!("at node {name}"));
        let mut previous: Option<i64> = None;
        let mut sinks_input: Vec<ITerm> = Vec::new();
        for (need, children) in by_need.into_iter().rev() {
            if log.debug_on("CTS", "insertion delay", 2) {
                log.debug("CTS", "insertion delay", 2, || " sinks [".into());
                for c in &children {
                    log.debug("CTS", "insertion delay", 2, || format!("{}, ", c.0));
                }
                log.debug("CTS", "insertion delay", 2, || "]".into());
                log.debug("CTS", "insertion delay", 2, || format!(" need {need} delay"));
            }
            let Some(prev) = previous else {
                previous = Some(need);
                sinks_input = children;
                continue;
            };
            let diff = prev - need;
            log.debug("CTS", "insertion delay", 3, || format!(" previous delay = {prev}"));
            log.debug("CTS", "insertion delay", 3, || format!(" Has a {diff} dly diff with previous"));
            let masters = self.compute_number_of_delay_buffers(log, d, diff, src, &sinks_input);
            if masters.is_empty() {
                sinks_input.extend(children);
                log.debug("CTS", "insertion delay", 1, || " Not possible to insert buffers".into());
                continue;
            }
            log.debug("CTS", "insertion delay", 2, || format!(" dly buffers needed: {}", masters.len()));
            let input = self.insert_delay_buffers(log, d, legal, src, &masters, &sinks_input)?;
            sinks_input = children;
            sinks_input.push(input);
            previous = Some(need);
        }
        self.graph[node].dly_needed = previous;
        Ok(())
    }

    /// `insertDelayBuffers`: the loads moved behind a chain of buffers spaced from the source toward
    /// their centre, each legalized through the root tree's legalizer and inserted by the
    /// database's own buffer insertion. Returns the first buffer's input.
    fn insert_delay_buffers(&mut self, log: &mut Log, d: &mut dyn TreeDb, legal: &Legalizer, src: (i32, i32), masters: &[String], sinks: &[ITerm]) -> Res<ITerm> {
        let n = masters.len();
        log.debug("CTS", "insertion delay", 3, || format!("Inserting {n} buffers for sinks:"));
        let mut driving_net: Option<String> = None;
        let mut bbox = Bbox::init();
        for s in sinks {
            if driving_net.is_none() {
                driving_net = d.iterm_net(s);
            }
            log.debug("CTS", "insertion delay", 3, || format!("  {}/{}", s.0, s.1));
            d.disconnect(s)?;
            let at = avg_xy(log, d, s);
            bbox.merge(at);
        }
        let Some(mut net) = driving_net else { return crate::log::refuse("insertDelayBuffers: the loads have no net") };
        // Float arithmetic, as the reference's `float` offsets.
        let off_x = (bbox.x_center() - src.0) as f32 / (n as i32 + 1) as f32;
        let off_y = (bbox.y_center() - src.1) as f32 / (n as i32 + 1) as f32;
        let unit = f64::from(self.o.wire_segment_unit);
        let clk = self.trees[self.root].sdc_clock.clone();
        let mut first: Option<String> = None;
        for (i, master) in masters.iter().enumerate() {
            let k = (i + 1) as f32;
            let loc_x = f64::from(src.0 as f32 + off_x * k) / unit;
            let loc_y = f64::from(src.1 as f32 + off_y * k) / unit;
            let legal_at = legal.legalize_one_buffer(P::new(loc_x, loc_y), master);
            let loc = ((legal_at.x * unit) as i32, (legal_at.y * unit) as i32);
            let net_name = format!("delaynet_{}_{clk}", self.delay_buf_index);
            let buf_name = format!("delaybuf_{}_{clk}", self.delay_buf_index);
            self.delay_buf_index += 1;
            let last = d.insert_buffer_before_loads(&net, sinks, master, loc, &buf_name, &net_name)?;
            log.debug("CTS", "insertion delay", 1, || format!("new delay buffer {last} inserted at ({} {})", loc.0, loc.1));
            let out = d.inst_first_output(&last).ok_or_else(|| crate::log::Stop::Refused(format!("{last}: no output")))?;
            net = d.iterm_net(&out).ok_or_else(|| crate::log::Stop::Refused(format!("{last}: output unconnected")))?;
            if first.is_none() {
                first = Some(last);
            }
        }
        let first = first.expect("at least one buffer");
        d.inst_iterms(&first).into_iter().find(|t| d.iterm_is_input_signal(t, true)).ok_or_else(|| crate::log::Stop::Refused(format!("{first}: no input")))
    }
}

/// `solveDP`'s pick: over weights from 1 up and buffers in order, the chain nearest the target;
/// on equal distance the SHORTER chain, and on equal length the first found. The empty chain
/// (distance = target) is the baseline. `dp[w * n + j]` is a chain length, -1 when unset.
fn pick_best_chain(dp: &[i32], n: usize, max_w: i64, target: i64) -> (i64, i32) {
    let (mut best_w, mut best_j, mut best_dist, mut best_len) = (0i64, -1i32, target, 0i32);
    for w in 1..=max_w {
        for j in 0..n {
            let len = dp[w as usize * n + j];
            if len == -1 {
                continue;
            }
            let dist = (w - target).abs();
            if dist < best_dist || (dist == best_dist && len < best_len) {
                best_w = w;
                best_j = j as i32;
                best_dist = dist;
                best_len = len;
            }
        }
    }
    (best_w, best_j)
}

/// `propagateClock`: through buffers, inverters and combinational cells; through a sequential
/// cell only at a clock-gate clock or latch data input.
fn propagate_clock(d: &dyn TreeDb, input: &ITerm) -> bool {
    let master = d.inst_master(&input.0);
    let Some(cell) = d.liberty_cell(&master) else { return false };
    propagates_through(cell, &input.1)
}

/// The cell side of `propagateClock`.
fn propagates_through(cell: &vyges_sta::liberty::Cell, pin: &str) -> bool {
    if cell.is_inverter() || cell.is_buffer() {
        return true;
    }
    if cell.seqs.is_empty() {
        return true;
    }
    let Some(port) = cell.ports.iter().find(|p| p.name == pin) else { return false };
    let latch_data = cell.seqs.iter().any(|s| !s.is_register && s.data.as_deref().is_some_and(|f| vyges_sta::liberty::identifiers(f).iter().any(|p| p == pin)));
    port.is_clock_gate_clock || latch_data
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (`solveDP`): nearest the target wins; a tie in distance goes to the shorter chain; a
    /// tie in both keeps the first (lower weight, then lower buffer index).
    #[test]
    fn the_nearest_then_shortest_chain_is_picked() {
        // n = 2 buffers, weights 0..=4, target 2: w=1 (j=0, len 1) and w=3 (j=1, len 3) are both
        // 1 away; the shorter (w=1) wins. w=3 j=0 (len 1) ties w=1 in both: the first stays.
        let n = 2;
        let mut dp = vec![-1; 5 * n];
        dp[n] = 1;
        dp[3 * n + 1] = 3;
        dp[3 * n] = 1;
        assert_eq!(pick_best_chain(&dp, n, 4, 2), (1, 0));
        // Nothing nearer than the empty chain: none.
        assert_eq!(pick_best_chain(&vec![-1; 5 * n], n, 4, 2), (0, -1));
    }

    /// Rule (`propagateClock`): a sequential cell passes the clock only at a clock-gate clock pin
    /// (or a latch's data input).
    #[test]
    fn a_gater_passes_the_clock_at_its_clock_gate_pin_only() {
        use vyges_sta::liberty::{Cell, Direction, Port, Seq};
        let port = |name: &str, dir: Direction, gate: bool| Port { name: name.into(), direction: dir, is_clock_gate_clock: gate, ..Port::default() };
        let mut icg = Cell { name: "ICG".into(), ..Cell::default() };
        icg.ports = vec![port("CLK", Direction::Input, true), port("GATE", Direction::Input, false), port("GCLK", Direction::Output, false)];
        icg.seqs = vec![Seq { is_register: false, clock: Some("CLK".into()), data: Some("GATE".into()), clear: None, preset: None, clear_preset_var1: Default::default(), clear_preset_var2: Default::default(), output: Some("IQ".into()), output_inv: None }];
        assert!(propagates_through(&icg, "CLK"));
        assert!(propagates_through(&icg, "GATE"), "a latch's data input propagates too");
        icg.ports[0].is_clock_gate_clock = false;
        assert!(!propagates_through(&icg, "CLK"));
    }
}
