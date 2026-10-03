// SPDX-License-Identifier: Apache-2.0
//! The clock buffers: the list (given, or inferred by the resizer), the root and sink buffers
//! chosen from it, the list trimmed and sorted for characterization, and the characterization
//! buffer.
//!
//! One function per reference function, in call order:
//! - [`infer_clock_buffer_list`] — `Resizer::inferClockBufferList` (with `isClockCellCandidate`),
//!   reached from `setBufferList` when the list is empty;
//! - [`select_root_buffer`] / [`select_sink_buffer`] — `selectRootBuffer` /
//!   `selectSinkBuffer`, each through [`select_best_max_cap_buffer`];
//! - [`finalize_root_sink_buffers`] — `TechChar::finalizeRootSinkBuffers`;
//! - [`trim_sort_buffer_list`] — `TechChar::trimSortBufferList`;
//! - [`characterization_buffer`] — the buffer loop of `TechChar::initCharacterization`.

use crate::log::{fmt_e, fmt_f32, fmt_f64, Log, Res};
use crate::options::Options;
use vyges_sta::liberty::{Cell, Library};

/// What buffer selection reads about the design and its libraries.
pub trait BufferLibs {
    /// The libraries, in read order.
    fn libraries(&self) -> &[Library];
    /// `db_->findMaster(name)` exists.
    fn has_master(&self, name: &str) -> bool;
    /// The liberty cell a master links to (`network_->libertyCell(dbToSta(master))`).
    fn liberty_cell(&self, name: &str) -> Option<&Cell>;
    /// The master has an INPUT terminal with LEF `USE CLOCK`; the first such terminal's name.
    fn lef_clock_input(&self, master: &str) -> Option<String>;
    /// The resizer's `dont_use_` (`set_dont_use`, with the liberty `dont_use` cells).
    fn resizer_dont_use(&self, cell: &str) -> bool;
}

/// `containsIgnoreCase`.
fn contains_ignore_case(s: &str, sub: &str) -> bool {
    s.to_ascii_lowercase().contains(&sub.to_ascii_lowercase())
}

/// `Resizer::isClockCellCandidate`.
fn is_clock_cell_candidate(cell: &Cell, libs: &dyn BufferLibs) -> bool {
    !cell.dont_use && !libs.resizer_dont_use(&cell.name) && !cell.always_on && !cell.is_isolation_cell && !cell.is_level_shifter
}

/// `LibertyLibrary::buffers()`: the library's cells by name (its cell map's order), each a
/// buffer and not `dont_use`.
fn library_buffers(lib: &Library) -> impl Iterator<Item = &Cell> {
    lib.cells.values().filter(|c| !c.dont_use && c.is_buffer())
}

/// `Resizer::inferClockBufferList(lib_name, buffers)`: every candidate buffer of every library
/// (or the named one), then the first non-empty of — the `is_clock_cell` ones, the ones with a
/// LEF clock input, the user's string or footprint matches, `CLKBUF` matches, `BUF` matches — else
/// every candidate (RSZ-0110 when there is none).
pub fn infer_clock_buffer_list(o: &Options, log: &mut Log, libs: &dyn BufferLibs) -> Res<Vec<String>> {
    let (mut attr, mut lef, mut user, mut clkbuf, mut buf, mut all) = (vec![], vec![], vec![], vec![], vec![], vec![]);
    for lib in libs.libraries() {
        if o.cts_library.as_ref().is_some_and(|n| *n != lib.name) {
            continue;
        }
        for c in library_buffers(lib) {
            if !is_clock_cell_candidate(c, libs) {
                continue;
            }
            all.push(c);
            if c.is_clock_cell {
                attr.push(c);
            }
            if libs.lef_clock_input(&c.name).is_some() {
                lef.push(c);
            }
            if let Some(s) = &o.clock_buffer_string {
                if contains_ignore_case(&c.name, s) {
                    user.push(c);
                }
            } else if let Some(f) = &o.clock_buffer_footprint {
                if contains_ignore_case(&c.footprint, f) {
                    user.push(c);
                }
            }
            if contains_ignore_case(&c.name, "CLKBUF") {
                clkbuf.push(c);
            }
            if contains_ignore_case(&c.name, "BUF") {
                buf.push(c);
            }
        }
    }
    const G: &str = "inferClockBufferList";
    let selected = if !attr.is_empty() {
        for c in &attr {
            log.debug("RSZ", G, 1, || format!("{} has clock cell attribute", c.name));
        }
        attr
    } else if !lef.is_empty() {
        for c in &lef {
            let term = libs.lef_clock_input(&c.name).unwrap_or_default();
            log.debug("RSZ", G, 1, || format!("{} has input port {term} with LEF USE as CLOCK", c.name));
        }
        lef
    } else if !user.is_empty() {
        for c in &user {
            if let Some(s) = &o.clock_buffer_string {
                log.debug("RSZ", G, 1, || format!("{} found by user-configured string '{s}'", c.name));
            } else if let Some(f) = &o.clock_buffer_footprint {
                log.debug("RSZ", G, 1, || format!("{} found by user-configured footprint '{f}'", c.name));
            }
        }
        user
    } else if !clkbuf.is_empty() {
        for c in &clkbuf {
            log.debug("RSZ", G, 1, || format!("{} found by 'CLKBUF' pattern match", c.name));
        }
        clkbuf
    } else if !buf.is_empty() {
        for c in &buf {
            log.debug("RSZ", G, 1, || format!("{} found by 'BUF' pattern match", c.name));
        }
        buf
    } else {
        log.debug("RSZ", G, 1, || "No buffers with clock attributes or name patterns found, using all buffers".into());
        if all.is_empty() {
            return log.error("RSZ", 110, "No clock buffer candidates could be found from any libraries.");
        }
        all
    };
    let mut names = Vec::new();
    for c in selected {
        names.push(c.name.clone());
        log.debug("RSZ", G, 1, || format!("{} has been inferred as clock buffer", c.name));
    }
    Ok(names)
}

/// `setBufferList(buffers)`: the given names (each a master, CTS-0126), or — given
/// none — the inferred list, marked inferred.
pub fn set_buffer_list(o: &mut Options, log: &mut Log, libs: &dyn BufferLibs, buffers: &str) -> Res<()> {
    let list = crate::options::words(buffers);
    if list.is_empty() {
        o.buffer_list = infer_clock_buffer_list(o, log, libs)?;
        o.buffer_list_inferred = true;
    } else {
        for b in &list {
            if !libs.has_master(b) {
                return log.error("CTS", 126, format!("No physical master cell found for buffer {b}."));
            }
        }
        // The resizer's clock buffers (`setClockBuffersList`) are read by later stages only.
        o.buffer_list = list;
    }
    Ok(())
}

/// A buffer's output port max capacitance (`capacitanceLimit(max)`): `None` when unset.
fn buffer_max_cap(cell: &Cell) -> Option<f32> {
    cell.buffer_ports().and_then(|(_, out)| out.max_capacitance)
}

/// `selectBestMaxCapBuffer(buffers, totalCap)`: the smallest-area buffer whose max cap × derate
/// exceeds the load, else the one with the largest max cap (the first of equals each way).
pub fn select_best_max_cap_buffer(o: &Options, log: &mut Log, libs: &dyn BufferLibs, buffers: &[String], total_cap: f32) -> Res<String> {
    let (mut best, mut next_best) = (String::new(), String::new());
    let mut best_area = f32::MAX;
    let mut best_cap = 0.0f32;
    let derate = o.sink_buffer_max_cap_derate;
    for name in buffers {
        if !libs.has_master(name) {
            return log.error("CTS", 117, format!("Physical master could not be found for cell '{name}'"));
        }
        let Some(cell) = libs.liberty_cell(name) else {
            return log.error("CTS", 112, format!("Liberty cell could not be found for cell '{name}'"));
        };
        let area = cell.area;
        let limit = buffer_max_cap(cell);
        let max_cap = limit.unwrap_or(0.0);
        log.debug("CTS", "buffering", 1, || {
            format!("{name} has cap limit:{} vs. total cap:{}, derate:{}", fmt_f32(max_cap * derate as f32), fmt_f32(total_cap), fmt_f64(derate))
        });
        // `maxCap * derate` is a double; the load widens to compare.
        if limit.is_some() && f64::from(max_cap) * derate > f64::from(total_cap) && area < best_area {
            best = name.clone();
            best_area = area;
        }
        if max_cap > best_cap {
            next_best = name.clone();
            best_cap = max_cap;
        }
    }
    if next_best.is_empty() {
        return log.error("CTS", 2, "Characterization could not select a buffer: candidate buffers lack max capacitance information.");
    }
    Ok(if best.is_empty() { next_best } else { best })
}

/// The wire across the core's longer side, in microns, as both selections compute it: the DBU
/// length as a `float` over the integer DBU per micron.
fn core_wire_length(core: (i32, i32), dbu: i32) -> f32 {
    core.0.max(core.1) as f32 / dbu as f32
}

/// `selectRootBuffer`: `-root_buf`, else the buffer list; one buffer is taken as is; else the
/// best buffer for half the core-spanning wire's signal capacitance (marked inferred).
/// `wire_signal_cap` is `wireSignalCapacitance` (farads per meter).
pub fn select_root_buffer(o: &mut Options, log: &mut Log, libs: &dyn BufferLibs, core: (i32, i32), dbu: i32, wire_signal_cap: f64) -> Res<String> {
    if o.root_buffers.is_empty() {
        o.root_buffers = o.buffer_list.clone();
    }
    if o.root_buffers.len() == 1 {
        return Ok(o.root_buffers[0].clone());
    }
    let len = core_wire_length(core, dbu);
    let cap = (wire_signal_cap * 1e-6 * f64::from(len) / 2.0) as f32;
    let buffers = o.root_buffers.clone();
    select_best_max_cap_buffer(o, log, libs, &buffers, cap)
}

/// `selectSinkBuffer`: as [`select_root_buffer`] for the sink buffer, over the whole wire.
pub fn select_sink_buffer(o: &mut Options, log: &mut Log, libs: &dyn BufferLibs, core: (i32, i32), dbu: i32, wire_signal_cap: f64) -> Res<String> {
    if o.sink_buffers.is_empty() {
        o.sink_buffers = o.buffer_list.clone();
    }
    if o.sink_buffers.len() == 1 {
        return Ok(o.sink_buffers[0].clone());
    }
    let len = core_wire_length(core, dbu);
    let cap = (wire_signal_cap * 1e-6 * f64::from(len)) as f32;
    let buffers = o.sink_buffers.clone();
    let sink = select_best_max_cap_buffer(o, log, libs, &buffers, cap)?;
    log.debug("CTS", "buffering", 1, || format!("{sink} has been selected as sink buffer to drive sink wire cap of {}", fmt_e(f64::from(cap), 2)));
    Ok(sink)
}

/// `TechChar::finalizeRootSinkBuffers`.
pub fn finalize_root_sink_buffers(o: &Options, log: &mut Log, root: &str, sink: &str) {
    log.info("CTS", 50, format!("Root buffer is {root}."));
    log.info("CTS", 51, format!("Sink buffer is {sink}."));
    if o.sink_buffer_max_cap_derate_set {
        log.info("CTS", 53, format!("Max cap limit derate of {:.3} was used to infer root or sink buffers.", o.sink_buffer_max_cap_derate));
    }
}

/// `TechChar::getMaxCapLimit`: the buffer's output max cap, 0 when unset.
fn max_cap_limit(libs: &dyn BufferLibs, buf: &str) -> f32 {
    libs.liberty_cell(buf).and_then(buffer_max_cap).unwrap_or(0.0)
}

/// `TechChar::trimSortBufferList`: every buffer a master (CTS-0074) with a liberty cell
/// (CTS-0106); an inferred list keeps only max caps between the root's and the sink's; sorted by
/// max cap; an inferred list then drops a buffer within 10% of the last kept one's; CTS-0052.
pub fn trim_sort_buffer_list(o: &Options, log: &mut Log, libs: &dyn BufferLibs, root: &str, sink: &str, mut buffers: Vec<String>) -> Res<Vec<String>> {
    for b in &buffers {
        if !libs.has_master(b) {
            return log.error("CTS", 74, format!("No physical master cell found for buffer {b}."));
        }
        if libs.liberty_cell(b).is_none() {
            return log.error("CTS", 106, format!("No liberty cell found for buffer {b}."));
        }
    }
    if o.buffer_list_inferred {
        let (sink_cap, root_cap) = (max_cap_limit(libs, sink), max_cap_limit(libs, root));
        let (low, high) = if root_cap <= sink_cap { (root_cap, sink_cap) } else { (sink_cap, root_cap) };
        buffers.retain(|b| {
            let cap = max_cap_limit(libs, b);
            let keep = !(cap < low || cap > high);
            if !keep {
                log.debug("CTS", "buffering", 1, || format!("  removing {b} outside of low/high max cap"));
            }
            keep
        });
    }
    // `std::ranges::sort` is not stable; below 17 elements it is an insertion sort, which is.
    if buffers.len() > 16 {
        let caps: Vec<f32> = buffers.iter().map(|b| max_cap_limit(libs, b)).collect();
        let mut sorted = caps.clone();
        sorted.sort_by(f32::total_cmp);
        if sorted.windows(2).any(|w| w[0] == w[1]) {
            return crate::log::refuse("more than 16 clock buffers with equal max caps: the introsort's tie order is not modelled");
        }
    }
    buffers.sort_by(|a, b| max_cap_limit(libs, a).total_cmp(&max_cap_limit(libs, b)));
    if o.buffer_list_inferred && !buffers.is_empty() {
        let mut prev = max_cap_limit(libs, &buffers[0]);
        let mut kept = vec![buffers[0].clone()];
        for b in &buffers[1..] {
            let curr = max_cap_limit(libs, b);
            // A float ratio against the double 0.1.
            if f64::from((prev - curr).abs() / curr) < 0.1 {
                log.debug("CTS", "buffering", 1, || format!("  removing {b} within 10% of prev neighbor's max cap"));
            } else {
                kept.push(b.clone());
                prev = curr;
            }
        }
        buffers = kept;
    }
    log.info("CTS", 52, "The following clock buffers will be used for CTS:");
    for b in &buffers {
        log.report(format!("                    {b}"));
    }
    Ok(buffers)
}

/// `TechChar::collectSlewsLoadsFromTableAxis`: from the first input→output arc set, each arc's
/// delay table axes — input transitions to `slews`, output capacitances to `loads` — appended, so
/// the lists grow buffer by buffer; at `tech char` debug level 2, both as they stand.
fn collect_slews_loads_from_table_axis(log: &mut Log, cell: &Cell, slews: &mut Vec<f32>, loads: &mut Vec<f32>) {
    use vyges_sta::liberty::Model;
    use vyges_sta::table::AxisVar;
    if let Some((input, output)) = cell.buffer_ports() {
        if let Some(set) = cell.arc_sets.iter().find(|a| a.from == input.name && a.to == output.name) {
            for arc in &set.arcs {
                let Model::Gate(g) = &arc.model else { continue };
                let Some(t) = &g.delay else { continue };
                let (mut s, mut l) = (None, None);
                for axis in t.axes.iter().take(2) {
                    match axis.var {
                        AxisVar::InputNetTransition => s = Some(&axis.values),
                        AxisVar::TotalOutputNetCapacitance => l = Some(&axis.values),
                        _ => {}
                    }
                }
                slews.extend(s.into_iter().flatten());
                loads.extend(l.into_iter().flatten());
            }
        }
    }
    if log.debug_on("CTS", "tech char", 2) {
        log.report(format!("axis slews at {}", cell.name));
        for v in slews.iter() {
            log.report(format!("{} ", fmt_e(f64::from(*v), 3)));
        }
        log.report(format!("axis loads at {}", cell.name));
        for v in loads.iter() {
            log.report(format!("{} ", fmt_e(f64::from(*v), 3)));
        }
    }
}

/// The characterization buffer (`charBuf_`): over the sorted list, the last one whose max cap is
/// at least the largest so far (CTS-0111 for a buffer without one: the library default is never
/// read), each buffer's table axes collected; CTS-0049. Returns it and the axes.
pub fn characterization_buffer(log: &mut Log, libs: &dyn BufferLibs, buffers: &[String]) -> Res<(String, Vec<f32>, Vec<f32>)> {
    let mut max_buff_cap = 0.0f32;
    let mut name = String::new();
    let (mut slews, mut loads) = (Vec::new(), Vec::new());
    for b in buffers {
        let Some(cell) = libs.liberty_cell(b) else { return log.error("CTS", 106, format!("No liberty cell found for buffer {b}.")) };
        let Some(cap) = buffer_max_cap(cell) else {
            return log.error("CTS", 111, format!("No max capacitance found for cell {b}."));
        };
        if cap >= max_buff_cap {
            max_buff_cap = cap;
            name = b.clone();
        }
        collect_slews_loads_from_table_axis(log, cell, &mut slews, &mut loads);
    }
    if name.is_empty() {
        return log.error("CTS", 113, "Characterization buffer is not defined.\n    Check that -buf_list has supported buffers from platform.");
    }
    log.info("CTS", 49, format!("Characterization buffer is {name}."));
    Ok((name, slews, loads))
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::liberty_parse::parse;

    /// Buffers named by (name, area, max_capacitance in pF or none, is_clock_cell).
    fn lib(cells: &[(&str, f32, Option<f32>, bool)]) -> Library {
        let mut t = String::from("library (l) { time_unit : \"1ns\"; capacitive_load_unit (1, pf);\n");
        for (n, area, cap, clk) in cells {
            let cap = cap.map(|c| format!("max_capacitance : {c};")).unwrap_or_default();
            t += &format!(
                "cell ({n}) {{ area : {area}; is_clock_cell : {clk}; pin (A) {{ direction : input; capacitance : 0.001; }}
                 pin (Z) {{ direction : output; function : \"A\"; {cap} }} }}\n"
            );
        }
        t += "}";
        Library::read(&parse(&t).unwrap()).unwrap()
    }

    struct Libs(Vec<Library>, Vec<&'static str>);

    impl BufferLibs for Libs {
        fn libraries(&self) -> &[Library] {
            &self.0
        }
        fn has_master(&self, name: &str) -> bool {
            self.liberty_cell(name).is_some()
        }
        fn liberty_cell(&self, name: &str) -> Option<&Cell> {
            self.0.iter().find_map(|l| l.cells.get(name))
        }
        fn lef_clock_input(&self, _: &str) -> Option<String> {
            None
        }
        fn resizer_dont_use(&self, cell: &str) -> bool {
            self.1.contains(&cell)
        }
    }

    // Rule (inferClockBufferList): the first non-empty of is_clock_cell, LEF clock input, user
    // string, CLKBUF, BUF, else every candidate; a resizer dont_use cell is never a candidate;
    // the library's cells in NAME order.
    #[test]
    fn inference_takes_the_first_non_empty_class() {
        let l = Libs(vec![lib(&[("BUF_X2", 1.0, Some(0.1), false), ("CLKBUF_X1", 1.0, Some(0.1), false), ("CLKBUF_X3", 2.0, Some(0.2), false)])], vec![]);
        let mut log = Log::default();
        assert_eq!(infer_clock_buffer_list(&Options::default(), &mut log, &l).unwrap(), ["CLKBUF_X1", "CLKBUF_X3"]);
        let l = Libs(vec![lib(&[("BUF_X2", 1.0, Some(0.1), true), ("CLKBUF_X1", 1.0, Some(0.1), false)])], vec![]);
        assert_eq!(infer_clock_buffer_list(&Options::default(), &mut log, &l).unwrap(), ["BUF_X2"], "is_clock_cell wins");
        let l = Libs(vec![lib(&[("BUF_X2", 1.0, Some(0.1), false), ("CLKBUF_X1", 1.0, Some(0.1), false)])], vec!["CLKBUF_X1"]);
        assert_eq!(infer_clock_buffer_list(&Options::default(), &mut log, &l).unwrap(), ["BUF_X2"], "dont_use drops CLKBUF");
    }

    // Rule (selectBestMaxCapBuffer): the smallest area with max cap × derate (a double) above the
    // load; none — the largest max cap, the first of equals.
    #[test]
    fn best_max_cap_buffer() {
        let l = Libs(vec![lib(&[("A", 3.0, Some(0.5), false), ("B", 1.0, Some(0.2), false), ("C", 2.0, Some(0.5), false)])], vec![]);
        let o = Options::default();
        let mut log = Log::default();
        let names: Vec<String> = ["A", "B", "C"].map(String::from).to_vec();
        // derate 0.01: B covers 0.2 pF × 0.01 = 2 fF, A and C 5 fF.
        assert_eq!(select_best_max_cap_buffer(&o, &mut log, &l, &names, 1.5e-15).unwrap(), "B");
        assert_eq!(select_best_max_cap_buffer(&o, &mut log, &l, &names, 3e-15).unwrap(), "C", "the smaller area of the two that cover it");
        assert_eq!(select_best_max_cap_buffer(&o, &mut log, &l, &names, 1e-12).unwrap(), "A", "none covers: the first largest max cap");
    }

    // Rule (trimSortBufferList): an inferred list keeps max caps within [root, sink], sorts by max
    // cap, then drops one within 10% of the last KEPT; a given list is only sorted.
    #[test]
    fn inferred_lists_are_trimmed_given_lists_only_sorted() {
        let l = Libs(vec![lib(&[("A", 1.0, Some(0.10), false), ("B", 1.0, Some(0.105), false), ("C", 1.0, Some(0.2), false), ("D", 1.0, Some(0.4), false)])], vec![]);
        let names: Vec<String> = ["D", "C", "B", "A"].map(String::from).to_vec();
        let mut o = Options { buffer_list_inferred: true, ..Options::default() };
        let mut log = Log::default();
        assert_eq!(trim_sort_buffer_list(&o, &mut log, &l, "A", "C", names.clone()).unwrap(), ["A", "C"]);
        o.buffer_list_inferred = false;
        assert_eq!(trim_sort_buffer_list(&o, &mut log, &l, "A", "C", names).unwrap(), ["A", "B", "C", "D"]);
    }

    // Rule (initCharacterization): the characterization buffer is the LAST with the largest max
    // cap (`>=`); a buffer with no max cap is CTS-0111.
    #[test]
    fn the_characterization_buffer_is_the_last_largest() {
        let l = Libs(vec![lib(&[("A", 1.0, Some(0.2), false), ("B", 1.0, Some(0.2), false), ("N", 1.0, None, false)])], vec![]);
        let mut log = Log::default();
        assert_eq!(characterization_buffer(&mut log, &l, &["A".into(), "B".into()]).unwrap().0, "B");
        assert_eq!(characterization_buffer(&mut log, &l, &["N".into()]).unwrap_err(), crate::log::Stop::Error { tool: "CTS", id: 111 });
    }
}
