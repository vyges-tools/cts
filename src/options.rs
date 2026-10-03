// SPDX-License-Identifier: Apache-2.0
//! `CtsOptions` and the Tcl commands that set it: `set_cts_config` (persistent) and
//! `clock_tree_synthesis`'s own keys, each applied in the order the procs apply them.
//!
//! Only what the modelled stages read is kept; a key a later stage reads is accepted and stored
//! as text, so the run that reaches that stage can refuse it there.

use crate::log::{Log, Res};
use std::collections::BTreeMap;

/// The engine-level state the procs write (`TritonCTS` members and `CtsOptions`).
#[derive(Debug, Clone)]
pub struct Options {
    /// `CtsOptions::bufferList_` and whether it was inferred (`setBufferListInferred`).
    pub buffer_list: Vec<String>,
    pub buffer_list_inferred: bool,
    /// `TritonCTS::rootBuffers_` / `sinkBuffers_` (cleared at the end of each run).
    pub root_buffers: Vec<String>,
    pub sink_buffers: Vec<String>,
    /// `-library` (`isCtsLibrarySet`).
    pub cts_library: Option<String>,
    /// `-sink_buffer_max_cap_derate` (0.01 unless set).
    pub sink_buffer_max_cap_derate: f64,
    pub sink_buffer_max_cap_derate_set: bool,
    /// `-clk_nets`: the names, as given.
    pub clock_nets: Vec<String>,
    /// `-clk_nets` as given (`CtsOptions::getClockNets`).
    pub clock_nets_text: String,
    /// `-skip_nets`.
    pub skip_nets: Vec<String>,
    /// `insertionDelayEnabled` (on unless `-no_insertion_delay`).
    pub insertion_delay: bool,
    /// rsz's `clock_buffer_string_` / `clock_buffer_footprint_` (`set_cts_config`).
    pub clock_buffer_string: Option<String>,
    pub clock_buffer_footprint: Option<String>,
    /// `maxWl_` (DBU): set from the resizer's max wire length when positive; 0 otherwise. Kept as
    /// the result of computing it: only clock-gate cloning (and later slices) reads it, so a
    /// refusal in its computation stops only a run that needs the value.
    pub max_wl: Result<i32, crate::log::Stop>,
    /// Every other key and flag as last given, for the stages that read them.
    pub other: BTreeMap<String, String>,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            buffer_list: Vec::new(),
            buffer_list_inferred: false,
            root_buffers: Vec::new(),
            sink_buffers: Vec::new(),
            cts_library: None,
            sink_buffer_max_cap_derate: 0.01,
            sink_buffer_max_cap_derate_set: false,
            clock_nets: Vec::new(),
            clock_nets_text: String::new(),
            skip_nets: Vec::new(),
            insertion_delay: true,
            clock_buffer_string: None,
            clock_buffer_footprint: None,
            max_wl: Ok(0),
            other: BTreeMap::new(),
        }
    }
}

/// `sta::parse_key_args` then `check_argc_eq0`: exact keys and flags, else the first key, then
/// the first flag, that the argument is a prefix of; STA-0562 for none; STA-0564 for a positional
/// argument.
pub fn parse_key_args(log: &mut Log, cmd: &str, args: &[String], keys: &[&str], flags: &[&str]) -> Res<(BTreeMap<String, String>, Vec<String>)> {
    let (mut k, mut f) = (BTreeMap::new(), Vec::new());
    let mut positional = false;
    // `is_keyword_arg`: a dash, then a letter.
    let keyword = |a: &str| a.len() >= 2 && a.starts_with('-') && a.as_bytes()[1].is_ascii_alphabetic();
    let mut i = 0;
    while i < args.len() {
        let a = args[i].as_str();
        if !keyword(a) {
            positional = true;
            i += 1;
            continue;
        }
        // An exact key, an exact flag, then the FIRST key, else the first flag, it begins.
        let key = keys.iter().find(|x| **x == a).or_else(|| if flags.contains(&a) { None } else { keys.iter().find(|x| x.starts_with(a)) });
        let flag = if key.is_some() { None } else { flags.iter().find(|x| **x == a).or_else(|| flags.iter().find(|x| x.starts_with(a))) };
        if let Some(key) = key {
            let Some(v) = args.get(i + 1) else {
                let id = if keys.contains(&a) { 560 } else { 561 };
                return log.error("STA", id, format!("{cmd} {key} missing value."));
            };
            k.insert(key.to_string(), v.clone());
            i += 2;
        } else if let Some(flag) = flag {
            f.push(flag.to_string());
            i += 1;
        } else {
            return log.error("STA", 562, format!("{cmd} {a} is not a known keyword or flag."));
        }
    }
    // `check_argc_eq0`.
    if positional {
        return log.error("STA", 564, format!("{cmd} positional arguments not supported."));
    }
    Ok((k, f))
}

/// A Tcl list as `std::istream_iterator<std::string>` splits it: on whitespace.
pub fn words(s: &str) -> Vec<String> {
    s.split_whitespace().map(String::from).collect()
}

pub const CTS_KEYS: &[&str] = &[
    "-root_buf", "-buf_list", "-wire_unit", "-clk_nets", "-sink_clustering_size", "-num_static_layers", "-sink_clustering_buffer",
    "-distance_between_buffers", "-branching_point_buffers_distance", "-clustering_exponent", "-clustering_unbalance_ratio",
    "-sink_clustering_max_diameter", "-macro_clustering_size", "-macro_clustering_max_diameter", "-sink_clustering_levels", "-tree_buf",
    "-apply_ndr", "-sink_buffer_max_cap_derate", "-delay_buffer_derate", "-library",
];
pub const CTS_FLAGS: &[&str] = &[
    "-post_cts_disable", "-sink_clustering_enable", "-balance_levels", "-obstruction_aware", "-no_obstruction_aware",
    "-dont_use_dummy_load", "-repair_clock_nets", "-no_insertion_delay",
];
pub const CONFIG_KEYS: &[&str] = &[
    "-apply_ndr", "-branching_point_buffers_distance", "-buf_list", "-clock_buffer_footprint", "-clock_buffer_string",
    "-clustering_exponent", "-clustering_unbalance_ratio", "-delay_buffer_derate", "-distance_between_buffers", "-library",
    "-macro_clustering_max_diameter", "-macro_clustering_size", "-num_static_layers", "-sink_buffer_max_cap_derate",
    "-sink_clustering_levels", "-root_buf", "-sink_clustering_max_diameter", "-sink_clustering_size", "-skip_nets", "-tree_buf",
    "-wire_unit",
];

/// What setting a buffer option reads about the design: whether a master exists.
pub trait Masters {
    fn has_master(&self, name: &str) -> bool;
}

/// `TritonCTS::setRootBuffer`: each name must be a master (CTS-0127).
pub fn set_root_buffer(o: &mut Options, log: &mut Log, m: &dyn Masters, buffers: &str) -> Res<()> {
    let list = words(buffers);
    for b in &list {
        if !m.has_master(b) {
            return log.error("CTS", 127, format!("No physical master cell found for buffer {b}."));
        }
    }
    o.root_buffers = list;
    Ok(())
}

/// `set_cts_config`, in the proc's order. `set_buffer_list` is the caller's (it may infer).
pub fn set_cts_config(o: &mut Options, log: &mut Log, m: &dyn Masters, args: &[String], set_buffer_list: &mut dyn FnMut(&mut Options, &mut Log, &str) -> Res<()>) -> Res<()> {
    let (keys, _) = parse_key_args(log, "set_cts_config", args, CONFIG_KEYS, &[])?;
    let other = |o: &mut Options, k: &str| {
        if let Some(v) = keys.get(k) {
            o.other.insert(k.to_string(), v.clone());
        }
    };
    other(o, "-apply_ndr");
    match (keys.get("-clock_buffer_string"), keys.get("-clock_buffer_footprint")) {
        (Some(_), Some(_)) => return log.error("CTS", 135, "-clock_buffer_string and -clock_buffer_footprint are mutually exclusive."),
        (Some(s), None) => {
            if o.clock_buffer_footprint.is_some() {
                return log.error("CTS", 133, "-clock_buffer_string cannot be set because -clock_buffer_footprint is already defined.");
            }
            // Resizer::setClockBufferString: the string, the footprint cleared.
            o.clock_buffer_string = Some(s.clone());
            o.clock_buffer_footprint = None;
            log.info("RSZ", 205, format!("Clock buffer string set to '{s}'"));
        }
        (None, Some(f)) => {
            if o.clock_buffer_string.is_some() {
                return log.error("CTS", 134, "-clock_buffer_footprint cannot be set because -clock_buffer_string is already defined.");
            }
            o.clock_buffer_footprint = Some(f.clone());
            o.clock_buffer_string = None;
            log.info("RSZ", 206, format!("Clock buffer footprint set to '{f}'"));
        }
        (None, None) => {}
    }
    other(o, "-branching_point_buffers_distance");
    if let Some(b) = keys.get("-buf_list") {
        set_buffer_list(o, log, b)?;
    }
    other(o, "-clustering_exponent");
    other(o, "-clustering_unbalance_ratio");
    if let Some(d) = keys.get("-delay_buffer_derate") {
        if num(d)? < 0.0 {
            return log.error("CTS", 129, "delay_buffer_derate needs to be greater than or equal to 0.");
        }
        other(o, "-delay_buffer_derate");
    }
    other(o, "-distance_between_buffers");
    if let Some(l) = keys.get("-library") {
        o.cts_library = Some(l.clone());
    }
    other(o, "-macro_clustering_max_diameter");
    other(o, "-macro_clustering_size");
    other(o, "-num_static_layers");
    if let Some(r) = keys.get("-root_buf") {
        set_root_buffer(o, log, m, r)?;
    }
    if let Some(d) = keys.get("-sink_buffer_max_cap_derate") {
        let d = num(d)?;
        if !(0.0..=1.0).contains(&d) {
            return log.error("CTS", 130, "sink_buffer_max_cap_derate needs to be between 0 and 1.0.");
        }
        o.sink_buffer_max_cap_derate = d;
        o.sink_buffer_max_cap_derate_set = true;
    }
    other(o, "-sink_clustering_levels");
    other(o, "-sink_clustering_max_diameter");
    other(o, "-sink_clustering_size");
    if let Some(s) = keys.get("-skip_nets") {
        // `get_nets` patterns: plain names only.
        if s.contains(['*', '?', '[', '\\']) {
            return crate::log::refuse(format!("set_cts_config -skip_nets {s}: patterns are not modelled"));
        }
        o.skip_nets.extend(tcl_list(s));
    }
    other(o, "-tree_buf");
    other(o, "-wire_unit");
    Ok(())
}

/// A Tcl list's elements: whitespace-separated words, a `{…}` or `"…"` element taken whole without
/// its delimiters (what `get_nets $list` iterates).
pub fn tcl_list(s: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut chars = s.chars().peekable();
    while let Some(&c) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
            continue;
        }
        let mut w = String::new();
        match c {
            '{' => {
                chars.next();
                let mut depth = 1;
                for ch in chars.by_ref() {
                    match ch {
                        '{' => depth += 1,
                        '}' => {
                            depth -= 1;
                            if depth == 0 {
                                break;
                            }
                        }
                        _ => {}
                    }
                    w.push(ch);
                }
            }
            '"' => {
                chars.next();
                for ch in chars.by_ref() {
                    if ch == '"' {
                        break;
                    }
                    w.push(ch);
                }
            }
            _ => {
                while let Some(&ch) = chars.peek() {
                    if ch.is_whitespace() {
                        break;
                    }
                    w.push(ch);
                    chars.next();
                }
            }
        }
        out.push(w);
    }
    out
}

/// A Tcl number as `expr` compares it.
pub fn num(s: &str) -> Res<f64> {
    s.trim().parse::<f64>().or_else(|_| crate::log::refuse(format!("{s}: not a number")))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (Tcl lists, as `get_nets $keys(-skip_nets)` reads them): quoted and braced elements
    // lose their delimiters.
    #[test]
    fn tcl_lists_unquote() {
        assert_eq!(tcl_list(r#""gclk1" "gclk3""#), ["gclk1", "gclk3"]);
        assert_eq!(tcl_list("{a b} c"), ["a b", "c"]);
    }
}
