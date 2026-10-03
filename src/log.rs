// SPDX-License-Identifier: Apache-2.0
//! The run's output, line by line as the reference's logger writes it, and how a run stops.

use std::collections::BTreeMap;

/// The lines a run printed and the debug levels in force.
#[derive(Debug, Default)]
pub struct Log {
    pub lines: Vec<String>,
    /// `set_debug_level <tool> <group> <level>`.
    pub debug: BTreeMap<(String, String), i32>,
}

/// Why a run ended early: the reference's own error (its line is in the log), or a behaviour this
/// engine does not model (no line; the run is refused, never guessed).
#[derive(Debug, Clone, PartialEq)]
pub enum Stop {
    Error { tool: &'static str, id: u32 },
    Refused(String),
}

pub type Res<T> = Result<T, Stop>;

/// A refusal.
pub fn refuse<T>(why: impl Into<String>) -> Res<T> {
    Err(Stop::Refused(why.into()))
}

impl Log {
    fn push(&mut self, text: String) {
        // A message with newlines is printed as it is: one output line per text line.
        self.lines.extend(text.split('\n').map(String::from));
    }

    pub fn info(&mut self, tool: &str, id: u32, msg: impl AsRef<str>) {
        self.push(format!("[INFO {tool}-{id:04}] {}", msg.as_ref()));
    }

    pub fn warn(&mut self, tool: &str, id: u32, msg: impl AsRef<str>) {
        self.push(format!("[WARNING {tool}-{id:04}] {}", msg.as_ref()));
    }

    /// `logger_->error`: the line, then the run stops.
    pub fn error<T>(&mut self, tool: &'static str, id: u32, msg: impl AsRef<str>) -> Res<T> {
        self.push(format!("[ERROR {tool}-{id:04}] {}", msg.as_ref()));
        Err(Stop::Error { tool, id })
    }

    /// `logger_->report`: the text alone.
    pub fn report(&mut self, msg: impl AsRef<str>) {
        self.push(msg.as_ref().to_string());
    }

    /// `debugPrint`'s check: the group's level is at least `level`.
    pub fn debug_on(&self, tool: &str, group: &str, level: i32) -> bool {
        self.debug.get(&(tool.to_string(), group.to_string())).is_some_and(|&l| l >= level)
    }

    pub fn debug(&mut self, tool: &str, group: &str, level: i32, msg: impl FnOnce() -> String) {
        if self.debug_on(tool, group, level) {
            let m = msg();
            self.push(format!("[DEBUG {tool}-{group}] {m}"));
        }
    }
}

/// `fmt`'s `{}` for a float, from the value's own shortest digits (`fixed`: `{}`, `sci`: `{:e}`):
/// fixed for decimal exponents in [-4, 16), a whole number without a fraction; else `d.ddde±XX`.
fn fmt_g(finite_nonzero: bool, fixed: String, sci: String) -> String {
    if !finite_nonzero {
        return match fixed.as_str() {
            "NaN" => "nan".into(),
            "inf" | "-inf" => fixed,
            _ => fixed.trim_end_matches(".0").to_string(),
        };
    }
    let (mantissa, exp) = sci.split_once('e').expect("exponent form");
    let exp: i32 = exp.parse().expect("an exponent");
    if (-4..16).contains(&exp) {
        fixed.strip_suffix(".0").map(String::from).unwrap_or(fixed)
    } else {
        format!("{mantissa}e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs())
    }
}

/// `fmt`'s `{}` for a `float`.
pub fn fmt_f32(v: f32) -> String {
    fmt_g(v.is_finite() && v != 0.0, format!("{v}"), format!("{v:e}"))
}

/// `fmt`'s `{}` for a `double`.
pub fn fmt_f64(v: f64) -> String {
    fmt_g(v.is_finite() && v != 0.0, format!("{v}"), format!("{v:e}"))
}

/// `fmt`'s `{:0.Ne}`: N fraction digits, a signed exponent of at least two digits.
pub fn fmt_e(v: f64, digits: usize) -> String {
    let s = format!("{v:.digits$e}");
    let (mantissa, exp) = s.split_once('e').expect("exponent form");
    let exp: i32 = exp.parse().expect("an exponent");
    format!("{mantissa}e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (fmt `{}`): shortest round-trip digits; fixed for exponents in [-4, 16), whole numbers
    // without a fraction; else scientific with a two-digit signed exponent.
    #[test]
    fn floats_print_as_fmt_does() {
        assert_eq!(fmt_f64(0.01), "0.01");
        assert_eq!(fmt_f64(1.0), "1");
        assert_eq!(fmt_f32(1.5e-14), "1.5e-14");
        assert_eq!(fmt_f32(0.1), "0.1");
        assert_eq!(fmt_f64(2.5e-5), "2.5e-05");
        assert_eq!(fmt_e(1.234e-14, 2), "1.23e-14");
        assert_eq!(fmt_e(12345.0, 2), "1.23e+04");
    }

    // Rule (logger): a message with newlines is several output lines; an error stops the run.
    #[test]
    fn lines_split_and_errors_stop() {
        let mut l = Log::default();
        l.warn("CTS", 104, "a\nb");
        assert_eq!(l.lines, ["[WARNING CTS-0104] a", "b"]);
        let r: Res<()> = l.error("CTS", 73, "x");
        assert_eq!(r, Err(Stop::Error { tool: "CTS", id: 73 }));
    }
}
