// SPDX-License-Identifier: Apache-2.0
//! Clock tree synthesis of a placed design: the clock buffers chosen, each clock net found and its
//! sinks collected, then (later slices) an H-tree characterized, built and buffered per net.
//!
//! The modules follow the command's call sequence:
//! - [`run`]: the command and `runTritonCts`, a thin sequencer of the stages in their order.
//! - [`options`]: the options the Tcl procs set, in the procs' order.
//! - [`buffers`]: the buffer list (given or inferred), root and sink buffers, the characterization
//!   buffer.
//! - [`discovery`]: the clock nets and their sinks.
//! - [`log`]: the output lines and how a run stops.

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_cts::…` paths resolve here.
extern crate self as vyges_cts;

pub mod buffers;
pub mod clustering;
pub mod discovery;
pub mod htree;
pub mod latency;
pub mod lemon;
pub mod log;
pub mod options;
pub mod run;
pub mod sink_clustering;
pub mod techchar;
pub mod write;

/// The `vyges-cts` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
