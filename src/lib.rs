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

pub mod buffers;
pub mod discovery;
pub mod log;
pub mod options;
pub mod run;
