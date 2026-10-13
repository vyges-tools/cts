# vyges-cts — clock tree synthesis

> **Physical construction — opt-in, and not part of `vyges install loom`.** Install the group with
> `vyges install physical` (or just this engine with `vyges install cts`), then run
> `vyges physical cts`. It is also a standalone `vyges-cts` binary on your PATH, which is what the
> dispatch calls and what a flow author targets directly.

`vyges-cts` answers: **which clock nets does this placed design have, which sinks hang on each,
and which buffers will build their trees?** It reads the design database, the libraries, the wire
RC and the constraints, then runs the command's own sequence, step by step, printing each line the
command prints.

What the command does, in the order it is decided:

| stage | decides |
| --- | --- |
| **the buffer list** | the buffers given, or inferred from the libraries: cells marked as clock cells, else cells with a LEF clock input, else a configured name or footprint match, else `CLKBUF`, else `BUF` |
| **root and sink buffers** | the smallest-area buffer whose max capacitance (derated) covers the wire across the core — half of it for the root — else the one with the largest max capacitance |
| **the characterization buffer** | the list trimmed to the root–sink capacitance range and sorted; the last with the largest max capacitance |
| **clock-gate cloning** | a gater whose loads, joined by a Steiner tree, fall into clusters farther apart than the maximum wire length is cloned per cluster, each clone placed at its loads' centre |
| **clock nets** | each clock's root nets; the network followed through gaters, buffers and bidirect pads; each net's sinks — macros and registers split behind a new buffer when both are present |

**Not yet:** the characterization tables, the H-tree, dummy loads and latency balancing. A design
with a clock net is therefore refused after discovery (`modelled_through: discovery`) — the
report holds every line up to that point, and the tree list it would build.

## Status and exit codes

| status | exit | meaning |
| --- | --- | --- |
| `up_to_date` | 0 | clock nets were looked for and none was found: nothing to build |
| `refused` | 3 | an input this engine does not model, or a clock tree to build (`reason`) |
| `vacuous` | 2 | the job has no `clock_tree_synthesis` step — NOT a pass |
| `error` | 2 | an error the command raises; its line is the last in `lines` |
