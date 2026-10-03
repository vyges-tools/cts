# vyges-cts

Clock tree synthesis of a placed design. Today the engine chooses the clock buffers, finds every
clock net and collects its sinks — cloning clock gaters whose loads are spread too far apart — and
reports each step exactly as the command's log reads. Characterization and building the trees
come next; until then a design with a clock net is refused after discovery, never guessed at.

```sh
vyges install loom
vyges loom cts clock_tree_synthesis job.json
vyges loom cts --describe
```

Documentation: [`docs/src/cts.md`](docs/src/cts.md) (an mdBook; `mdbook build docs`), with the
[CLI reference](docs/src/reference/vyges-cts.md).

## The job

The commands a flow script runs before `clock_tree_synthesis`, in order, with their arguments:

```json
{
  "steps": [
    { "cmd": "read_db",         "args": ["placed.odb"] },
    { "cmd": "read_liberty",    "args": ["cells.lib"] },
    { "cmd": "set_wire_rc",     "args": ["-clock", "-layer", "metal3"] },
    { "cmd": "read_sdc",        "args": ["constraints.sdc"] },
    { "cmd": "set_cts_config",  "args": ["-buf_list", "CLKBUF_X3"] },
    { "cmd": "clock_tree_synthesis", "args": [] }
  ]
}
```

The report carries the command's own lines, in order (`[INFO CTS-0050] Root buffer is …`), a
`status`, and, when clock nets were found, each clock tree to build — its net, clock, sink count
and parent:

```json
{ "tool": "vyges-cts", "status": "refused", "modelled_through": "discovery",
  "reason": "characterization and clock tree building are not modelled yet",
  "clock_nets": 1, "lines": [ "[INFO CTS-0050] Root buffer is CLKBUF_X3.", … ] }
```

## What is modelled

- the buffer list, given or inferred from the libraries (clock-cell attribute, LEF clock pins,
  a configured string or footprint, the CLKBUF and BUF name patterns);
- the root, sink and characterization buffers, by max capacitance against the wire across the core;
- the clock network through gaters, buffers and bidirect pads, each clock's root nets, each net's
  sinks, macro and register trees split behind a new buffer;
- clock-gate cloning: a gater whose loads cluster apart beyond the maximum wire length is cloned
  per cluster.

Refused rather than guessed: hierarchical designs where a clone or register-tree buffer is made,
insertion-delay cells, several corners, disabled timing arcs, case analysis, generated clocks.

## Build

The command-line engine reads the design database through `vyges-opendb`, which builds the
database library from source. Check out `vyges-tools/opendb`, `opendb-lib`, `sta`, `grt`, `est`,
`stt`, `loom`, `layout` and `rsz` beside this repository (see `.github/workflows/ci.yml`), then:

```sh
cargo build --release --features cli
cargo test  --release --features cli
```

Without `--features cli` the library builds with no C++ dependency, and the pure rules are tested
on their own.

Licensed under Apache-2.0.
