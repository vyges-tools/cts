# vyges loom cts — CLI reference

_Generated from `vyges loom cts --help` — this page is the tool's own output, verbatim._

```text
vyges loom cts — clock tree synthesis of a placed design: the clock buffers chosen, each clock net
found and its sinks collected (the trees themselves are not built yet)

USAGE:
  vyges loom cts clock_tree_synthesis <job.json> [-o FILE]
  vyges loom cts --describe
  vyges loom cts --help
  vyges loom cts --version

JOB FIELDS:
  steps        required — the commands in order, each {"cmd": ..., "args": [...]}, as a flow
               script passes them: read_db, read_liberty, set_wire_rc, set_layer_rc,
               set_dont_use, set_debug_level, read_sdc, set_cts_config, then
               clock_tree_synthesis [options]

MODELLED:
  the buffer list (given, or inferred from the libraries), the root, sink and characterization
  buffers, clock-gate cloning, every clock net and its sinks (CTS-0049..0052, CTS-0007..0011,
  CTS-0008, CTS-0082) — the report's lines are the command's own, in its order.
  Not yet: characterization, the trees, dummy loads, latency balancing — a design with a clock
  net is refused after discovery (modelled_through: discovery).

OPTIONS:
  -o FILE               write the JSON report to FILE instead of stdout
  --json                accepted; the report is JSON either way
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)
  --sponsor             sponsor Vyges (github.com/sponsors/vyges-ip)
  --star                star this tool on GitHub

EXIT STATUS:
  0  up_to_date   clock nets were looked for and none was found: nothing to build
  2  vacuous      the job has no clock_tree_synthesis step. NOT a pass.
  2  error        usage, unreadable input, or an error the command raises (its line is reported)
  3  refused      an input this engine does not model — or a clock tree to build — see `reason`
```
