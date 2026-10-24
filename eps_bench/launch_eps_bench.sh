#!/usr/bin/env bash
# Launch the EPS parked-bench stack.
#
# Sets EPS_BENCH=1, which:
#   * process_config.py strips controlsd/plannerd/selfdrived/card/radard/modeld/... so
#     nothing but our injector publishes `sendcan` and nothing else commands steer/long.
#   * pandad (child, inherits the env) holds SAFETY_ALLOUTPUT (param 0): relay OPEN,
#     no bus-0<->2 forwarding, steer TX allowed at standstill.
#
# After this is up (engine running, parked, wheels straight, hands clear), open the
# operator UI in a second SSH session:
#     python3 eps_bench/eps_menu.py      # text menu (validated fallback)
# or run the on-screen app per eps_bench/README.md.
#
# PARKED BENCH ONLY. Not driveable. See eps_bench/README.md and SP_HARNESS_DIRECTIVE.md.
set -e
export EPS_BENCH=1
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # repo root
cd "$DIR"
echo "[eps_bench] EPS_BENCH=1 - launching stripped stack (ALLOUTPUT) from $DIR"
exec ./launch_openpilot.sh
