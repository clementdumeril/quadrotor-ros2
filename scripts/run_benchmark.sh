#!/usr/bin/env bash
# Whole benchmark: each controller in its own fresh simulation, every scenario, then the
# recorded NMPC flight.
#   bash scripts/run_benchmark.sh [video file on the Webots host]
#   CONTROLLERS=nmpc bash scripts/run_benchmark.sh      # one controller only
# Takes about 35 min per controller on a 2016 laptop.
set -u
cd "$(dirname "$0")/.."
VIDEO="${1:-}"
STACK="ros2 launch quad_webots|position_controller|nmpc_controller|quad_mission/mission|webots_controller|ros2_supervisor"

stack_running() {
  pgrep -f "$STACK" > /dev/null || tasklist.exe 2>/dev/null | grep -qi webots
}

stop_sim() {
  # SIGTERM, not SIGINT: a job started with & in a script ignores SIGINT, so an earlier
  # version left the previous stack running and two controllers drove the same drone.
  pkill -TERM -f "ros2 launch quad_webots"
  for _ in $(seq 1 60); do stack_running || return 0; sleep 2; done   # Webots may be encoding a video
  pkill -KILL -f "$STACK"
  sleep 2
  if stack_running; then echo "simulation still running, close Webots and retry" >&2; exit 1; fi
}

run_sim() {  # $1 = controller
  if stack_running; then echo "another simulation is running, stop it first" >&2; exit 1; fi
  bash scripts/sim.sh controller:="$1" > "/tmp/sim_$1.log" 2>&1 &
  for _ in $(seq 1 90); do grep -q "Mavic driver ready" "/tmp/sim_$1.log" && break; sleep 2; done
  sleep 3
}

for c in ${CONTROLLERS:-pid nmpc}; do
  run_sim "$c"
  bash scripts/ros.sh python3 scripts/benchmark.py --controller "$c"
  if [ "$c" = nmpc ] && [ -n "$VIDEO" ]; then
    bash scripts/ros.sh python3 scripts/benchmark.py --controller nmpc --scenarios wind --speeds 5 \
      --record "$VIDEO" --out /tmp/showcase
  fi
  stop_sim
done
