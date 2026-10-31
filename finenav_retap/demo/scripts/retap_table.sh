#!/bin/bash
# ==============================================================================
#  Reproduce the resolution / N table through the existing rosbag launch.
# ==============================================================================
#  One row of config/README.md's table per run: replay the whole bag through
#  retap_rosbag.launch.py, interrupt the launch once playback ends, and read
#  the map node's shutdown line:
#
#    RetapTerrainMap: final <N> scans (processed ..., dropped ...);
#      map <occupied> occupied, <drivable> drivable (x%, y m2 at r m);
#      <t> ms/scan over <N> scans
#
#  Every number in the table (occupied / drivable / ratio / area / ms per scan)
#  comes from that one line, so the table and this script cannot disagree.
#
#  EQUAL INPUT IS PART OF THE PROTOCOL.  All 1220 clouds in the bag must reach
#  the map in every row, otherwise "drivable area at 0.05 m" is compared against
#  "drivable area at 0.2 m computed from 27% less data".  The map consumes a
#  cloud only as fast as it can analyse it (measured on this laptop: ~22/s at
#  0.20 m, ~4.9/s at 0.10 m, ~1.45/s at 0.05 m with ~390k voxels), and the
#  observation buffer (max_buffer_size 2) silently discards what arrives
#  faster.  Each row therefore plays the bag at <= ~80% of the measured
#  capacity, and the script checks `processed` against the cloud count in the
#  bag and shouts if the row was truncated.
#
#  Usage (from the workspace root, after `source install/setup.bash`):
#
#    demo/scripts/retap_table.sh                       # the default rows
#    demo/scripts/retap_table.sh terrain_res005_ring49_n3.yaml:0.12
#    demo/scripts/retap_table.sh terrain_res005_ring49_n3.yaml:0.12:1100
#    demo/scripts/retap_table.sh --list
#
#  Each row is config:bag_rate[:timeout_s].  The timeout defaults to
#  bag_duration / bag_rate + 60 s, so it follows the rate automatically; only
#  pass one when the bag directory cannot be found.
#
#  bag_path:=/path/to/bag can be appended to any call; by default the launch
#  finds the single rosbag2_* directory in the workspace root.
# ==============================================================================
set +u

ROWS=(
  "terrain_res020_ring09.yaml:1.0"                    # 0.20 m, ring 9  (0.30 m half-span)
  "terrain_res010_ring25.yaml:0.4"                    # 0.10 m, ring 25 (0.35 m half-span)
  "terrain_res005_ring49.yaml:0.12"                   # 0.05 m, ring 49 (0.325 m), N = 0
  "terrain_res005_ring49_sparsity_off.yaml:0.12"      # sparsity_threshold 1.01
  "terrain_res005_ring49_n3.yaml:0.12"                # N = 3
  "terrain_res005_ring49_n5.yaml:0.12"                # N = 5
  "terrain_res005_ring49_pnt2.yaml:0.12"              # N_th = 2 (density rescaled)
)

if [ "${1:-}" = "--list" ]; then
  printf '%s\n' "${ROWS[@]}"
  exit 0
fi
if [ $# -gt 0 ]; then
  ROWS=("$@")
fi

# Bag path from `bag_path:=` in the extra launch arguments, else the single
# rosbag2_* directory here (the launch's own default rule).
bag_path_of() {
  local arg
  for arg in "$@"; do
    case "$arg" in
      bag_path:=*) echo "${arg#bag_path:=}"; return 0 ;;
    esac
  done
  ls -d rosbag2_* 2>/dev/null | head -1
}

bag_duration_s() {
  ros2 bag info "$1" 2>/dev/null |
    awk '/^Duration:/ { gsub("s", "", $2); print $2; exit }'
}

# Number of clouds in the bag: the per-row input volume every row must reach.
bag_scan_count() {
  ros2 bag info "$1" 2>/dev/null |
    awk '/Topic: \/cloud_registered_body/ {
           for (i = 1; i <= NF; i++) if ($i == "Count:") { print $(i + 1); exit }
         }'
}

run_row() { # config:rate[:timeout] [extra launch args...]
  local spec=$1; shift || true
  local cfg=${spec%%:*} rest=${spec#*:}
  local rate=${rest%%:*} tmo=${rest##*:}
  local bag expected=0
  bag=$(bag_path_of "$@")
  if [ "${rest}" = "${rest%%:*}" ]; then
    local duration
    duration=$(bag_duration_s "${bag}")
    if [ -n "${duration}" ]; then
      # +60 s, not +25: a SIGINT that lands while the map is still draining a
      # batch can take the process down before its destructor prints the final
      # line.  The extra margin lets the map go idle (and the state be stable)
      # before the interrupt.
      tmo=$(awk -v d="${duration}" -v r="${rate}" 'BEGIN { printf "%d", d / r + 60 }')
    else
      tmo=400
    fi
  fi
  expected=$(bag_scan_count "${bag}")
  expected=${expected:-0}

  local log
  log=$(mktemp /tmp/retap_table_XXXX.log)
  echo ">>> ${cfg}  bag_rate=${rate}  timeout=${tmo}s  expected=${expected} clouds"
  # SIGINT (not SIGTERM): the map plugin's destructor prints the final line the
  # table is built from.  The launch runs in the foreground because a `&` job of
  # a non-interactive shell inherits SIGINT=SIG_IGN, which launch then never
  # sees -- it would ignore the interrupt and `wait` would block forever.
  timeout -s INT "${tmo}" ros2 launch finenav_retap retap_rosbag.launch.py \
    config:="${cfg}" rviz:=false bag_rate:="${rate}" "$@" > "${log}" 2>&1

  local line fused
  line=$(grep -E "RetapTerrainMap: final" "${log}" | tail -1 | sed 's/^.*\]: //')
  fused=$(echo "${line}" | sed -n 's/^RetapTerrainMap: final \([0-9]*\) scans.*/\1/p')
  if [ -z "${line}" ]; then
    # The destructor line is the clean path.  When the shutdown races a busy
    # map the process can die by signal before it prints, so fall back to the
    # last periodic line -- but only when it already accounts for every cloud
    # in the bag, which makes it the same end state the destructor would print.
    line=$(grep -E "RetapTerrainMap: online" "${log}" | tail -1 | sed 's/^.*\]: //')
    fused=$(echo "${line}" | sed -n 's/^RetapTerrainMap: online \([0-9]*\) scans.*/\1/p')
    if [ -n "${line}" ] && [ "${expected}" -gt 0 ] && [ "${fused}" = "${expected}" ]; then
      echo "    (destructor line missing; last periodic line already covers all clouds)"
      line=$(echo "${line}" | sed 's/^RetapTerrainMap: online/RetapTerrainMap: final/')
    else
      [ -n "${line}" ] && echo "    (destructor line missing; last periodic line is stale)"
      line=""
    fi
  fi
  if [ -z "${line}" ]; then
    echo "    !! no usable line; last rows of ${log}:"; tail -5 "${log}" | sed 's/^/    /'
  else
    echo "    ${line}"
    if [ "${expected}" -gt 0 ] && [ -n "${fused}" ]; then
      # 98%, not 100%: under load ~1% of the clouds arrive without a robot
      # state at their stamp (the fuser logs "no robot state available ...
      # message dropped"), which is a property of the replay path, not of the
      # config.  Below 98% the row is capacity-limited and cannot be compared.
      if [ $((fused * 100)) -lt $((expected * 98)) ]; then
        echo "    !! ${fused}/${expected} clouds fused: this row is NOT comparable," \
             "lower bag_rate (the buffer drops what the map cannot analyse in time)"
      elif [ "${fused}" -lt "${expected}" ]; then
        echo "    (${fused}/${expected} clouds fused; the rest were dropped upstream" \
             "for lack of a robot state at their stamp -- see the WARN lines)"
      fi
    fi
  fi
  echo "    log: ${log}"
  pkill -f retap_demo 2>/dev/null
  pkill -f "ros2 bag play" 2>/dev/null
  sleep 2
}

for row in "${ROWS[@]}"; do
  run_row "${row}"
done
