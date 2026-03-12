#!/bin/bash
# Benchmark suite v2: ULG replay, uncapped FPS, Amdahl's law analysis
#
# Metrics per run: CPU%, VRAM, RAM, FPS percentiles, frame time percentiles,
#                  update_ms, draw_ms (phase breakdown for Amdahl's law)
#
# Amdahl's law design:
#   - Scaling series: 1, 4, 9, 16, 32, 64 drones (same feature config)
#     Shows how per-drone cost scales → reveals serial vs parallel fractions
#   - Feature isolation: each feature toggled individually at fixed drone counts
#     Shows per-feature cost delta → identifies expensive features
#   - Combined: all features on → checks if costs are additive
#
# Data source: 13b99e7f ULG log replayed on all drones with X/Z grid offset
# Resolution: 1920x1080 only

EXE="./build/Release/mavsim-viewer.exe"
OUT="bench_results_v2.csv"
LOG="tests/fixtures/13b99e7f-7cbe-4dc9-aab8-3c1c7b363d54.ulg"
DUR=30
W=1920
H=1080

# Header
# vram_free_mb: free VRAM in MB (NVIDIA=used, AMD=free — compute delta between runs for cost)
echo "drones,trail_mode,underwater,view,ortho,sidebar,resolution,framebuffer,mem_mb,vram_free_mb,cpu_pct,fps_min,fps_1pct,fps_5pct,fps_median,fps_avg,fps_max,ft_min,ft_p50,ft_p95,ft_p99,ft_avg,ft_max,update_ms,draw_ms,samples" > "$OUT"

RUN=0

# Count total runs for progress
# Scaling: 6 counts × 2 configs (baseline + trail) = 12
# Feature isolation at 16 drones: 6 configs = 6
# Feature isolation at 64 drones: 6 configs = 6
# Combined at 16 and 64: 2
# 1988 view at 16 and 64: 2 × 2 = 4
# Total: 30
TOTAL=30

run_bench() {
    local COUNT=$1
    local LABEL=$2
    shift 2
    RUN=$((RUN+1))
    echo "[$RUN/$TOTAL] $LABEL"
    "$EXE" -benchlog "$LOG" -benchn $COUNT -benchtime $DUR -benchout "$OUT" -w $W -h $H "$@" 2>/dev/null
}

echo "=== Phase 1: Scaling series (Amdahl's law) ==="
echo "--- Baseline: trail off ---"
for COUNT in 1 4 9 16 32 64; do
    run_bench $COUNT "${COUNT} drones | baseline (trail off)" -benchtrail 0 -benchview 0
done

echo ""
echo "--- Trail on ---"
for COUNT in 1 4 9 16 32 64; do
    run_bench $COUNT "${COUNT} drones | trail on" -benchtrail 1 -benchview 0
done

echo ""
echo "=== Phase 2: Feature isolation (16 drones) ==="
run_bench 16 "16 drones | baseline (trail off)"      -benchtrail 0 -benchview 0
run_bench 16 "16 drones | trail on"                   -benchtrail 1 -benchview 0
run_bench 16 "16 drones | speed ribbon"               -benchtrail 2 -benchview 0
run_bench 16 "16 drones | underwater + trail"          -benchtrail 1 -benchview 0 -benchuw
run_bench 16 "16 drones | ortho + trail"               -benchtrail 1 -benchview 0 -benchortho
run_bench 16 "16 drones | sidebar + trail"             -benchtrail 1 -benchview 0 -benchsidebar

echo ""
echo "=== Phase 3: Feature isolation (64 drones) ==="
run_bench 64 "64 drones | baseline (trail off)"       -benchtrail 0 -benchview 0
run_bench 64 "64 drones | trail on"                    -benchtrail 1 -benchview 0
run_bench 64 "64 drones | speed ribbon"                -benchtrail 2 -benchview 0
run_bench 64 "64 drones | underwater + trail"           -benchtrail 1 -benchview 0 -benchuw
run_bench 64 "64 drones | ortho + trail"                -benchtrail 1 -benchview 0 -benchortho
run_bench 64 "64 drones | sidebar + trail"              -benchtrail 1 -benchview 0 -benchsidebar

echo ""
echo "=== Phase 4: Combined + 1988 view ==="
# All features on (trail + underwater + sidebar)
run_bench 16 "16 drones | ALL features"  -benchtrail 2 -benchview 0 -benchuw -benchsidebar
run_bench 64 "64 drones | ALL features"  -benchtrail 2 -benchview 0 -benchuw -benchsidebar

# 1988 view (hidden mode) — same configs
run_bench 16 "16 drones | 1988 baseline"  -benchtrail 0 -benchview 4
run_bench 16 "16 drones | 1988 ribbon"    -benchtrail 2 -benchview 4
run_bench 64 "64 drones | 1988 baseline"  -benchtrail 0 -benchview 4
run_bench 64 "64 drones | 1988 ribbon"    -benchtrail 2 -benchview 4

echo ""
echo "All $TOTAL benchmarks complete. Results in $OUT"
echo ""
echo "=== Amdahl's Law Analysis ==="
echo "Compare scaling series: frame_time should be T_serial + N * T_per_drone"
echo "  T_per_drone = (ft_avg[64] - ft_avg[1]) / 63"
echo "  T_serial    = ft_avg[1] - T_per_drone"
echo "  Serial fraction S = T_serial / ft_avg[64]"
echo ""
cat "$OUT"
