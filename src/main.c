#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "data_source.h"
#include "ulog_replay.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include "debug_panel.h"
#include "ortho_panel.h"
#include "asset_path.h"

// Process memory query (avoids windows.h / raylib conflicts)
#ifdef _MSC_VER
  // Forward-declare just what we need from Win32 API
  typedef struct { unsigned long cb; unsigned long PageFaultCount;
    size_t PeakWorkingSetSize; size_t WorkingSetSize;
    size_t QuotaPeakPagedPoolUsage; size_t QuotaPagedPoolUsage;
    size_t QuotaPeakNonPagedPoolUsage; size_t QuotaNonPagedPoolUsage;
    size_t PagefileUsage; size_t PeakPagefileUsage;
  } PROCESS_MEMORY_COUNTERS_T;
  __declspec(dllimport) void* __stdcall GetCurrentProcess(void);
  __declspec(dllimport) int __stdcall K32GetProcessMemoryInfo(void*, PROCESS_MEMORY_COUNTERS_T*, unsigned long);
  #pragma comment(lib, "kernel32.lib")
  static size_t get_process_memory_mb(void) {
      PROCESS_MEMORY_COUNTERS_T pmc = {0};
      pmc.cb = sizeof(pmc);
      if (K32GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
          return pmc.WorkingSetSize / (1024 * 1024);
      return 0;
  }

  // CPU utilization via GetProcessTimes (kernel + user time)
  typedef struct { unsigned long dwLowDateTime; unsigned long dwHighDateTime; } FILETIME_T;
  __declspec(dllimport) int __stdcall GetProcessTimes(void*, FILETIME_T*, FILETIME_T*, FILETIME_T*, FILETIME_T*);
  __declspec(dllimport) int __stdcall QueryPerformanceCounter(int64_t*);
  __declspec(dllimport) int __stdcall QueryPerformanceFrequency(int64_t*);

  static uint64_t filetime_to_u64(FILETIME_T ft) {
      return ((uint64_t)ft.dwHighDateTime << 32) | (uint64_t)ft.dwLowDateTime;
  }

  typedef struct {
      int64_t wall_start;
      uint64_t cpu_start;   // kernel + user time in 100ns units
  } cpu_sample_t;

  static void cpu_sample_begin(cpu_sample_t *s) {
      FILETIME_T create, ex, kern, user;
      GetProcessTimes(GetCurrentProcess(), &create, &ex, &kern, &user);
      s->cpu_start = filetime_to_u64(kern) + filetime_to_u64(user);
      QueryPerformanceCounter(&s->wall_start);
  }

  static float cpu_sample_end_pct(const cpu_sample_t *s) {
      FILETIME_T create, ex, kern, user;
      GetProcessTimes(GetCurrentProcess(), &create, &ex, &kern, &user);
      uint64_t cpu_end = filetime_to_u64(kern) + filetime_to_u64(user);
      int64_t wall_end;
      QueryPerformanceCounter(&wall_end);
      int64_t freq;
      QueryPerformanceFrequency(&freq);
      double wall_s = (double)(wall_end - s->wall_start) / (double)freq;
      double cpu_s  = (double)(cpu_end - s->cpu_start) / 1e7;
      if (wall_s <= 0.0) return 0.0f;
      return (float)(cpu_s / wall_s * 100.0);
  }

  // High-resolution timer for frame phase breakdown
  static double qpc_now_ms(void) {
      int64_t now, freq;
      QueryPerformanceCounter(&now);
      QueryPerformanceFrequency(&freq);
      return (double)now / (double)freq * 1000.0;
  }

  // VRAM query via NVIDIA OpenGL extension (best-effort)
  #pragma comment(lib, "opengl32.lib")
  extern void __stdcall glGetIntegerv(unsigned int pname, int *data);
  extern unsigned int __stdcall glGetError(void);
  // VRAM query: try NVIDIA NVX extension, then AMD ATI extension
  #define GL_GPU_MEM_TOTAL_NVX  0x9048
  #define GL_GPU_MEM_AVAIL_NVX  0x9049
  #define GL_VBO_FREE_MEMORY_ATI 0x87FB
  #define GL_TEXTURE_FREE_MEMORY_ATI 0x87FC
  static size_t get_vram_used_mb(void) {
      int total_kb = 0, avail_kb = 0;
      glGetError();
      // Try NVIDIA NVX
      glGetIntegerv(GL_GPU_MEM_TOTAL_NVX, &total_kb);
      if (glGetError() == 0 && total_kb > 0) {
          glGetIntegerv(GL_GPU_MEM_AVAIL_NVX, &avail_kb);
          if (glGetError() == 0 && avail_kb >= 0)
              return (size_t)(total_kb - avail_kb) / 1024;
      }
      // Try AMD ATI_meminfo: returns [total_free, largest_free, total_aux_free, largest_aux_free] in KB
      glGetError();
      int ati_info[4] = {0};
      glGetIntegerv(GL_TEXTURE_FREE_MEMORY_ATI, ati_info);
      if (glGetError() == 0 && ati_info[0] > 0) {
          // ATI reports free VRAM, not used. Return free so we can compute deltas.
          return (size_t)ati_info[0] / 1024;
      }
      return 0;
  }
#else
  static size_t get_process_memory_mb(void) { return 0; }
  typedef struct { int64_t wall_start; uint64_t cpu_start; } cpu_sample_t;
  static void cpu_sample_begin(cpu_sample_t *s) { (void)s; }
  static float cpu_sample_end_pct(const cpu_sample_t *s) { (void)s; return 0.0f; }
  static double qpc_now_ms(void) { return 0.0; }
  static size_t get_vram_used_mb(void) { return 0; }
#endif

#define MAX_VEHICLES 65
#define MISSILE_COUNT 16
#define BENCH_COUNT  64

// HSV to RGB for procedural color generation
static Color hsv_to_color(float h, float s, float v) {
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r, g, b;
    if (h < 60)       { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else               { r = c; g = 0; b = x; }
    return (Color){ (unsigned char)((r+m)*255), (unsigned char)((g+m)*255),
                    (unsigned char)((b+m)*255), 255 };
}

static Color vehicle_color(int i) {
    // First 16: hand-picked distinct colors
    static const Color named[] = {
        {230, 230, 230, 255}, {230,  41,  55, 255}, {  0, 228,  48, 255},
        {  0, 121, 241, 255}, {253, 249,   0, 255}, {255,   0, 255, 255},
        {  0, 255, 255, 255}, {255, 161,   0, 255}, {200, 122, 255, 255},
        {127, 106,  79, 255}, {255, 109, 194, 255}, {  0, 182, 172, 255},
        {135, 206, 235, 255}, {255, 203, 164, 255}, {170, 255, 128, 255},
        {200, 200, 200, 255},
    };
    if (i < 16) return named[i];
    // Beyond 16: golden-angle hue spread with alternating brightness
    float hue = fmodf(i * 137.508f, 360.0f);
    float sat = 0.7f + (i % 3) * 0.1f;
    float val = 0.8f + (i % 2) * 0.15f;
    return hsv_to_color(hue, sat, val);
}

// ── Anime missile barrage simulation ────────────────────────────────────────
// 16 drones launch from a near-single origin, arc outward in a spherical fan,
// then heat-seek a VTOL target with oscillating sidewind for visual drama.
typedef struct {
    float theta;        // spherical azimuth of fan-out direction
    float phi;          // spherical elevation of fan-out direction
    float phase;        // oscillation phase offset (unique per missile)
    float osc_freq;     // sidewind oscillation frequency
    float osc_amp;      // sidewind amplitude
    float arc_radius;   // how far it arcs out before seeking
    float speed;        // base speed multiplier
} missile_params_t;

static void missile_barrage_init(missile_params_t params[MISSILE_COUNT]) {
    for (int i = 0; i < MISSILE_COUNT; i++) {
        // Distribute missiles in a spherical fan (golden angle spiral)
        float golden = (float)M_PI * (3.0f - sqrtf(5.0f));
        float y = 1.0f - ((float)i / (MISSILE_COUNT - 1)) * 1.4f; // -0.4 to 1.0
        float radius_at_y = sqrtf(1.0f - y * y);
        float angle = (float)i * golden;

        params[i].theta = angle;
        params[i].phi = asinf(y * 0.6f + 0.2f); // bias upward slightly
        params[i].phase = (float)i * 0.7f + (float)i * (float)i * 0.03f;
        params[i].osc_freq = 3.0f + (float)(i % 5) * 1.2f;
        params[i].osc_amp = 1.5f + (float)(i % 3) * 0.8f;
        params[i].arc_radius = 15.0f + (float)(i % 4) * 5.0f;
        params[i].speed = 0.9f + (float)(i % 7) * 0.05f;
    }
}

// Generate hil_state for a missile at time t (seconds)
static void missile_barrage_state(const missile_params_t *p, float t,
                                  Vector3 origin, Vector3 target,
                                  hil_state_t *out_state)
{
    // Phase 1: fan out in spherical arc (0 to ~3s)
    // Phase 2: seek target with oscillation (~3s onward)
    float fan_duration = 2.5f * (1.0f / p->speed);
    float seek_start = fan_duration;
    float total_flight = seek_start + 6.0f; // converge over ~6s

    if (t > total_flight) t = total_flight; // clamp

    float progress = t / total_flight; // 0 to 1 overall

    // Fan-out direction in world space
    float ct = cosf(p->theta), st = sinf(p->theta);
    float cp = cosf(p->phi), sp = sinf(p->phi);
    Vector3 fan_dir = { ct * cp, sp, st * cp };

    // Arc apex point
    Vector3 apex = {
        origin.x + fan_dir.x * p->arc_radius,
        origin.y + fan_dir.y * p->arc_radius + 5.0f,
        origin.z + fan_dir.z * p->arc_radius
    };

    Vector3 pos;
    if (t < seek_start) {
        // Phase 1: accelerating arc from origin toward apex
        float u = t / seek_start;
        float ease = u * u * (3.0f - 2.0f * u); // smoothstep
        pos.x = origin.x + (apex.x - origin.x) * ease;
        pos.y = origin.y + (apex.y - origin.y) * ease;
        pos.z = origin.z + (apex.z - origin.z) * ease;
    } else {
        // Phase 2: curved seek from apex toward target
        float u = (t - seek_start) / (total_flight - seek_start);
        float ease = u * u * (3.0f - 2.0f * u); // smoothstep into target

        pos.x = apex.x + (target.x - apex.x) * ease;
        pos.y = apex.y + (target.y - apex.y) * ease;
        pos.z = apex.z + (target.z - apex.z) * ease;
    }

    // Sidewind oscillation (perpendicular to travel direction)
    Vector3 to_target = { target.x - pos.x, target.y - pos.y, target.z - pos.z };
    float dist = sqrtf(to_target.x*to_target.x + to_target.y*to_target.y + to_target.z*to_target.z);
    if (dist > 0.1f) {
        to_target.x /= dist; to_target.y /= dist; to_target.z /= dist;
    }
    // Perpendicular vectors for oscillation
    Vector3 up = {0, 1, 0};
    Vector3 side = {
        to_target.y * up.z - to_target.z * up.y,
        to_target.z * up.x - to_target.x * up.z,
        to_target.x * up.y - to_target.y * up.x
    };
    float slen = sqrtf(side.x*side.x + side.y*side.y + side.z*side.z);
    if (slen > 0.001f) { side.x /= slen; side.y /= slen; side.z /= slen; }

    Vector3 up2 = {
        to_target.y * side.z - to_target.z * side.y,
        to_target.z * side.x - to_target.x * side.z,
        to_target.x * side.y - to_target.y * side.x
    };

    // Oscillation decays as missile approaches target
    float decay = 1.0f - progress * progress;
    float osc_h = sinf(t * p->osc_freq + p->phase) * p->osc_amp * decay;
    float osc_v = cosf(t * p->osc_freq * 0.7f + p->phase * 1.3f) * p->osc_amp * 0.6f * decay;

    pos.x += side.x * osc_h + up2.x * osc_v;
    pos.y += side.y * osc_h + up2.y * osc_v;
    pos.z += side.z * osc_h + up2.z * osc_v;

    // Convert to lat/lon/alt for hil_state
    // Use a simple NED offset: origin at (0,0,alt), position is meters offset
    // We'll set lat/lon relative to a base and encode as degE7
    double base_lat = 47.397742;
    double base_lon = 8.545594;
    double base_alt = 489.4;

    double meters_per_deg_lat = 111132.0;
    double meters_per_deg_lon = 111132.0 * cos(base_lat * M_PI / 180.0);

    out_state->lat = (int32_t)((base_lat + (double)pos.z / meters_per_deg_lat) * 1e7);
    out_state->lon = (int32_t)((base_lon + (double)pos.x / meters_per_deg_lon) * 1e7);
    out_state->alt = (int32_t)((base_alt + (double)pos.y) * 1000.0);

    // Velocity (finite difference approximation via direction)
    float spd = 8.0f + progress * 15.0f; // accelerating
    out_state->vx = (int16_t)(to_target.z * spd * 100.0f); // NED north = +Z in our coords
    out_state->vy = (int16_t)(to_target.x * spd * 100.0f); // NED east = +X
    out_state->vz = (int16_t)(-to_target.y * spd * 100.0f); // NED down = -Y
    out_state->ind_airspeed = (uint16_t)(spd * 100.0f);
    out_state->true_airspeed = out_state->ind_airspeed;

    // Simple orientation: nose toward target
    out_state->quaternion[0] = 1.0f;
    out_state->quaternion[1] = 0.0f;
    out_state->quaternion[2] = 0.0f;
    out_state->quaternion[3] = 0.0f;
    // Compute yaw quaternion from to_target
    float yaw = atan2f(to_target.x, to_target.z);
    float pitch = -asinf(to_target.y);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cp2 = cosf(pitch * 0.5f), sp2 = sinf(pitch * 0.5f);
    out_state->quaternion[0] = cy * cp2;
    out_state->quaternion[1] = 0;
    out_state->quaternion[2] = sy * cp2;
    out_state->quaternion[3] = cy * sp2;

    out_state->valid = true;
    out_state->time_usec = (uint64_t)(t * 1e6);
}

// ── Benchmark: drones in offset vertical figure-eights at max airspeed ──
#define BENCH_SPEED     30.0f   // m/s (~108 km/h, typical max airspeed)
#define BENCH_RADIUS    25.0f   // radius of each lobe of the figure-eight

typedef struct {
    float phase;        // phase offset so drones don't overlap
    float center_x;     // center of figure-eight in world X
    float center_z;     // center of figure-eight in world Z
} bench_params_t;

static void bench_init(bench_params_t *params, int count) {
    // Arrange in a square grid: 3x3 for 9, 4x4 for 16
    int cols = (int)ceilf(sqrtf((float)count));
    float grid_spacing = BENCH_RADIUS * 2.5f;
    float offset = (cols - 1) * 0.5f;
    for (int i = 0; i < count; i++) {
        int row = i / cols;
        int col = i % cols;
        params[i].center_x = (col - offset) * grid_spacing;
        params[i].center_z = (row - offset) * grid_spacing;
        // Offset phase so adjacent drones are never at the same point
        params[i].phase = (float)i * (2.0f * (float)M_PI / count);
    }
}

static void bench_state(const bench_params_t *p, float t,
                         hil_state_t *out_state)
{
    // Vertical figure-eight using lemniscate of Bernoulli parametric form:
    //   x(t) = R * sin(a)
    //   y(t) = R * sin(a) * cos(a)  (= R/2 * sin(2a))
    // where a = angular parameter advancing at constant speed
    float period = (2.0f * (float)M_PI * BENCH_RADIUS * 2.0f) / BENCH_SPEED;
    float a = (t / period) * 2.0f * (float)M_PI + p->phase;

    // Horizontal position: figure-eight in XZ plane offset by center
    float local_x = BENCH_RADIUS * sinf(a);
    float local_z = BENCH_RADIUS * sinf(a) * cosf(a);

    // Vertical: oscillate between 10m and 10+2*R meters
    float local_y = 10.0f + BENCH_RADIUS + BENCH_RADIUS * cosf(a);

    float pos_x = p->center_x + local_x;
    float pos_z = p->center_z + local_z;
    float pos_y = local_y;

    // Velocity (analytical derivative)
    float da_dt = (2.0f * (float)M_PI) / period;
    float vx = BENCH_RADIUS * cosf(a) * da_dt;
    float vz = BENCH_RADIUS * (cosf(a) * cosf(a) - sinf(a) * sinf(a)) * da_dt;
    float vy = -BENCH_RADIUS * sinf(a) * da_dt;

    // Speed for airspeed reporting
    float spd = sqrtf(vx*vx + vy*vy + vz*vz);

    // Convert to lat/lon/alt
    double base_lat = 47.397742;
    double base_lon = 8.545594;
    double base_alt = 489.4;
    double meters_per_deg_lat = 111132.0;
    double meters_per_deg_lon = 111132.0 * cos(base_lat * M_PI / 180.0);

    out_state->lat = (int32_t)((base_lat + (double)pos_z / meters_per_deg_lat) * 1e7);
    out_state->lon = (int32_t)((base_lon + (double)pos_x / meters_per_deg_lon) * 1e7);
    out_state->alt = (int32_t)((base_alt + (double)pos_y) * 1000.0);

    // NED velocity
    out_state->vx = (int16_t)(vz * 100.0f);   // NED north
    out_state->vy = (int16_t)(vx * 100.0f);   // NED east
    out_state->vz = (int16_t)(-vy * 100.0f);  // NED down
    out_state->ind_airspeed = (uint16_t)(spd * 100.0f);
    out_state->true_airspeed = out_state->ind_airspeed;

    // Orientation: nose along velocity vector
    float yaw = atan2f(vx, vz);
    float pitch = -asinf(vy / (spd > 0.1f ? spd : 0.1f));
    float roll = atan2f(vx * da_dt, 9.81f) * 0.3f; // gentle banking

    // Euler to quaternion (ZYX order for NED)
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    out_state->quaternion[0] = cr*cp*cy + sr*sp*sy;
    out_state->quaternion[1] = sr*cp*cy - cr*sp*sy;
    out_state->quaternion[2] = cr*cp*sy + sr*sp*cy;
    out_state->quaternion[3] = cr*sp*cy - sr*cp*sy;

    out_state->valid = true;
    out_state->time_usec = (uint64_t)(t * 1e6);
}

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file.ulg>  Replay ULog file\n");
    printf("  -w <width>     Window width (default: 1280)\n");
    printf("  -h <height>    Window height (default: 720)\n");
}

int main(int argc, char *argv[]) {
    uint16_t base_port = 19410;
    int vehicle_count = 1;
    int model_idx = MODEL_QUADROTOR;
    int win_w = 1280;
    int win_h = 720;
    bool debug = false;
    // PX4 SIH default spawn position
    double origin_lat = 47.397742;
    double origin_lon = 8.545594;
    double origin_alt = 489.4;
    bool origin_specified = false;
    const char *replay_file = NULL;
    bool missile_mode = false;
    bool bench_mode = false;
    int bench_count = 0;
    float bench_duration = 0.0f;    // auto-exit after N seconds (0 = disabled)
    int bench_trail_mode = -1;      // -1 = don't override
    int bench_underwater = -1;      // -1 = don't override
    int bench_view = -1;            // -1 = don't override (0=grid,1=jmav,2=rez,3=snow,4=1988)
    int bench_sel = -1;             // -1 = auto (center drone)
    int bench_ortho = 0;            // 0 = perspective, 1 = ORTHO_TOP
    int bench_sidebar = 0;          // 0 = off, 1 = sidebar ortho panels
    const char *bench_outfile = NULL;
    const char *bench_logfile = NULL;  // ULG log for replay-based benchmarks
    bool fake_mode = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-udp") == 0 && i + 1 < argc) {
            base_port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            vehicle_count = atoi(argv[++i]);
            if (vehicle_count < 1) vehicle_count = 1;
            if (vehicle_count > MAX_VEHICLES) vehicle_count = MAX_VEHICLES;
        } else if (strcmp(argv[i], "-origin") == 0 && i + 3 < argc) {
            origin_lat = atof(argv[++i]);
            origin_lon = atof(argv[++i]);
            origin_alt = atof(argv[++i]);
            origin_specified = true;
        } else if (strcmp(argv[i], "-mc") == 0) {
            model_idx = MODEL_QUADROTOR;
        } else if (strcmp(argv[i], "-fw") == 0) {
            model_idx = MODEL_FIXEDWING;
        } else if (strcmp(argv[i], "-ts") == 0) {
            model_idx = MODEL_TAILSITTER;
        } else if (strcmp(argv[i], "-vtol") == 0) {
            model_idx = MODEL_VTOL;
        } else if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            win_w = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            win_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--replay") == 0 && i + 1 < argc) {
            replay_file = argv[++i];
        } else if (strcmp(argv[i], "-missile") == 0) {
            missile_mode = true;
        } else if (strcmp(argv[i], "-bench") == 0) {
            bench_mode = true;
            bench_count = 16; // default benchmark
        } else if (strcmp(argv[i], "-bench9") == 0) {
            bench_mode = true;
            bench_count = 9;
        } else if (strcmp(argv[i], "-bench16") == 0) {
            bench_mode = true;
            bench_count = 16;
        } else if (strcmp(argv[i], "-bench32") == 0) {
            bench_mode = true;
            bench_count = 32;
        } else if (strcmp(argv[i], "-bench64") == 0) {
            bench_mode = true;
            bench_count = 64;
        } else if (strcmp(argv[i], "-benchtime") == 0 && i + 1 < argc) {
            bench_duration = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "-benchtrail") == 0 && i + 1 < argc) {
            bench_trail_mode = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-benchuw") == 0) {
            bench_underwater = 1;
        } else if (strcmp(argv[i], "-benchview") == 0 && i + 1 < argc) {
            bench_view = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-benchout") == 0 && i + 1 < argc) {
            bench_outfile = argv[++i];
        } else if (strcmp(argv[i], "-benchsel") == 0 && i + 1 < argc) {
            bench_sel = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-benchortho") == 0) {
            bench_ortho = 1;
        } else if (strcmp(argv[i], "-benchsidebar") == 0) {
            bench_sidebar = 1;
        } else if (strcmp(argv[i], "-benchlog") == 0 && i + 1 < argc) {
            bench_logfile = argv[++i];
            bench_mode = true;
            if (bench_count == 0) bench_count = 16;
        } else if (strcmp(argv[i], "-benchn") == 0 && i + 1 < argc) {
            bench_mode = true;
            bench_count = atoi(argv[++i]);
            if (bench_count < 1) bench_count = 1;
            if (bench_count > BENCH_COUNT) bench_count = BENCH_COUNT;
        } else if (strcmp(argv[i], "-fake") == 0) {
            fake_mode = true;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    asset_path_init();

    // Init Raylib
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "MAVSim Viewer");
    if (bench_mode && bench_duration > 0.0f)
        SetTargetFPS(0);   // uncap for accurate profiling
    else
        SetTargetFPS(60);

    // Init data sources
    data_source_t sources[MAX_VEHICLES];
    memset(sources, 0, sizeof(sources));
    bool is_replay = (replay_file != NULL);

    // Missile mode overrides vehicle count
    missile_params_t missile_params[MISSILE_COUNT];
    float missile_time = 0.0f;
    if (missile_mode) {
        vehicle_count = MISSILE_COUNT + 1; // 16 missiles + 1 target
        missile_barrage_init(missile_params);
        origin_specified = true; // force shared origin
    }

    // Benchmark mode: drones in figure-eights (synthetic) or ULG replay
    bench_params_t bench_params[BENCH_COUNT];
    float bench_time = 0.0f;
    ulog_replay_ctx_t *bench_replays = NULL;
    float bench_offset_x[BENCH_COUNT];  // East offset per drone (meters)
    float bench_offset_z[BENCH_COUNT];  // North offset per drone (meters)
    #define BENCH_LOG_SPACING 12.5f
    if (bench_mode) {
        vehicle_count = bench_count;
        if (bench_logfile) {
            // ULG replay-based benchmark: each drone replays the same log
            bench_replays = (ulog_replay_ctx_t *)calloc(bench_count, sizeof(ulog_replay_ctx_t));
            int cols = (int)ceilf(sqrtf((float)bench_count));
            float grid_offset = (cols - 1) * 0.5f;
            for (int i = 0; i < bench_count; i++) {
                if (ulog_replay_init(&bench_replays[i], bench_logfile) != 0) {
                    fprintf(stderr, "Failed to open ULog for bench drone %d: %s\n", i, bench_logfile);
                    free(bench_replays); bench_replays = NULL;
                    CloseWindow(); return 1;
                }
                int row = i / cols;
                int col = i % cols;
                bench_offset_x[i] = (col - grid_offset) * BENCH_LOG_SPACING;
                bench_offset_z[i] = (row - grid_offset) * BENCH_LOG_SPACING;
            }
            // Derive NED origin from the log's home position
            ulog_replay_advance(&bench_replays[0], 5.0f, 1.0f, false, false);
            if (bench_replays[0].home.valid) {
                origin_lat = (double)bench_replays[0].home.lat / 1e7;
                origin_lon = (double)bench_replays[0].home.lon / 1e7;
                origin_alt = (double)bench_replays[0].home.alt / 1000.0;
            }
            // Rewind all replays to start
            for (int i = 0; i < bench_count; i++)
                ulog_replay_seek(&bench_replays[i], 0.0f);
            printf("Bench: ULG replay with %d drones, grid %dx%d @ %.0fm spacing\n",
                   bench_count, cols, (bench_count + cols - 1) / cols, BENCH_LOG_SPACING);
        } else {
            bench_init(bench_params, bench_count);
        }
        origin_specified = true;
    }

    if (fake_mode) {
        vehicle_count = 1;
        origin_specified = true;
    }

    if (is_replay) {
        vehicle_count = 1;  // single vehicle replay (multi-file is future scope)
        if (data_source_ulog_create(&sources[0], replay_file) != 0) {
            fprintf(stderr, "Failed to open ULog: %s\n", replay_file);
            CloseWindow();
            return 1;
        }
    } else if (!missile_mode && !bench_mode && !fake_mode) {
        for (int i = 0; i < vehicle_count; i++) {
            if (data_source_mavlink_create(&sources[i], base_port + i, (uint8_t)i, debug) != 0) {
                fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", base_port + i);
                CloseWindow();
                return 1;
            }
        }
    }

    // Init vehicles
    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    vehicle_t vehicles[MAX_VEHICLES];
    if (missile_mode) {
        // Missiles: FPV quads, target: VTOL
        for (int i = 0; i < MISSILE_COUNT; i++) {
            vehicle_init(&vehicles[i], MODEL_FPV_QUAD, scene.lighting_shader);
            vehicles[i].color = vehicle_color(i);
        }
        vehicle_init(&vehicles[MISSILE_COUNT], MODEL_VTOL, scene.lighting_shader);
        vehicles[MISSILE_COUNT].color = vehicle_color(MISSILE_COUNT);
    } else if (bench_mode) {
        int bench_model = bench_replays ? MODEL_VTOL : MODEL_QUADROTOR;
        for (int i = 0; i < bench_count; i++) {
            vehicle_init(&vehicles[i], bench_model, scene.lighting_shader);
            vehicles[i].color = vehicle_color(i);
        }
    } else if (is_replay) {
        // Persistent trail for replay: 36000 points (~10+ min at adaptive rate)
        vehicle_init_ex(&vehicles[0], model_idx, scene.lighting_shader, 36000);
        vehicles[0].color = vehicle_color(0);
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
            vehicles[i].color = vehicle_color(i);
        }
    }

    // For multi-vehicle or explicit origin: pre-set the NED origin on all vehicles
    if (vehicle_count > 1 || origin_specified) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
        printf("NED origin: lat=%.6f lon=%.6f alt=%.1f\n", origin_lat, origin_lon, origin_alt);
    }

    hud_t hud;
    hud_init(&hud);
    hud.is_replay = is_replay;

    debug_panel_t dbg_panel;
    debug_panel_init(&dbg_panel);

    ortho_panel_t ortho;
    ortho_panel_init(&ortho);
    if (bench_sidebar) ortho.visible = true;

    int selected = 0;
    if (bench_mode && bench_sel >= 0) {
        selected = bench_sel;
    } else if (bench_mode) {
        // Auto-select back-center drone: middle of last row
        int cols = (int)ceilf(sqrtf((float)bench_count));
        int last_row = (bench_count - 1) / cols;
        selected = last_row * cols + cols / 2;
        if (selected >= bench_count) selected = bench_count - 1;
    }
    bool was_connected[MAX_VEHICLES];
    memset(was_connected, 0, sizeof(was_connected));
    Vector3 last_pos[MAX_VEHICLES];
    memset(last_pos, 0, sizeof(last_pos));
    bool show_hud = true;
    int trail_mode = 1;              // 0=off, 1=directional trail, 2=speed ribbon
    bool show_ground_track = false;  // ground projection off by default
    bool classic_colors = false;     // L key: toggle classic (red/blue) vs modern (yellow/purple)
    bool missile_paused = false;

    // Frame markers for replay (issue #41)
    #define MAX_MARKERS 256
    #define MARKER_LABEL_MAX 48
    float marker_times[MAX_MARKERS];       // replay timestamp (seconds) for each marker
    Vector3 marker_positions[MAX_MARKERS]; // world position at each marker
    char marker_labels[MAX_MARKERS][MARKER_LABEL_MAX];
    // Snapshotted telemetry at each marker (for stable trail-matched colors)
    float marker_roll[MAX_MARKERS];
    float marker_pitch[MAX_MARKERS];
    float marker_vert[MAX_MARKERS];
    float marker_speed[MAX_MARKERS];
    memset(marker_labels, 0, sizeof(marker_labels));
    int marker_count = 0;
    int current_marker = -1;               // index of last-jumped-to marker (-1 = none)
    double last_marker_drop_time = 0.0;    // for B→L chord detection
    int last_marker_drop_idx = -1;         // which marker was just dropped

    // System markers (from ULog mode changes) — cubes, not user-editable
    #define MAX_SYS_MARKERS 256
    int sys_marker_count = 0;
    float sys_marker_times[MAX_SYS_MARKERS];
    Vector3 sys_marker_positions[MAX_SYS_MARKERS];
    char sys_marker_labels[MAX_SYS_MARKERS][MARKER_LABEL_MAX];
    float sys_marker_roll[MAX_SYS_MARKERS];
    float sys_marker_pitch[MAX_SYS_MARKERS];
    float sys_marker_vert[MAX_SYS_MARKERS];
    float sys_marker_speed[MAX_SYS_MARKERS];
    memset(sys_marker_labels, 0, sizeof(sys_marker_labels));
    int current_sys_marker = -1;  // -1 = none selected
    bool sys_marker_selected = false;  // true = current selection is a sys marker, false = user marker

    // Helper: sync vehicle state after a replay seek.
    // Forces state copy from ctx→sources→vehicle, truncates trail to seek time.
    #define REPLAY_SYNC_AFTER_SEEK(ctx, src, veh) do { \
        (src)->state = (ctx)->state; \
        (src)->home = (ctx)->home; \
        (src)->playback.position_s = (float)(ctx)->wall_accum; \
        uint64_t _range = (ctx)->parser.end_timestamp - (ctx)->parser.start_timestamp; \
        if (_range > 0) (src)->playback.progress = (src)->playback.position_s / ((float)((double)_range / 1e6)); \
        (veh)->current_time = (src)->playback.position_s; \
        vehicle_update((veh), &(src)->state, &(src)->home); \
        vehicle_truncate_trail((veh), (src)->playback.position_s); \
    } while(0)

    // Populate system markers by seeking to each mode change time
    // Read directly from ulog ctx since sources[0].playback isn't populated yet (ulog_poll hasn't run)
    if (is_replay) {
        ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
        int count = ctx->mode_change_count;
        if (count > MAX_SYS_MARKERS) count = MAX_SYS_MARKERS;
        if (count > 0) {
            // Store times and labels now; positions resolved lazily after origin is established
            for (int i = 0; i < count; i++) {
                sys_marker_times[i] = ctx->mode_changes[i].time_s;
                const char *name = ulog_nav_state_name(ctx->mode_changes[i].nav_state);
                snprintf(sys_marker_labels[i], MARKER_LABEL_MAX, "%s", name);
                sys_marker_positions[i] = (Vector3){0};
            }
            sys_marker_count = count;
        }
    }

    bool sys_markers_resolved = false;  // set true after positions are computed

    // Pre-computed flight trail (built once after origin is established)
    // Stores the entire flight path so marker seeks can restore trail instantly
    #define PRECOMP_TRAIL_MAX 36000
    Vector3 *precomp_trail = NULL;
    float *precomp_roll = NULL, *precomp_pitch = NULL;
    float *precomp_vert = NULL, *precomp_speed = NULL, *precomp_time = NULL;
    int precomp_count = 0;
    float precomp_speed_max = 0.0f;
    bool precomp_ready = false;

    // Marker label input state
    bool show_marker_labels = true;
    bool marker_input_active = false;
    char marker_input_buf[MARKER_LABEL_MAX];
    int marker_input_len = 0;
    int marker_input_target = -1;          // which marker is being labeled

    // Bench replay playback controls
    float bench_replay_speed = 1.0f;
    bool bench_replay_paused = false;

    // Benchmark instrumentation
    if (bench_trail_mode >= 0) trail_mode = bench_trail_mode;
    if (bench_underwater == 1) scene.is_underwater = true;
    if (bench_view >= 0) scene.view_mode = (view_mode_t)bench_view;

    // Bench ortho: top-down view
    if (bench_mode && bench_ortho) {
        scene.ortho_mode = ORTHO_TOP;
        // Set span to cover the full formation
        int cols = (int)ceilf(sqrtf((float)bench_count));
        float grid_sp = BENCH_RADIUS * 2.5f;
        scene.ortho_span = cols * grid_sp + BENCH_RADIUS * 4.0f;
    }

    // Bench camera: free mode, positioned behind and above the entire formation
    if (bench_mode && bench_duration > 0.0f && !bench_ortho) {
        scene.cam_mode = CAM_MODE_FREE;
        int cols = (int)ceilf(sqrtf((float)bench_count));
        float grid_sp = bench_replays ? BENCH_LOG_SPACING : (BENCH_RADIUS * 2.5f);
        float extent = cols * grid_sp;
        float cam_dist = extent * 0.5f + 30.0f;
        float cam_height = extent * 0.25f + 25.0f;
        scene.camera.position = (Vector3){ 0.0f, cam_height, cam_dist };
        scene.camera.target = (Vector3){ 0.0f, 15.0f, 0.0f };
        scene.camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    }

    #define BENCH_MAX_SAMPLES 4096
    float bench_fps_samples[BENCH_MAX_SAMPLES];
    float bench_ft_samples[BENCH_MAX_SAMPLES];   // frame time in ms
    int bench_sample_count = 0;
    float bench_elapsed = 0.0f;
    float bench_sample_timer = 0.0f;
    float bench_fps_min = 9999.0f, bench_fps_max = 0.0f, bench_fps_sum = 0.0f;
    float bench_ft_min = 9999.0f, bench_ft_max = 0.0f, bench_ft_sum = 0.0f;
    // CPU% tracking
    cpu_sample_t bench_cpu_sample;
    float bench_cpu_pct_sum = 0.0f;
    int bench_cpu_count = 0;
    // Phase timing accumulators (ms)
    double bench_update_sum = 0.0, bench_draw_sum = 0.0;
    int bench_phase_count = 0;
    if (bench_mode && bench_duration > 0.0f)
        cpu_sample_begin(&bench_cpu_sample);

    // Main loop
    while (!WindowShouldClose()) {
        // Benchmark: collect FPS, frame time, CPU% and auto-exit
        if (bench_duration > 0.0f) {
            float dt = GetFrameTime();
            float ft_ms = dt * 1000.0f;
            bench_elapsed += dt;
            bench_sample_timer += dt;
            if (bench_elapsed > 2.0f) { // skip first 2s warmup
                float fps = (float)GetFPS();
                if (bench_sample_timer >= 0.1f) { // sample every 100ms
                    bench_sample_timer = 0.0f;
                    if (bench_sample_count < BENCH_MAX_SAMPLES) {
                        bench_fps_samples[bench_sample_count] = fps;
                        bench_ft_samples[bench_sample_count] = ft_ms;
                        bench_sample_count++;
                    }
                    bench_fps_sum += fps;
                    if (fps < bench_fps_min) bench_fps_min = fps;
                    if (fps > bench_fps_max) bench_fps_max = fps;
                    bench_ft_sum += ft_ms;
                    if (ft_ms < bench_ft_min) bench_ft_min = ft_ms;
                    if (ft_ms > bench_ft_max) bench_ft_max = ft_ms;
                    // CPU% snapshot every second
                    bench_cpu_count++;
                    if (bench_cpu_count % 10 == 0) {
                        bench_cpu_pct_sum += cpu_sample_end_pct(&bench_cpu_sample);
                        cpu_sample_begin(&bench_cpu_sample);
                    }
                }
            }
            if (bench_elapsed >= bench_duration + 2.0f) { // +2s for warmup
                // Sort FPS samples for percentiles
                for (int i = 0; i < bench_sample_count - 1; i++)
                    for (int j = i + 1; j < bench_sample_count; j++)
                        if (bench_fps_samples[i] > bench_fps_samples[j]) {
                            float tmp = bench_fps_samples[i];
                            bench_fps_samples[i] = bench_fps_samples[j];
                            bench_fps_samples[j] = tmp;
                        }
                // Sort frame time samples for percentiles
                float ft_sorted[BENCH_MAX_SAMPLES];
                memcpy(ft_sorted, bench_ft_samples, bench_sample_count * sizeof(float));
                for (int i = 0; i < bench_sample_count - 1; i++)
                    for (int j = i + 1; j < bench_sample_count; j++)
                        if (ft_sorted[i] > ft_sorted[j]) {
                            float tmp = ft_sorted[i];
                            ft_sorted[i] = ft_sorted[j];
                            ft_sorted[j] = tmp;
                        }

                int n = bench_sample_count;
                float fps_avg = n > 0 ? bench_fps_sum / n : 0;
                float fps_p1  = n > 0 ? bench_fps_samples[(int)(n * 0.01f)] : 0;
                float fps_p5  = n > 0 ? bench_fps_samples[(int)(n * 0.05f)] : 0;
                float fps_med = n > 0 ? bench_fps_samples[n / 2] : 0;

                float ft_avg = n > 0 ? bench_ft_sum / n : 0;
                float ft_p50 = n > 0 ? ft_sorted[n / 2] : 0;
                float ft_p95 = n > 0 ? ft_sorted[(int)(n * 0.95f)] : 0;
                float ft_p99 = n > 0 ? ft_sorted[(int)(n * 0.99f)] : 0;

                int cpu_snapshots = bench_cpu_count / 10;
                float cpu_pct = cpu_snapshots > 0 ? bench_cpu_pct_sum / cpu_snapshots : 0;
                size_t mem_mb = get_process_memory_mb();
                size_t vram_mb = get_vram_used_mb();
                int render_w = GetRenderWidth();
                int render_h = GetRenderHeight();
                float update_avg = bench_phase_count > 0 ? (float)(bench_update_sum / bench_phase_count) : 0;
                float draw_avg = bench_phase_count > 0 ? (float)(bench_draw_sum / bench_phase_count) : 0;

                if (bench_outfile) {
                    FILE *f = fopen(bench_outfile, "a");
                    if (f) {
                        fprintf(f, "%d,%d,%d,%d,%d,%d,%dx%d,%dx%d,"
                                   "%zu,%zu,%.1f,"
                                   "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
                                   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                                   "%.3f,%.3f,%d\n",
                                bench_count, bench_trail_mode, bench_underwater == 1 ? 1 : 0,
                                bench_view >= 0 ? bench_view : 0, bench_ortho, bench_sidebar,
                                GetScreenWidth(), GetScreenHeight(), render_w, render_h,
                                mem_mb, vram_mb, cpu_pct,
                                bench_fps_min, fps_p1, fps_p5, fps_med, fps_avg, bench_fps_max,
                                bench_ft_min, ft_p50, ft_p95, ft_p99, ft_avg, bench_ft_max,
                                update_avg, draw_avg, n);
                        fclose(f);
                    }
                }
                printf("BENCH: drones=%d trail=%d uw=%d view=%d res=%dx%d fb=%dx%d\n"
                       "  MEM=%zuMB VRAM=%zuMB CPU=%.1f%%\n"
                       "  FPS: min=%.1f 1%%=%.1f 5%%=%.1f med=%.1f avg=%.1f max=%.1f\n"
                       "  FT:  min=%.3f p50=%.3f p95=%.3f p99=%.3f avg=%.3f max=%.3fms\n"
                       "  update=%.3fms draw=%.3fms (n=%d)\n",
                       bench_count, trail_mode, scene.is_underwater ? 1 : 0,
                       (int)scene.view_mode, GetScreenWidth(), GetScreenHeight(),
                       render_w, render_h,
                       mem_mb, vram_mb, cpu_pct,
                       bench_fps_min, fps_p1, fps_p5, fps_med, fps_avg, bench_fps_max,
                       bench_ft_min, ft_p50, ft_p95, ft_p99, ft_avg, bench_ft_max,
                       update_avg, draw_avg, n);
                break;
            }
        }
        // Sync underwater flag to vehicles (controls Y clamp in vehicle_update)
        for (int i = 0; i < vehicle_count; i++)
            vehicles[i].is_underwater = scene.is_underwater;

        // Missile mode controls
        if (missile_mode) {
            if (IsKeyPressed(KEY_SPACE)) missile_paused = !missile_paused;
        }

        // Missile barrage: generate synthetic flight data
        if (missile_mode) {
            float dt = missile_paused ? 0.0f : GetFrameTime();
            missile_time += dt;

            // Loop the animation
            float total_anim = 10.0f;
            if (missile_time > total_anim) {
                missile_time = 0.0f;
                for (int i = 0; i < vehicle_count; i++)
                    vehicle_reset_trail(&vehicles[i]);
            }

            Vector3 launch = { 0.0f, 15.0f, 0.0f };
            Vector3 target_center = { 40.0f, 20.0f, 30.0f };

            // Compute target VTOL's actual position (slow orbit)
            float tgt_a = missile_time * 0.3f;
            Vector3 target = {
                target_center.x + cosf(tgt_a) * 3.0f,
                target_center.y + sinf(missile_time * 0.5f) * 1.0f,
                target_center.z + sinf(tgt_a) * 3.0f
            };

            // Update missiles — they track the VTOL's live position
            for (int i = 0; i < MISSILE_COUNT; i++) {
                float t = missile_time - (float)i * 0.05f;
                if (t < 0.0f) t = 0.0f;

                hil_state_t state = {0};
                missile_barrage_state(&missile_params[i], t, launch, target, &state);
                vehicle_update(&vehicles[i], &state, NULL);
                vehicles[i].active = true;
            }

            // Target VTOL state
            {
                hil_state_t tgt_state = {0};
                float tx = target.x;
                float tz = target.z;
                float ty = target.y;

                double base_lat = 47.397742;
                double base_lon = 8.545594;
                double base_alt = 489.4;
                double meters_per_deg_lat = 111132.0;
                double meters_per_deg_lon = 111132.0 * cos(base_lat * M_PI / 180.0);

                tgt_state.lat = (int32_t)((base_lat + (double)tz / meters_per_deg_lat) * 1e7);
                tgt_state.lon = (int32_t)((base_lon + (double)tx / meters_per_deg_lon) * 1e7);
                tgt_state.alt = (int32_t)((base_alt + (double)ty) * 1000.0);
                tgt_state.vx = 0; tgt_state.vy = 0; tgt_state.vz = 0;
                tgt_state.quaternion[0] = cosf(tgt_a * 0.5f);
                tgt_state.quaternion[1] = 0;
                tgt_state.quaternion[2] = sinf(tgt_a * 0.5f);
                tgt_state.quaternion[3] = 0;
                tgt_state.valid = true;
                tgt_state.time_usec = (uint64_t)(missile_time * 1e6);

                vehicle_update(&vehicles[MISSILE_COUNT], &tgt_state, NULL);
                vehicles[MISSILE_COUNT].active = true;
            }
        }

        // Bench replay playback controls (only when not auto-benchmarking)
        if (bench_mode && bench_replays && bench_duration <= 0.0f) {
            if (IsKeyPressed(KEY_SPACE)) bench_replay_paused = !bench_replay_paused;
            if (IsKeyPressed(KEY_EQUAL)) {
                if (bench_replay_speed < 0.5f) bench_replay_speed = 0.5f;
                else if (bench_replay_speed < 1.0f) bench_replay_speed = 1.0f;
                else if (bench_replay_speed < 2.0f) bench_replay_speed = 2.0f;
                else if (bench_replay_speed < 4.0f) bench_replay_speed = 4.0f;
                else if (bench_replay_speed < 8.0f) bench_replay_speed = 8.0f;
                else bench_replay_speed = 16.0f;
                printf("Replay speed: %.1fx\n", bench_replay_speed);
            }
            if (IsKeyPressed(KEY_MINUS)) {
                if (bench_replay_speed > 8.0f) bench_replay_speed = 8.0f;
                else if (bench_replay_speed > 4.0f) bench_replay_speed = 4.0f;
                else if (bench_replay_speed > 2.0f) bench_replay_speed = 2.0f;
                else if (bench_replay_speed > 1.0f) bench_replay_speed = 1.0f;
                else if (bench_replay_speed > 0.5f) bench_replay_speed = 0.5f;
                else bench_replay_speed = 0.25f;
                printf("Replay speed: %.2fx\n", bench_replay_speed);
            }
            if (IsKeyPressed(KEY_R)) {
                for (int i = 0; i < bench_count; i++) {
                    ulog_replay_seek(&bench_replays[i], 0.0f);
                    vehicle_reset_trail(&vehicles[i]);
                }
                printf("Replay: rewound to start\n");
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float pos = (float)bench_replays[0].wall_accum + step;
                for (int i = 0; i < bench_count; i++) {
                    ulog_replay_seek(&bench_replays[i], pos);
                    vehicle_reset_trail(&vehicles[i]);
                }
                printf("Replay: skip to %.0fs\n", pos);
            }
            if (IsKeyPressed(KEY_LEFT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float pos = (float)bench_replays[0].wall_accum - step;
                if (pos < 0.0f) pos = 0.0f;
                for (int i = 0; i < bench_count; i++) {
                    ulog_replay_seek(&bench_replays[i], pos);
                    vehicle_reset_trail(&vehicles[i]);
                }
                printf("Replay: skip to %.0fs\n", pos);
            }
        }

        // Benchmark mode: ULG replay or synthetic figure-eights
        if (bench_mode) {
            double t_update_start = qpc_now_ms();
            float bdt = GetFrameTime();
            if (bench_replay_paused && bench_replays) bdt = 0.0f;
            bench_time += bdt;
            if (bench_replays) {
                // ULG replay with X/Z offset per drone
                double ref_lat_rad = origin_lat * (M_PI / 180.0);
                double mpd_lat = 111132.0;
                double mpd_lon = 111132.0 * cos(ref_lat_rad);
                for (int i = 0; i < bench_count; i++) {
                    ulog_replay_advance(&bench_replays[i], bdt, bench_replay_speed, true, true);
                    hil_state_t state = bench_replays[i].state;
                    // Apply grid offset to lat/lon
                    state.lat += (int32_t)(bench_offset_z[i] / mpd_lat * 1e7);
                    state.lon += (int32_t)(bench_offset_x[i] / mpd_lon * 1e7);
                    vehicle_update(&vehicles[i], &state, NULL);
                    vehicles[i].active = state.valid;
                }
            } else {
                for (int i = 0; i < bench_count; i++) {
                    hil_state_t state = {0};
                    bench_state(&bench_params[i], bench_time, &state);
                    vehicle_update(&vehicles[i], &state, NULL);
                    vehicles[i].active = true;
                }
            }
            double t_update_end = qpc_now_ms();
            if (bench_elapsed > 2.0f) {
                bench_update_sum += (t_update_end - t_update_start);
                bench_phase_count++;
            }
        }

        // Fake mode: WASDQE flight control
        if (fake_mode) {
            static float fake_x = 0.0f, fake_y = 10.0f, fake_z = 0.0f;
            static float fake_yaw = 0.0f;   // radians
            static float fake_pitch = 0.0f; // radians
            static float fake_roll = 0.0f;
            float dt = GetFrameTime();
            float move_speed = 20.0f * dt;  // m/s
            float rot_speed = 2.0f * dt;    // rad/s

            // Shift = boost
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
                move_speed *= 3.0f;

            // Rotation: arrow keys
            if (IsKeyDown(KEY_LEFT))  fake_yaw -= rot_speed;
            if (IsKeyDown(KEY_RIGHT)) fake_yaw += rot_speed;

            // Movement relative to yaw
            float fwd_x = sinf(fake_yaw);
            float fwd_z = cosf(fake_yaw);
            float right_x = cosf(fake_yaw);
            float right_z = -sinf(fake_yaw);

            // Tilt for visual feedback
            float target_pitch = 0.0f, target_roll = 0.0f;

            if (IsKeyDown(KEY_W)) { fake_x += fwd_x * move_speed; fake_z += fwd_z * move_speed; target_pitch = -0.3f; }
            if (IsKeyDown(KEY_S)) { fake_x -= fwd_x * move_speed; fake_z -= fwd_z * move_speed; target_pitch = 0.3f; }
            if (IsKeyDown(KEY_D)) { fake_x += right_x * move_speed; fake_z += right_z * move_speed; target_roll = 0.4f; }
            if (IsKeyDown(KEY_A)) { fake_x -= right_x * move_speed; fake_z -= right_z * move_speed; target_roll = -0.4f; }
            if (IsKeyDown(KEY_E)) fake_y += move_speed;
            if (IsKeyDown(KEY_Q)) fake_y -= move_speed;

            // Smooth tilt
            fake_pitch += (target_pitch - fake_pitch) * 5.0f * dt;
            fake_roll += (target_roll - fake_roll) * 5.0f * dt;

            // Build hil_state
            double meters_per_deg_lat = 111132.0;
            double meters_per_deg_lon = 111132.0 * cos(origin_lat * M_PI / 180.0);

            hil_state_t state = {0};
            state.lat = (int32_t)((origin_lat + (double)fake_z / meters_per_deg_lat) * 1e7);
            state.lon = (int32_t)((origin_lon + (double)fake_x / meters_per_deg_lon) * 1e7);
            state.alt = (int32_t)((origin_alt + (double)fake_y) * 1000.0);

            // Velocity for HUD readout
            float vx = 0, vy = 0, vz = 0;
            if (IsKeyDown(KEY_W)) { vx += fwd_z * 20.0f; vy += fwd_x * 20.0f; }
            if (IsKeyDown(KEY_S)) { vx -= fwd_z * 20.0f; vy -= fwd_x * 20.0f; }
            if (IsKeyDown(KEY_D)) { vy += right_x * 20.0f; vx += right_z * 20.0f; }  // NED east
            if (IsKeyDown(KEY_A)) { vy -= right_x * 20.0f; vx -= right_z * 20.0f; }
            float vz_ned = 0;
            if (IsKeyDown(KEY_E)) vz_ned = -20.0f; // NED down = negative for climb
            if (IsKeyDown(KEY_Q)) vz_ned = 20.0f;
            state.vx = (int16_t)(vx * 100.0f);
            state.vy = (int16_t)(vy * 100.0f);
            state.vz = (int16_t)(vz_ned * 100.0f);
            float spd = sqrtf(vx*vx + vy*vy + vz_ned*vz_ned);
            state.ind_airspeed = (uint16_t)(spd * 100.0f);
            state.true_airspeed = state.ind_airspeed;

            // Quaternion from yaw/pitch/roll (ZYX convention)
            float cy = cosf(fake_yaw * 0.5f), sy = sinf(fake_yaw * 0.5f);
            float cp = cosf(fake_pitch * 0.5f), sp = sinf(fake_pitch * 0.5f);
            float cr = cosf(fake_roll * 0.5f), sr = sinf(fake_roll * 0.5f);
            state.quaternion[0] = cr*cp*cy + sr*sp*sy;
            state.quaternion[1] = sr*cp*cy - cr*sp*sy;
            state.quaternion[2] = cr*sp*cy + sr*cp*sy;
            state.quaternion[3] = cr*cp*sy - sr*sp*cy;

            state.valid = true;
            state.time_usec = (uint64_t)(GetTime() * 1e6);

            vehicle_update(&vehicles[0], &state, NULL);
            vehicles[0].active = true;
        }

        // Poll all data sources and update vehicles
        for (int i = 0; i < vehicle_count && !missile_mode && !bench_mode && !fake_mode; i++) {
            data_source_poll(&sources[i], GetFrameTime());

            // Reset trail and origin on reconnect
            if (sources[i].connected && !was_connected[i]) {
                vehicle_reset_trail(&vehicles[i]);
                if (sources[i].mav_type != 0)
                    vehicle_set_type(&vehicles[i], sources[i].mav_type);
                if (!origin_specified && vehicle_count == 1) {
                    vehicles[i].origin_set = false;
                    vehicles[i].origin_wait_count = 0;
                }
            }
            was_connected[i] = sources[i].connected;

            vehicles[i].current_time = sources[i].playback.position_s;
            vehicle_update(&vehicles[i], &sources[i].state, &sources[i].home);
            vehicles[i].sysid = sources[i].sysid;

            // Detect position jump (new SITL connecting before disconnect timeout)
            if (vehicles[i].active && vehicles[i].trail_count > 0) {
                Vector3 delta = Vector3Subtract(vehicles[i].position, last_pos[i]);
                if (Vector3Length(delta) > 50.0f) {
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            last_pos[i] = vehicles[i].position;
        }

        // Lazy-resolve system marker positions once origin is established
        if (is_replay && !sys_markers_resolved && sys_marker_count > 0 && vehicles[0].origin_set) {
            ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
            float saved_pos = sources[0].playback.position_s;
            // Resolve positions; discard markers with no valid GPS (lat=0, lon=0)
            int valid = 0;
            for (int i = 0; i < sys_marker_count; i++) {
                ulog_replay_seek(ctx, sys_marker_times[i]);
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
                // Skip markers with no valid position
                if (sources[0].state.lat == 0 && sources[0].state.lon == 0) continue;
                Vector3 mp = vehicles[0].position;
                if (mp.x == 0.0f && mp.y == 0.0f && mp.z == 0.0f) continue;
                float mdist = sqrtf(mp.x * mp.x + mp.y * mp.y + mp.z * mp.z);
                if (mdist > 5000.0f) continue;
                sys_marker_times[valid] = sys_marker_times[i];
                memcpy(sys_marker_labels[valid], sys_marker_labels[i], MARKER_LABEL_MAX);
                sys_marker_positions[valid] = vehicles[0].position;
                sys_marker_roll[valid] = vehicles[0].roll_deg;
                sys_marker_pitch[valid] = vehicles[0].pitch_deg;
                sys_marker_vert[valid] = vehicles[0].vertical_speed;
                sys_marker_speed[valid] = sqrtf(vehicles[0].ground_speed * vehicles[0].ground_speed +
                                                 vehicles[0].vertical_speed * vehicles[0].vertical_speed);
                valid++;
            }
            sys_marker_count = valid;

            // Pre-compute entire flight trail from ULog position data
            if (!precomp_ready) {
                precomp_trail = calloc(PRECOMP_TRAIL_MAX, sizeof(Vector3));
                precomp_roll  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_pitch = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_vert  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_speed = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_time  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));

                if (precomp_trail && precomp_roll && precomp_pitch &&
                    precomp_vert && precomp_speed && precomp_time) {
                    double lat0 = vehicles[0].lat0;
                    double lon0 = vehicles[0].lon0;
                    double alt0 = vehicles[0].alt0;
                    float cos_lat0 = (float)cos(lat0);
                    Vector3 prev_pos = {0};
                    bool prev_valid = false;
                    int pc = 0;

                    ulog_replay_seek(ctx, 0.0f);
                    float step = 0.2f;
                    while (pc < PRECOMP_TRAIL_MAX) {
                        float prev_wall = (float)ctx->wall_accum;
                        bool ok = ulog_replay_advance(ctx, step, 1.0f, false, false);
                        if (!ok || (float)ctx->wall_accum <= prev_wall) break;

                        hil_state_t *st = &ctx->state;
                        if (!st->valid) continue;
                        if (st->lat == 0 && st->lon == 0) continue;

                        double lat = st->lat * 1e-7 * (M_PI / 180.0);
                        double lon = st->lon * 1e-7 * (M_PI / 180.0);
                        double alt = st->alt * 1e-3;
                        double ned_x = 6371000.0 * (lat - lat0);
                        double ned_y = 6371000.0 * (lon - lon0) * cos_lat0;
                        double ned_z = alt - alt0;
                        Vector3 pos = {
                            (float)ned_y,
                            (float)ned_z < 0.0f ? 0.0f : (float)ned_z,
                            (float)(-ned_x)
                        };

                        if (prev_valid) {
                            float dx = pos.x - prev_pos.x;
                            float dy = pos.y - prev_pos.y;
                            float dz = pos.z - prev_pos.z;
                            if (dx*dx + dy*dy + dz*dz < 0.25f) continue;
                        }

                        float qw = st->quaternion[0], qx = st->quaternion[1];
                        float qy = st->quaternion[2], qz = st->quaternion[3];
                        if (qw < 0) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
                        float roll = atan2f(2.0f*(qw*qx + qy*qz),
                                            1.0f - 2.0f*(qx*qx + qy*qy)) * RAD2DEG;
                        float sin_p = 2.0f*(qw*qy - qz*qx);
                        if (sin_p > 1.0f) sin_p = 1.0f;
                        if (sin_p < -1.0f) sin_p = -1.0f;
                        float pitch = asinf(sin_p) * RAD2DEG;
                        float vert_s = -st->vz * 0.01f;
                        float gs = sqrtf((float)st->vx*st->vx + (float)st->vy*st->vy) * 0.01f;
                        float spd = sqrtf(gs*gs + vert_s*vert_s);

                        precomp_trail[pc] = pos;
                        precomp_roll[pc] = roll;
                        precomp_pitch[pc] = pitch;
                        precomp_vert[pc] = vert_s;
                        precomp_speed[pc] = spd;
                        precomp_time[pc] = (float)ctx->wall_accum;
                        if (spd > precomp_speed_max) precomp_speed_max = spd;
                        pc++;
                        prev_pos = pos;
                        prev_valid = true;
                    }
                    precomp_count = pc;
                    precomp_ready = true;
                }
            }

            // Restore playback to where it was
            ulog_replay_seek(ctx, saved_pos);
            vehicle_reset_trail(&vehicles[0]);
            REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
            sys_markers_resolved = true;
        }

        // Check if any source is connected (for HUD)
        bool any_connected = fake_mode || missile_mode || bench_mode;
        for (int i = 0; i < vehicle_count && !any_connected; i++) {
            if (sources[i].connected) { any_connected = true; break; }
        }

        // Update HUD sim time from selected vehicle
        hud_update(&hud, sources[selected].state.time_usec,
                   sources[selected].connected, GetFrameTime());

        // Handle input (blocked during marker label entry)
        if (!marker_input_active) {
        scene_handle_input(&scene);

        // Help overlay toggle (? key = Shift+/)
        if (IsKeyPressed(KEY_SLASH) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            hud.show_help = !hud.show_help;
        }

        // Toggle HUD visibility
        if (IsKeyPressed(KEY_H)) {
            show_hud = !show_hud;
        }

        // Cycle trail mode: off → trail → speed ribbon
        if (IsKeyPressed(KEY_T)) {
            trail_mode = (trail_mode + 1) % 3;
        }

        // Toggle classic/modern arm colors
        if (IsKeyPressed(KEY_K)) {
            classic_colors = !classic_colors;
        }

        // Toggle ground track projection
        if (IsKeyPressed(KEY_G)) {
            show_ground_track = !show_ground_track;
        }

        // Toggle debug panel (Ctrl+D)
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_D)) {
            dbg_panel.visible = !dbg_panel.visible;
        }

        // Toggle ortho panel
        if (IsKeyPressed(KEY_O)) {
            ortho.visible = !ortho.visible;
        }

        // Cycle model for selected vehicle
        // Cycle model: M = within group, Shift+M = all models
        if (IsKeyPressed(KEY_M)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                int next = (vehicles[selected].model_idx + 1) % vehicle_model_count;
                vehicle_load_model(&vehicles[selected], next);
            } else {
                vehicle_cycle_model(&vehicles[selected]);
            }
        }

        // Vehicle selection input
        if (vehicle_count > 1) {
            if (IsKeyPressed(KEY_TAB)) {
                // Cycle to next connected vehicle, clear pins
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_LEFT_BRACKET) && !is_replay) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int prev = (selected - j + vehicle_count) % vehicle_count;
                    if (sources[prev].connected) { selected = prev; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET) && !is_replay) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            // Number keys 1-9: plain = select + clear pins, SHIFT = toggle pin
            for (int k = KEY_ONE; k <= KEY_NINE; k++) {
                if (IsKeyPressed(k)) {
                    int idx = k - KEY_ONE;
                    if (idx >= vehicle_count) continue;

                    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                        // SHIFT+number: toggle pin
                        if (idx == selected) continue;  // can't pin the primary

                        // Check if already pinned
                        int found = -1;
                        for (int p = 0; p < hud.pinned_count; p++) {
                            if (hud.pinned[p] == idx) { found = p; break; }
                        }

                        if (found >= 0) {
                            // Unpin: shift remaining
                            for (int p = found; p < hud.pinned_count - 1; p++)
                                hud.pinned[p] = hud.pinned[p + 1];
                            hud.pinned_count--;
                            hud.pinned[hud.pinned_count] = -1;
                        } else if (hud.pinned_count < HUD_MAX_PINNED && hud.pinned_count < vehicle_count - 1) {
                            hud.pinned[hud.pinned_count++] = idx;
                        }
                    } else if (!IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) {
                        // Plain number: switch primary, clear pins
                        selected = idx;
                        hud.pinned_count = 0;
                        memset(hud.pinned, -1, sizeof(hud.pinned));
                    }
                    // Note: Ctrl+number is reserved for 1988 easter egg
                }
            }
        }
        } // end !marker_input_active guard

        // Marker label text input — consumes all keyboard while active
        if (marker_input_active) {
            int ch;
            while ((ch = GetCharPressed()) != 0) {
                if (marker_input_len < MARKER_LABEL_MAX - 1 && ch >= 32 && ch < 127) {
                    marker_input_buf[marker_input_len++] = (char)ch;
                    marker_input_buf[marker_input_len] = '\0';
                }
            }
            if (IsKeyPressed(KEY_BACKSPACE) && marker_input_len > 0) {
                marker_input_buf[--marker_input_len] = '\0';
            }
            if (IsKeyPressed(KEY_ENTER)) {
                if (marker_input_target >= 0 && marker_input_target < marker_count) {
                    memcpy(marker_labels[marker_input_target], marker_input_buf, MARKER_LABEL_MAX);
                    // Re-sync drone to this marker's exact position
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, marker_times[marker_input_target]);
                    REPLAY_SYNC_AFTER_SEEK(rctx, &sources[0], &vehicles[0]);
                }
                current_marker = marker_input_target;
                marker_input_active = false;
            }
            if (IsKeyPressed(KEY_ESCAPE)) {
                if (marker_input_target >= 0 && marker_input_target < marker_count) {
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, marker_times[marker_input_target]);
                    REPLAY_SYNC_AFTER_SEEK(rctx, &sources[0], &vehicles[0]);
                }
                current_marker = marker_input_target;
                marker_input_active = false;
            }
        }

        // Replay playback controls
        if (is_replay && !marker_input_active) {
            bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[0].connected) {
                    // Replay ended — restart from beginning
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, 0.0f);
                    sources[0].connected = true;
                    sources[0].playback.paused = false;
                    vehicle_reset_trail(&vehicles[0]);
                    vehicles[0].origin_set = false;
                    vehicles[0].origin_wait_count = 0;
                    REPLAY_SYNC_AFTER_SEEK(rctx, &sources[0], &vehicles[0]);
                } else {
                    sources[0].playback.paused = !sources[0].playback.paused;
                }
            }
            if (IsKeyPressed(KEY_L) && shift) {
                sources[0].playback.looping = !sources[0].playback.looping;
            }
            if (IsKeyPressed(KEY_L) && !shift && (last_marker_drop_idx < 0 || GetTime() - last_marker_drop_time >= 0.5)) {
                show_marker_labels = !show_marker_labels;
            }
            if (IsKeyPressed(KEY_I)) {
                sources[0].playback.interpolation = !sources[0].playback.interpolation;
                printf("Interpolation: %s\n", sources[0].playback.interpolation ? "ON" : "OFF");
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                float *spd = &sources[0].playback.speed;
                if (*spd < 0.5f) *spd = 0.5f;
                else if (*spd < 1.0f) *spd = 1.0f;
                else if (*spd < 2.0f) *spd = 2.0f;
                else if (*spd < 4.0f) *spd = 4.0f;
                else if (*spd < 8.0f) *spd = 8.0f;
                else *spd = 16.0f;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float *spd = &sources[0].playback.speed;
                if (*spd > 8.0f) *spd = 8.0f;
                else if (*spd > 4.0f) *spd = 4.0f;
                else if (*spd > 2.0f) *spd = 2.0f;
                else if (*spd > 1.0f) *spd = 1.0f;
                else if (*spd > 0.5f) *spd = 0.5f;
                else *spd = 0.25f;
            }
            if (IsKeyPressed(KEY_R)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                ulog_replay_seek(ctx, 0.0f);
                sources[0].connected = true;
                sources[0].playback.paused = false;
                vehicle_reset_trail(&vehicles[0]);
                vehicles[0].origin_set = false;
                vehicles[0].origin_wait_count = 0;
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
                marker_count = 0;
                current_marker = -1;
            }

            // Timeline scrubbing: 3 levels of granularity
            // Shift+Arrow = single frame step (~20ms), Ctrl+Shift = 1s, plain = 5s
            if (IsKeyPressed(KEY_RIGHT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                ulog_replay_seek(ctx, sources[0].playback.position_s + step);
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
            }
            if (IsKeyPressed(KEY_LEFT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                float target = sources[0].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                ulog_replay_seek(ctx, target);
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
            }

            // Frame markers: B = drop marker, B→L = drop + label, Shift+B = delete current
            if (IsKeyPressed(KEY_B) && vehicles[0].active && !marker_input_active) {
                if (shift) {
                    // Shift+B: delete currently highlighted marker
                    if (current_marker >= 0 && current_marker < marker_count) {
                        for (int m = current_marker; m < marker_count - 1; m++) {
                            marker_times[m] = marker_times[m + 1];
                            marker_positions[m] = marker_positions[m + 1];
                            memcpy(marker_labels[m], marker_labels[m + 1], MARKER_LABEL_MAX);
                            marker_roll[m] = marker_roll[m + 1];
                            marker_pitch[m] = marker_pitch[m + 1];
                            marker_vert[m] = marker_vert[m + 1];
                            marker_speed[m] = marker_speed[m + 1];
                        }
                        marker_count--;
                        if (current_marker >= marker_count) current_marker = marker_count - 1;
                        last_marker_drop_idx = -1;
                    }
                } else if (marker_count < MAX_MARKERS) {
                    // B: drop marker at current position
                    float t = sources[0].playback.position_s;
                    Vector3 pos = vehicles[0].position;
                    int insert = marker_count;
                    for (int m = 0; m < marker_count; m++) {
                        if (marker_times[m] > t) { insert = m; break; }
                    }
                    // Shift existing markers up
                    for (int m = marker_count; m > insert; m--) {
                        marker_times[m] = marker_times[m - 1];
                        marker_positions[m] = marker_positions[m - 1];
                        memcpy(marker_labels[m], marker_labels[m - 1], MARKER_LABEL_MAX);
                        marker_roll[m] = marker_roll[m - 1];
                        marker_pitch[m] = marker_pitch[m - 1];
                        marker_vert[m] = marker_vert[m - 1];
                        marker_speed[m] = marker_speed[m - 1];
                    }
                    marker_times[insert] = t;
                    marker_positions[insert] = pos;
                    marker_labels[insert][0] = '\0';
                    marker_roll[insert] = vehicles[0].roll_deg;
                    marker_pitch[insert] = vehicles[0].pitch_deg;
                    marker_vert[insert] = vehicles[0].vertical_speed;
                    marker_speed[insert] = sqrtf(vehicles[0].ground_speed * vehicles[0].ground_speed +
                                                  vehicles[0].vertical_speed * vehicles[0].vertical_speed);
                    marker_count++;
                    current_marker = insert;
                    sys_marker_selected = false;
                    current_sys_marker = -1;
                    last_marker_drop_time = GetTime();
                    last_marker_drop_idx = insert;
                }
            }

            // B→L chord: if L pressed within 0.5s of dropping a marker, open label input
            if (IsKeyPressed(KEY_L) && !marker_input_active && last_marker_drop_idx >= 0) {
                double elapsed = GetTime() - last_marker_drop_time;
                if (elapsed < 0.5) {
                    marker_input_active = true;
                    marker_input_target = last_marker_drop_idx;
                    marker_input_buf[0] = '\0';
                    marker_input_len = 0;
                    sources[0].playback.paused = true;
                    last_marker_drop_idx = -1;
                }
            }
            // Unified [/] cycling through both user markers and system markers (sorted by time)
            if ((IsKeyPressed(KEY_LEFT_BRACKET) || IsKeyPressed(KEY_RIGHT_BRACKET))
                && (marker_count > 0 || sys_marker_count > 0)) {
                // Build a merged list of all markers sorted by time
                // Each entry: time, index, is_sys
                int total = marker_count + sys_marker_count;
                typedef struct { float time; int idx; bool is_sys; } merged_marker_t;
                merged_marker_t merged[MAX_MARKERS + MAX_SYS_MARKERS];
                int mi = 0;
                for (int m = 0; m < marker_count; m++) {
                    merged[mi++] = (merged_marker_t){marker_times[m], m, false};
                }
                for (int m = 0; m < sys_marker_count; m++) {
                    merged[mi++] = (merged_marker_t){sys_marker_times[m], m, true};
                }
                // Simple insertion sort (small N)
                for (int a = 1; a < total; a++) {
                    merged_marker_t key = merged[a];
                    int b = a - 1;
                    while (b >= 0 && merged[b].time > key.time) {
                        merged[b + 1] = merged[b];
                        b--;
                    }
                    merged[b + 1] = key;
                }

                // Find current position in merged list
                int cur_merged = -1;
                if (sys_marker_selected && current_sys_marker >= 0) {
                    for (int m = 0; m < total; m++) {
                        if (merged[m].is_sys && merged[m].idx == current_sys_marker) {
                            cur_merged = m; break;
                        }
                    }
                } else if (!sys_marker_selected && current_marker >= 0) {
                    for (int m = 0; m < total; m++) {
                        if (!merged[m].is_sys && merged[m].idx == current_marker) {
                            cur_merged = m; break;
                        }
                    }
                }

                int target_merged = -1;
                if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                    if (cur_merged > 0) {
                        target_merged = cur_merged - 1;
                    } else if (cur_merged == 0) {
                        target_merged = total - 1;  // wrap to last
                    } else {
                        // Find nearest before playhead
                        float t = sources[0].playback.position_s;
                        for (int m = total - 1; m >= 0; m--) {
                            if (merged[m].time <= t) { target_merged = m; break; }
                        }
                        if (target_merged < 0) target_merged = total - 1;
                    }
                } else {
                    if (cur_merged >= 0 && cur_merged < total - 1) {
                        target_merged = cur_merged + 1;
                    } else if (cur_merged < 0) {
                        float t = sources[0].playback.position_s;
                        for (int m = 0; m < total; m++) {
                            if (merged[m].time >= t) { target_merged = m; break; }
                        }
                        if (target_merged < 0) target_merged = 0;
                    } else {
                        target_merged = 0;  // wrap to first
                    }
                }

                if (target_merged >= 0 && target_merged < total) {
                    merged_marker_t tgt = merged[target_merged];
                    float seek_time = tgt.time;
                    Vector3 seek_pos = tgt.is_sys ? sys_marker_positions[tgt.idx] : marker_positions[tgt.idx];

                    // Update selection state
                    sys_marker_selected = tgt.is_sys;
                    if (tgt.is_sys) {
                        current_sys_marker = tgt.idx;
                        current_marker = -1;
                    } else {
                        current_marker = tgt.idx;
                        current_sys_marker = -1;
                    }

                    if (shift) {
                        scene.cam_mode = CAM_MODE_FREE;
                        scene.free_track = true;
                        scene.camera.position = seek_pos;
                        scene.camera.target = vehicles[0].position;
                    } else {
                        ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;

                        // Seek and sync state (no vehicle_update yet — precomp trail first)
                        ulog_replay_seek(ctx, seek_time);
                        sources[0].state = ctx->state;
                        sources[0].home = ctx->home;
                        sources[0].playback.position_s = (float)ctx->wall_accum;
                        uint64_t _range = ctx->parser.end_timestamp - ctx->parser.start_timestamp;
                        if (_range > 0) sources[0].playback.progress = sources[0].playback.position_s / ((float)((double)_range / 1e6));
                        vehicles[0].current_time = sources[0].playback.position_s;

                        // Restore trail from pre-computed data up to seek_time
                        if (precomp_ready && precomp_count > 0) {
                            int lo = 0, hi = precomp_count - 1, cut = 0;
                            while (lo <= hi) {
                                int mid = (lo + hi) / 2;
                                if (precomp_time[mid] <= seek_time) {
                                    cut = mid + 1;
                                    lo = mid + 1;
                                } else {
                                    hi = mid - 1;
                                }
                            }
                            vehicle_t *v = &vehicles[0];
                            int n = cut;
                            if (n > v->trail_capacity) n = v->trail_capacity;
                            int src_start = cut - n;
                            v->trail_count = n;
                            v->trail_head = n % v->trail_capacity;
                            v->trail_speed_max = 0.0f;
                            for (int i = 0; i < n; i++) {
                                int si = src_start + i;
                                v->trail[i] = precomp_trail[si];
                                v->trail_roll[i] = precomp_roll[si];
                                v->trail_pitch[i] = precomp_pitch[si];
                                v->trail_vert[i] = precomp_vert[si];
                                v->trail_speed[i] = precomp_speed[si];
                                v->trail_time[i] = precomp_time[si];
                                if (precomp_speed[si] > v->trail_speed_max)
                                    v->trail_speed_max = precomp_speed[si];
                            }
                        }
                        // Update vehicle state and suppress jump detector on next frame
                        vehicle_update(&vehicles[0], &sources[0].state, &sources[0].home);
                        last_pos[0] = vehicles[0].position;
                    }
                }
            }
        }

        // Update debug panel
        debug_panel_update(&dbg_panel, GetFrameTime());


        // Update camera to follow selected vehicle
        scene_update_camera(&scene, vehicles[selected].position, vehicles[selected].rotation);

        // Render ortho views to textures (before main BeginDrawing)
        if (ortho.visible) {
            ortho_panel_update(&ortho, vehicles[selected].position);
            ortho_panel_render(&ortho, &scene, vehicles, vehicle_count,
                               selected, scene.view_mode, trail_mode);
        }

        // Render
        double t_draw_start = qpc_now_ms();
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            BeginMode3D(scene.camera);
                scene_draw(&scene);
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw(&vehicles[i], scene.view_mode, i == selected,
                                     trail_mode, show_ground_track, scene.camera.position,
                                     classic_colors, scene.is_underwater);
                    }
                }
                // Draw frame marker spheres during replay
                if (is_replay && marker_count > 0) {
                    vehicle_draw_markers(marker_positions, marker_labels, marker_count,
                                         sys_marker_selected ? -1 : current_marker,
                                         scene.camera.position, scene.camera,
                                         marker_roll, marker_pitch, marker_vert, marker_speed,
                                         vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
                }
                // Draw system marker cubes during replay
                if (is_replay && sys_marker_count > 0) {
                    vehicle_draw_sys_markers(sys_marker_positions, sys_marker_labels, sys_marker_count,
                                             sys_marker_selected ? current_sys_marker : -1,
                                             scene.camera.position,
                                             sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed,
                                             vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
                }
            EndMode3D();

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // Underwater edge rings overlay
            scene_draw_underwater_overlay(&scene, GetScreenWidth(), GetScreenHeight());

            // Marker labels (2D billboarded text, after EndMode3D)
            if (is_replay && marker_count > 0 && show_marker_labels) {
                vehicle_draw_marker_labels(marker_positions, marker_labels, marker_count,
                                           sys_marker_selected ? -1 : current_marker,
                                           scene.camera.position, scene.camera,
                                           hud.font_label, hud.font_value,
                                           marker_roll, marker_pitch, marker_vert, marker_speed,
                                           vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
            }
            // System marker labels
            if (is_replay && sys_marker_count > 0 && show_marker_labels) {
                vehicle_draw_sys_marker_labels(sys_marker_positions, sys_marker_labels, sys_marker_count,
                                               sys_marker_selected ? current_sys_marker : -1,
                                               scene.camera.position, scene.camera,
                                               hud.font_label, hud.font_value,
                                               sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed,
                                               vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
            }

            // HUD
            if (show_hud) {
                hud_draw(&hud, vehicles, sources, vehicle_count,
                         selected, GetScreenWidth(), GetScreenHeight(),
                         scene.view_mode, scene.is_underwater, trail_mode,
                         marker_times, marker_labels, marker_count,
                         sys_marker_selected ? -1 : current_marker,
                         marker_roll, marker_pitch, marker_vert, marker_speed,
                         vehicles[0].trail_speed_max,
                         sys_marker_times, sys_marker_labels,
                         sys_marker_count, current_sys_marker, sys_marker_selected,
                         sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed);
            }

            // Free camera indicator (top right) — shows marker name if at a marker
            if (scene.cam_mode == CAM_MODE_FREE) {
                int sw = GetScreenWidth();
                float s = powf((float)GetScreenHeight() / 720.0f, 0.7f);
                float fs = 14 * s;
                char fc_buf[80];
                if (sys_marker_selected && current_sys_marker >= 0) {
                    snprintf(fc_buf, sizeof(fc_buf), "FREE CAM  %s", sys_marker_labels[current_sys_marker]);
                } else if (!sys_marker_selected && current_marker >= 0 && marker_labels[current_marker][0] != '\0') {
                    snprintf(fc_buf, sizeof(fc_buf), "FREE CAM  %d: %s", current_marker + 1, marker_labels[current_marker]);
                } else if (!sys_marker_selected && current_marker >= 0) {
                    snprintf(fc_buf, sizeof(fc_buf), "FREE CAM  Marker %d", current_marker + 1);
                } else {
                    snprintf(fc_buf, sizeof(fc_buf), "FREE CAM");
                }
                Vector2 tw = MeasureTextEx(hud.font_label, fc_buf, fs, 0.5f);
                float px = 8 * s, py = 4 * s;
                float rx = sw - tw.x - px * 2 - 10 * s;
                float ry = 10 * s;
                // Marker-colored text if at a marker, otherwise default
                Color fc_text;
                bool has_marker_col = false;
                if (sys_marker_selected && current_sys_marker >= 0) {
                    fc_text = vehicle_marker_color(sys_marker_roll[current_sys_marker],
                                                   sys_marker_pitch[current_sys_marker],
                                                   sys_marker_vert[current_sys_marker],
                                                   sys_marker_speed[current_sys_marker],
                                                   vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
                    has_marker_col = true;
                } else if (!sys_marker_selected && current_marker >= 0) {
                    fc_text = vehicle_marker_color(marker_roll[current_marker],
                                                   marker_pitch[current_marker],
                                                   marker_vert[current_marker],
                                                   marker_speed[current_marker],
                                                   vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
                    has_marker_col = true;
                }
                if (has_marker_col) {
                    if (scene.view_mode == VIEW_SNOW) {
                        fc_text.r = (unsigned char)(fc_text.r * 0.55f);
                        fc_text.g = (unsigned char)(fc_text.g * 0.55f);
                        fc_text.b = (unsigned char)(fc_text.b * 0.55f);
                    } else {
                        fc_text.r = (unsigned char)(fc_text.r + (230 - fc_text.r) * 0.7f);
                        fc_text.g = (unsigned char)(fc_text.g + (230 - fc_text.g) * 0.7f);
                        fc_text.b = (unsigned char)(fc_text.b + (230 - fc_text.b) * 0.7f);
                    }
                } else {
                    fc_text = scene.view_mode == VIEW_SNOW ? (Color){60, 65, 75, 255} : (Color){200, 208, 218, 255};
                }
                Color fc_bg = scene.view_mode == VIEW_SNOW ? (Color){220, 222, 226, 220} : (Color){10, 14, 20, 220};
                DrawRectangleRounded(
                    (Rectangle){rx, ry, tw.x + px * 2, tw.y + py * 2},
                    0.3f, 6, fc_bg);
                DrawTextEx(hud.font_label, fc_buf,
                           (Vector2){rx + px, ry + py}, fs, 0.5f, fc_text);
            }

            // Debug panel
            {
                int active_count = 0;
                int total_trail = 0;
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active) active_count++;
                    total_trail += vehicles[i].trail_count;
                }
                debug_panel_draw(&dbg_panel, GetScreenWidth(), GetScreenHeight(),
                                 scene.view_mode, hud.font_label,
                                 vehicle_count, active_count, total_trail);
            }

            // Ortho panel overlay
            int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
            ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.view_mode, hud.font_label);

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.view_mode, hud.font_label);

            // Marker label input overlay (view-mode-aware)
            if (marker_input_active) {
                int sw = GetScreenWidth(), sh = GetScreenHeight();
                float s = powf(sh / 720.0f, 0.7f);

                // View-mode colors
                Color scrim_col, box_bg, box_border, prompt_col, hint_col;
                Color field_bg, field_border, text_col, cursor_col;
                if (scene.view_mode == VIEW_SNOW) {
                    scrim_col    = (Color){255, 255, 255, 140};
                    box_bg       = (Color){248, 248, 250, 240};
                    box_border   = (Color){15, 15, 20, 120};
                    prompt_col   = (Color){60, 65, 75, 255};
                    hint_col     = (Color){120, 125, 135, 200};
                    field_bg     = (Color){235, 236, 240, 255};
                    field_border = (Color){15, 15, 20, 80};
                    text_col     = (Color){10, 10, 15, 255};
                    cursor_col   = (Color){15, 15, 20, 220};
                } else if (scene.view_mode == VIEW_1988) {
                    scrim_col    = (Color){5, 0, 15, 160};
                    box_bg       = (Color){5, 5, 16, 240};
                    box_border   = (Color){255, 20, 100, 140};
                    prompt_col   = (Color){255, 20, 100, 200};
                    hint_col     = (Color){180, 60, 120, 160};
                    field_bg     = (Color){12, 8, 24, 255};
                    field_border = (Color){255, 20, 100, 100};
                    text_col     = (Color){255, 220, 60, 255};
                    cursor_col   = (Color){255, 20, 100, 220};
                } else if (scene.view_mode == VIEW_REZ) {
                    scrim_col    = (Color){0, 0, 0, 150};
                    box_bg       = (Color){8, 8, 12, 235};
                    box_border   = (Color){0, 204, 218, 100};
                    prompt_col   = (Color){0, 204, 218, 200};
                    hint_col     = (Color){0, 140, 150, 160};
                    field_bg     = (Color){4, 4, 8, 255};
                    field_border = (Color){0, 204, 218, 80};
                    text_col     = (Color){200, 208, 218, 255};
                    cursor_col   = (Color){0, 204, 218, 220};
                } else {
                    scrim_col    = (Color){0, 0, 0, 140};
                    box_bg       = (Color){10, 14, 20, 235};
                    box_border   = (Color){0, 180, 204, 100};
                    prompt_col   = (Color){140, 150, 170, 255};
                    hint_col     = (Color){90, 95, 110, 200};
                    field_bg     = (Color){6, 8, 12, 255};
                    field_border = (Color){50, 55, 70, 180};
                    text_col     = WHITE;
                    cursor_col   = (Color){0, 255, 255, 220};
                }

                DrawRectangle(0, 0, sw, sh, scrim_col);

                float box_w = 520 * s, box_h = 110 * s;
                float bx = (sw - box_w) / 2, by = (sh - box_h) / 2;
                Rectangle box = {bx, by, box_w, box_h};

                DrawRectangleRounded(box, 0.06f, 8, box_bg);
                DrawRectangleRoundedLinesEx(box, 0.06f, 8, 1.5f * s, box_border);

                // Prompt label
                float prompt_fs = 15 * s;
                DrawTextEx(hud.font_label, "MARKER LABEL",
                           (Vector2){bx + 16 * s, by + 12 * s}, prompt_fs, 0.5f, prompt_col);

                // Hint text
                float hint_fs = 12 * s;
                float hint_w = MeasureTextEx(hud.font_label, "Enter to confirm  |  Esc to cancel", hint_fs, 0.5f).x;
                DrawTextEx(hud.font_label, "Enter to confirm  |  Esc to cancel",
                           (Vector2){bx + box_w - hint_w - 16 * s, by + 12 * s}, hint_fs, 0.5f, hint_col);

                // Input field
                float field_x = bx + 16 * s, field_y = by + 40 * s;
                float field_w = box_w - 32 * s, field_h = 44 * s;
                DrawRectangleRounded((Rectangle){field_x, field_y, field_w, field_h},
                                     0.08f, 6, field_bg);
                DrawRectangleRoundedLinesEx((Rectangle){field_x, field_y, field_w, field_h},
                                     0.08f, 6, 1.0f * s, field_border);

                // Input text
                float input_fs = 20 * s;
                float text_y = field_y + (field_h - input_fs) / 2;
                DrawTextEx(hud.font_value, marker_input_buf,
                           (Vector2){field_x + 12 * s, text_y}, input_fs, 0.5f, text_col);

                // Blinking cursor
                Vector2 tw = MeasureTextEx(hud.font_value, marker_input_buf, input_fs, 0.5f);
                if ((int)(GetTime() * 2.0) % 2 == 0) {
                    float cx = field_x + 12 * s + tw.x + 2;
                    DrawRectangle((int)cx, (int)(text_y), (int)(2 * s), (int)input_fs, cursor_col);
                }
            }

        EndDrawing();
        if (bench_mode && bench_duration > 0.0f && bench_elapsed > 2.0f) {
            double t_draw_end = qpc_now_ms();
            bench_draw_sum += (t_draw_end - t_draw_start);
        }
    }

    // Cleanup bench replays
    if (bench_replays) {
        for (int i = 0; i < bench_count; i++)
            ulog_replay_close(&bench_replays[i]);
        free(bench_replays);
    }

    // Cleanup
    ortho_panel_cleanup(&ortho);
    hud_cleanup(&hud);
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
        if (sources[i].ops) data_source_close(&sources[i]);
    }
    scene_cleanup(&scene);
    free(precomp_trail); free(precomp_roll); free(precomp_pitch);
    free(precomp_vert); free(precomp_speed); free(precomp_time);
    CloseWindow();

    return 0;
}
