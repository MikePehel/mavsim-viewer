#include "data_source.h"
#include "ulog_replay.h"
#include "terrain/terrain_params.h"
#include "ulog/terrain_params_extract.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static void ulog_poll(data_source_t *ds, float dt) {
    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)ds->impl;

    if (!ds->playback.paused) {
        bool still_playing = ulog_replay_advance(
            ctx, dt, ds->playback.speed, ds->playback.looping,
            ds->playback.interpolation);
        ds->connected = still_playing || ds->playback.looping;
    }

    // Copy state (even when paused, so HUD shows frozen data)
    ds->state = ctx->state;
    ds->home = ctx->home;
    ds->mav_type = ctx->vehicle_type;
    ds->playback.current_nav_state = ctx->current_nav_state;
    ds->ref_rejected = ctx->ref_rejected;
    ds->playback.takeoff_conf = ctx->takeoff_conf;
    ds->playback.takeoff_time_s = ctx->takeoff_time_s;
    ds->playback.takeoff_detected = ctx->takeoff_detected;
    ds->playback.home_from_topic = ctx->home_from_topic;
    ds->playback.time_offset_s = (float)ctx->time_offset_s;
    ds->playback.mode_changes = (const playback_mode_change_t *)ctx->mode_changes;
    ds->playback.mode_change_count = ctx->mode_change_count;
    ds->playback.statustext = &ctx->statustext;

    // Update playback progress
    uint64_t range = ctx->parser.end_timestamp - ctx->parser.start_timestamp;
    if (range > 0) {
        ds->playback.duration_s = (float)((double)range / 1e6);
        ds->playback.position_s = (float)ctx->wall_accum;
        if (ds->playback.position_s > ds->playback.duration_s)
            ds->playback.position_s = ds->playback.duration_s;
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
    }
}

static void ulog_close(data_source_t *ds) {
    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)ds->impl;
    if (ctx) {
        ulog_replay_close(ctx);
        free(ctx);
        ds->impl = NULL;
    }
}

static void ulog_seek(data_source_t *ds, float target_s) {
    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)ds->impl;
    ulog_replay_seek(ctx, target_s);

    // Sync data_source state from replay context after seek
    ds->state = ctx->state;
    ds->home = ctx->home;
    ds->playback.position_s = (float)ctx->wall_accum;
    uint64_t range = ctx->parser.end_timestamp - ctx->parser.start_timestamp;
    if (range > 0) {
        ds->playback.duration_s = (float)((double)range / 1e6);
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
    }
}

static void ulog_set_time_offset(data_source_t *ds, double offset_s) {
    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)ds->impl;
    ctx->time_offset_s = offset_s;
    ds->playback.time_offset_s = (float)offset_s;
}

static const data_source_ops_t ulog_ops = {
    .poll = ulog_poll,
    .seek = ulog_seek,
    .set_time_offset = ulog_set_time_offset,
    .close = ulog_close,
};

int data_source_ulog_create(data_source_t *ds, const char *filepath) {
    memset(ds, 0, sizeof(*ds));
    ds->ops = &ulog_ops;

    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)calloc(1, sizeof(ulog_replay_ctx_t));
    if (!ctx) return -1;

    int ret = ulog_replay_init(ctx, filepath);
    if (ret != 0) {
        free(ctx);
        return ret;
    }

    /*
     * Pull SIH_TERR_* out of the log's parameter snapshot and push them
     * into the shared terrain library. Non-SIH logs return false here;
     * apply with defaults so the library is in a known flat-baseline
     * state regardless of which log was loaded previously.
     */
    HawkeyeTerrainParams terr;
    bool from_log = ulog_extract_terrain_params(filepath, &terr);
    if (!from_log) {
        terr = hawkeye_terrain_params_default();
    }
    hawkeye_terrain_apply_params(&terr);

    /* Mirror the live path's [terrain] applied line (terrain_params_listener.c)
     * so replay is diagnosable from stdout too — without it a log that renders
     * flat is indistinguishable from a log whose params never got read. */
    printf("[terrain] replay: mode=%d amp=%.3f freq=%.4f seed=%d plane=%.3f%s\n",
           terr.mode, terr.amp, terr.freq, terr.seed, terr.plane_deg,
           from_log ? "" : " (no SIH_TERR_* in log — flat defaults)");

    ds->impl = ctx;
    ds->home = ctx->home;       // pre-scanned home for conflict detection
    ds->mav_type = ctx->vehicle_type;
    ds->connected = true;
    ds->sysid = 1;
    ds->playback.speed = 1.0f;
    ds->playback.looping = false;
    ds->playback.paused = false;
    ds->playback.interpolation = true;
    ds->playback.takeoff_conf = ctx->takeoff_conf;
    ds->playback.takeoff_time_s = ctx->takeoff_time_s;
    ds->playback.takeoff_detected = ctx->takeoff_detected;
    ds->playback.home_from_topic = ctx->home_from_topic;
    ds->playback.mode_changes = (const playback_mode_change_t *)ctx->mode_changes;
    ds->playback.mode_change_count = ctx->mode_change_count;
    ds->playback.correlation = NAN;
    ds->playback.rmse = NAN;

    // Compute duration from parser timestamps
    uint64_t range = ctx->parser.end_timestamp - ctx->parser.start_timestamp;
    if (range > 0)
        ds->playback.duration_s = (float)((double)range / 1e6);

    return 0;
}
