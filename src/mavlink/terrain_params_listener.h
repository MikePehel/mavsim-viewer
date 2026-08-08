/**
 * @file terrain_params_listener.h
 *
 * Live-SIH path for HawkeyeTerrainParams ingestion. Subscribes to
 * incoming MAVLink PARAM_VALUE messages, filters for the seven
 * SIH_TERR_* parameters, and forwards a populated struct to
 * hawkeye_terrain_apply_params() when:
 *
 *   - the initial snapshot completes (all seven SIH_TERR_* seen at
 *     least once, in any order), OR
 *   - any single value changes after the initial snapshot.
 *
 * State is module-internal — one global accumulator per process. The
 * integration site (the MAVLink receive dispatch) calls
 * terrain_params_on_param_value() for every PARAM_VALUE message it
 * decodes; the listener decides whether to act.
 *
 * The companion ULog path is in src/ulog/terrain_params_extract.{c,h}.
 * The two paths fill the same HawkeyeTerrainParams shape and feed the
 * same apply function; only the source of the values differs.
 *
 * ## Renderer coordination
 *
 * The listener applies internally so the underlying terrain library is
 * always in sync. The renderer needs its own notification so it can
 * refresh its heightmap caches. To avoid recomputing on every
 * PARAM_VALUE message, the listener exposes a consumable dirty flag
 * (`terrain_params_listener_consume_dirty()`): true exactly once after
 * an apply, then resets. The integration site drains it after each
 * dispatch and calls the renderer's apply function with
 * `terrain_params_listener_current()`.
 */

#ifndef HAWKEYE_MAVLINK_TERRAIN_PARAMS_LISTENER_H_
#define HAWKEYE_MAVLINK_TERRAIN_PARAMS_LISTENER_H_

#include <stdbool.h>

#include "terrain/terrain_params.h"

/*
 * Opaque forward declaration of MAVLink's PARAM_VALUE message struct.
 * The full definition lives in <mavlink_msg_param_value.h> (pulled in
 * by <mavlink.h>); callers that already include MAVLink can pass a
 * pointer here without us re-including the full MAVLink tree.
 *
 * C11 permits the same typedef to be declared more than once as long
 * as both refer to the same type (§6.7p3) — so this co-exists with
 * MAVLink's own typedef in any translation unit that includes both.
 */
typedef struct __mavlink_param_value_t mavlink_param_value_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Feed one decoded PARAM_VALUE message to the listener.
 *
 * Safe to call with a NULL pointer or a non-SIH_TERR_* parameter: in
 * either case the call is a no-op for the accumulator. The listener
 * decides internally whether the message justifies an apply call.
 */
void terrain_params_on_param_value(const mavlink_param_value_t *msg);

/**
 * Read the current accumulated SIH_TERR_* state. Pointer is valid for
 * the lifetime of the process and reflects whatever has been applied
 * most recently (or defaults if nothing yet).
 */
const HawkeyeTerrainParams *terrain_params_listener_current(void);

/**
 * Consume the "params were just applied" edge. Returns true exactly
 * once per apply event, then resets to false. Use this from the
 * integration site to fan an apply out to the renderer without
 * re-computing on every PARAM_VALUE message.
 */
bool terrain_params_listener_consume_dirty(void);

/**
 * Reset the listener's accumulator and seen-mask to startup state.
 * Primarily for tests; production code wouldn't normally need this.
 */
void terrain_params_listener_reset(void);

/**
 * Number of SIH_TERR_* params the listener tracks (5: EN/AMP/FREQ/SEED/PLANE).
 * Stable across builds; the request-retry layer uses it to size its
 * per-param bookkeeping without re-including the enum.
 */
int terrain_params_listener_param_count(void);

/**
 * Bitmask of param indices NOT yet received (bit i = 1 if index i is
 * still missing). Returns 0 once every SIH_TERR_* has been seen at
 * least once. The MAVLink receive layer drives a request-with-retry
 * loop off this value so individual UDP drops of a PARAM_VALUE on
 * connect can be recovered.
 */
unsigned int terrain_params_listener_missing_mask(void);

/**
 * Map a listener-local param index (0..param_count-1) back to its
 * upstream SIH_TERR_* name (used in PARAM_REQUEST_READ). Returns NULL
 * for out-of-range indices. Pointer is into static const storage —
 * safe to keep across calls.
 */
const char *terrain_params_listener_name_for_index(int idx);

/**
 * Seconds since this index's most-recent PARAM_VALUE was received
 * (monotonic clock). Returns -1.0 if the param has never been received
 * (still missing from the initial snapshot). Stamped on every accepted
 * PARAM_VALUE, even when the value didn't change — receipt itself is
 * what proves the path is alive. Used by the receiver's stale-detection
 * log so a silently broken re-sync loop becomes visible.
 */
double terrain_params_listener_seconds_since_received(int idx);

#ifdef __cplusplus
}
#endif

#endif /* HAWKEYE_MAVLINK_TERRAIN_PARAMS_LISTENER_H_ */
