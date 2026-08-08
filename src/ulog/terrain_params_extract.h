/**
 * @file terrain_params_extract.h
 *
 * Pull the SIH_TERR_* parameter group out of a ULog file's parameter
 * snapshot, returning a populated HawkeyeTerrainParams ready to feed
 * into hawkeye_terrain_apply_params().
 *
 * Both the boot-time `initial_parameters` (defs section 'P' messages)
 * and any runtime PARAM_SET-driven `changed_parameters` (data section
 * 'P' messages) are honored — the last write wins, so the returned
 * struct reflects the EFFECTIVE state at end-of-log. Callers replaying
 * the timeline can re-extract or use the MAVLink listener for live
 * updates.
 *
 * The function does not depend on the existing ulog_parser_t — it
 * walks the file independently using fseek-skip so the cost is small
 * even on multi-GB logs.
 */

#ifndef HAWKEYE_ULOG_TERRAIN_PARAMS_EXTRACT_H_
#define HAWKEYE_ULOG_TERRAIN_PARAMS_EXTRACT_H_

#include <stdbool.h>

#include "terrain/terrain_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open `ulog_path`, scan all 'P' parameter messages (both defs section
 * and data section), and fill `out` with the effective SIH_TERR_* values.
 *
 * @param ulog_path  Path to a .ulg file.
 * @param out        Destination struct. Initialised to defaults before
 *                   scanning, then overwritten with whatever the log
 *                   contains. Must be non-NULL.
 * @return true if SIH_TERR_EN was found in the log (i.e. the log was
 *         produced by a SIH-aware firmware); false otherwise (missing
 *         param, missing file, malformed header). On `false` the `out`
 *         struct is still initialised to safe defaults.
 */
bool ulog_extract_terrain_params(const char *ulog_path,
                                 HawkeyeTerrainParams *out);

#include <stddef.h>
#include <stdint.h>

/**
 * Buffer-source variant for the WASM build (browser hands the .ulg in as
 * a `Uint8Array`; there is no FILE*). Same semantics as
 * ulog_extract_terrain_params() — defs + data section walk, last write
 * wins, identical defs-vs-data int-decoding policy.
 *
 * @param buf  Pointer to the start of the ULog bytes.
 * @param len  Number of bytes available at `buf`.
 * @param out  Destination struct (initialised to defaults before scan).
 * @return same as ulog_extract_terrain_params: true iff SIH_TERR_EN was
 *         observed in the buffer; false on missing buffer, short read,
 *         or malformed header.
 */
bool ulog_extract_terrain_params_from_buffer(const uint8_t *buf, size_t len,
                                             HawkeyeTerrainParams *out);

#ifdef __cplusplus
}
#endif

#endif /* HAWKEYE_ULOG_TERRAIN_PARAMS_EXTRACT_H_ */
