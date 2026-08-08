/**
 * @file terrain_params_extract.c
 *
 * Self-contained ULog walker: pulls SIH_TERR_* parameters out of the
 * `P` message stream (both definitions section + any runtime changes
 * in the data section) and fills a HawkeyeTerrainParams.
 *
 * ## Why standalone (not built on ulog_parser_t)
 *
 * The existing ulog_parser_t targets timeline replay — it indexes
 * `D` (data) messages by timestamp and exposes per-topic field
 * accessors. It does NOT capture `P` messages, and extending it to
 * do so would mean editing files owned by another work scope. A
 * single-pass file walker for `P` messages only is ~80 lines, runs
 * in well under a second on multi-GB logs (everything non-P is
 * fseek-skipped, not read), and keeps surface inside the
 * files this scope owns.
 *
 * ## Value decoding (the defs-vs-data quirk)
 *
 * `P` messages appear in both sections of the file and the two layouts
 * are subtly different:
 *
 *   - Defs section (initial_parameters): the 4 value bytes are encoded
 *     in the declared type's representation. `int32_t SIH_TERR_OCT`
 *     value 6 lands as `0x06 0x00 0x00 0x00`.
 *
 *   - Data section (changed_parameters from PARAM_SET): PX4's PARAM_SET
 *     wire-encodes ALL values as float regardless of declared type, and
 *     the logger writes those float bits verbatim. So
 *     `int32_t SIH_TERR_SEED` set to 3 via PARAM_SET lands as
 *     `0x00 0x00 0x40 0x40` — the IEEE-754 bit pattern of 3.0f, not
 *     the int32 bit pattern of 3.
 *
 * The walker tracks which section it is in (flips to data on the first
 * data-only message: D/L/S/O/R). For each `P` message it inspects the
 * type prefix in the key (`"float"` vs `"int32_t"`) and chooses the
 * right decode path. This was confirmed against fixture
 * (tests/terrain/fixtures/sih_terrain_on.ulg).
 */

#include "terrain_params_extract.h"
#include "terrain/terrain_params.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ULOG_HEADER_BYTES        16
#define ULOG_MSG_HEADER_BYTES    3
#define ULOG_PARAM_KEY_MAX       255
#define ULOG_PARAM_PAYLOAD_MAX   512  /* key_len(1) + key(<=255) + value(<=8); 512 is comfortable */

static const uint8_t ULOG_MAGIC[7] = { 'U', 'L', 'o', 'g', 0x01, 0x12, 0x35 };

/*
 * Decode one parsed key/value pair into the output struct. `type_str`
 * is the type prefix ("float" or "int32_t"), `name` is the bare
 * parameter name, `value_bytes` points at the 4 value bytes. The
 * caller passes `in_data_section` so we can apply the float-bits-for-
 * int quirk that only affects the data section's `P` messages.
 */
static void apply_param(HawkeyeTerrainParams *out,
                        const char *type_str, const char *name,
                        const uint8_t *value_bytes, bool in_data_section,
                        bool *found_en)
{
    bool type_is_int = (strcmp(type_str, "int32_t") == 0);

    float fval = 0.0f;
    int   ival = 0;

    if (type_is_int) {
        /*
         * int32 params appear in two on-disk encodings depending on the
         * PX4 logger / param-set path that produced the log:
         *
         *   - native int32: the 4 bytes are the int value directly
         *     (e.g. SIH_TERR_EN=2 -> 0x02 0x00 0x00 0x00). The defs
         *     section always uses this, and current PX4 writes it for
         *     data-section changes too.
         *   - float-encoded: the 4 bytes are the IEEE-754 bits of the
         *     value cast to float (e.g. 2 -> 0x00 0x00 0x00 0x40 = 2.0f).
         *     Some older logs store data-section changes this way.
         *
         * Disambiguate by shape, not by section: a float-encoded small
         * integer has a LARGE native-int32 magnitude (1.0f = 0x3f800000
         * = 1065353216) AND decodes to a clean, in-range finite float; a
         * native small int has a tiny denormal float. Realistic
         * SIH_TERR_* values (mode 0..2, small seeds) never collide.
         */
        int32_t as_int;
        float   as_flt;
        memcpy(&as_int, value_bytes, sizeof(as_int));
        memcpy(&as_flt, value_bytes, sizeof(as_flt));

        bool float_encoded =
            (as_int > 1000000 || as_int < -1000000) &&
            isfinite(as_flt) && as_flt == floorf(as_flt) &&
            as_flt >= -2.0e9f && as_flt <= 2.0e9f;

        if (float_encoded) {
            ival = (int)as_flt;
            fval = as_flt;
        } else {
            ival = (int)as_int;
            fval = (float)as_int;
        }
    } else {
        /* Float-typed param (any section): IEEE-754 float bits. */
        memcpy(&fval, value_bytes, sizeof(fval));
        ival = (int)fval;
    }

    /*
     * SIH_TERR_* set: EN / AMP / FREQ / SEED / PLANE. AMP and
     * FREQ were restored upstream on top of
     * wall lattice. OCT / HURST / EROSION are still gone — those got
     * permanently hard-coded as deep noise internals. Pre-3.8 ULogs that
     * lack AMP/FREQ keys leave the struct's default values in place.
     * Pre-3.7 ULogs that still carry OCT/HURST/EROSION fall through and
     * are silently ignored.
     */
    if (strcmp(name, "SIH_TERR_EN") == 0) {
        /* tri-state: ival is the mode enum (0/1/2). Clamp
         * out-of-range values to OFF for safety against corrupted logs. */
        if (ival < HAWKEYE_TERRAIN_MODE_OFF || ival > HAWKEYE_TERRAIN_MODE_WALLS) {
            ival = HAWKEYE_TERRAIN_MODE_OFF;
        }
        out->mode = ival;
        *found_en = true;
    } else if (strcmp(name, "SIH_TERR_AMP") == 0) {
        out->amp = fval;
    } else if (strcmp(name, "SIH_TERR_FREQ") == 0) {
        out->freq = fval;
    } else if (strcmp(name, "SIH_TERR_SEED") == 0) {
        out->seed = ival;
    } else if (strcmp(name, "SIH_TERR_PLANE") == 0) {
        out->plane_deg = fval;
    }
    /*
     * SIH_LOC_LAT0 / SIH_LOC_LON0 are intentionally not stored in this
     * MVP: the viewer has no configurable world origin yet, so the SIH
     * spawn point IS the NED origin and `home_n_m = home_e_m = 0` is
     * the correct value (matches the upstream contract that
     * `terrain(home_n, home_e) == 0`). When a project-wide origin is
     * introduced, a flat-earth conversion of (LAT0 - origin_lat,
     * LON0 - origin_lon) goes here.
     */
}

/*
 * Process one 'P' (parameter) message payload. Format:
 *   byte 0:                key_len
 *   bytes 1..1+key_len:    "<type> <name>" (ASCII, no NUL)
 *   bytes 1+key_len..end:  value (4 bytes for float/int32)
 */
static void process_param_message(HawkeyeTerrainParams *out,
                                  const uint8_t *payload, int payload_len,
                                  bool in_data_section, bool *found_en)
{
    if (payload_len < 1) return;
    int key_len = payload[0];
    if (key_len < 3 || 1 + key_len + 4 > payload_len) return;

    /* Copy key into a NUL-terminated buffer so we can use strchr/strcmp */
    char key[ULOG_PARAM_KEY_MAX + 1];
    int klim = key_len < ULOG_PARAM_KEY_MAX ? key_len : ULOG_PARAM_KEY_MAX;
    memcpy(key, payload + 1, klim);
    key[klim] = '\0';

    /* Split "type name" on first space */
    char *space = strchr(key, ' ');
    if (!space) return;
    *space = '\0';
    const char *type_str = key;
    const char *name     = space + 1;

    const uint8_t *value_bytes = payload + 1 + key_len;
    apply_param(out, type_str, name, value_bytes, in_data_section, found_en);
}

/*
 * Helper: bump `in_data_section` true on the first data-only message
 * type. Same rule shared by the file and buffer walkers.
 */
static inline bool is_data_only_type(uint8_t msg_type)
{
    return (msg_type == 'D' || msg_type == 'L' || msg_type == 'S' ||
            msg_type == 'O' || msg_type == 'R');
}

bool ulog_extract_terrain_params(const char *ulog_path,
                                 HawkeyeTerrainParams *out)
{
    if (!out) return false;
    *out = hawkeye_terrain_params_default();

    if (!ulog_path) return false;

    FILE *fp = fopen(ulog_path, "rb");
    if (!fp) return false;

    uint8_t header[ULOG_HEADER_BYTES];
    if (fread(header, 1, ULOG_HEADER_BYTES, fp) != ULOG_HEADER_BYTES) {
        fclose(fp);
        return false;
    }
    if (memcmp(header, ULOG_MAGIC, sizeof(ULOG_MAGIC)) != 0) {
        fclose(fp);
        return false;
    }

    bool found_en        = false;
    bool in_data_section = false;
    uint8_t payload[ULOG_PARAM_PAYLOAD_MAX];

    for (;;) {
        uint8_t msg_hdr[ULOG_MSG_HEADER_BYTES];
        if (fread(msg_hdr, 1, ULOG_MSG_HEADER_BYTES, fp) != ULOG_MSG_HEADER_BYTES) {
            break;
        }

        uint16_t msg_size = (uint16_t)msg_hdr[0] | ((uint16_t)msg_hdr[1] << 8);
        uint8_t  msg_type = msg_hdr[2];

        if (is_data_only_type(msg_type)) {
            in_data_section = true;
        }

        if (msg_type == 'P' && msg_size <= ULOG_PARAM_PAYLOAD_MAX) {
            if (fread(payload, 1, msg_size, fp) != msg_size) break;
            process_param_message(out, payload, msg_size,
                                  in_data_section, &found_en);
        } else {
            /* Skip the message entirely without buffering its payload */
            if (fseek(fp, msg_size, SEEK_CUR) != 0) break;
        }
    }

    fclose(fp);
    return found_en;
}

bool ulog_extract_terrain_params_from_buffer(const uint8_t *buf, size_t len,
                                             HawkeyeTerrainParams *out)
{
    if (!out) return false;
    *out = hawkeye_terrain_params_default();

    if (!buf || len < ULOG_HEADER_BYTES) return false;
    if (memcmp(buf, ULOG_MAGIC, sizeof(ULOG_MAGIC)) != 0) return false;

    bool found_en        = false;
    bool in_data_section = false;

    /*
     * The WASM build hands the entire .ulg as a buffer (browser has no
     * filesystem); we walk it in-place with explicit offset bookkeeping
     * instead of fread/fseek. Identical message-decode policy as the
     * file walker — section flip, payload-size cap, `P`-message
     * processing — so a fixture file and its in-memory copy produce
     * byte-identical HawkeyeTerrainParams output.
     */
    size_t off = ULOG_HEADER_BYTES;
    while (off + ULOG_MSG_HEADER_BYTES <= len) {
        uint16_t msg_size = (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
        uint8_t  msg_type = buf[off + 2];
        off += ULOG_MSG_HEADER_BYTES;

        if (off + msg_size > len) break;  /* truncated payload */

        if (is_data_only_type(msg_type)) {
            in_data_section = true;
        }

        if (msg_type == 'P' && msg_size <= ULOG_PARAM_PAYLOAD_MAX) {
            process_param_message(out, buf + off, msg_size,
                                  in_data_section, &found_en);
        }
        /* Skip past the message body regardless of type. */
        off += msg_size;
    }

    return found_en;
}
