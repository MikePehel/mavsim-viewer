#include "tactical_hud.h"
#include "theme.h"
#include "ulog_replay.h"
#include "raylib.h"
#include "raymath.h"

#ifdef _WIN32
#undef DrawText
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

// All wireframe coordinates are from a 1920x1080 viewBox.
// We map proportionally to actual screen dimensions.
#define WF_W 1920.0f
#define WF_H 1080.0f

static float wx(float v, int sw) { return v * sw / WF_W; }
static float wy(float v, int sh) { return v * sh / WF_H; }
static float ws(float v, int sw, int sh) {
    float scale = (sw / WF_W < sh / WF_H) ? sw / WF_W : sh / WF_H;
    return v * scale;
}

// ── Tag shape ──────────────────────────────────────────────────────────────
// 5-point shape with rounded corners, matching the wireframe SVG path exactly.
// point_right=true: GS (point toward drone, rounded left corners)
// point_right=false: ALT (point toward drone, rounded right corners)
// The wireframe path uses 6.66px corner radius at 1920x1080.
static void draw_tag(float x, float y, float body_w, float h, float point_w,
                     float cr, bool point_right, float thick, Color color) {
    float mid_y = y + h / 2.0f;
    // Corner radius in pixels (cr is already screen-mapped)
    int segs = 6; // segments per rounded corner

    // Build the 5-point outline as line segments with rounded corners
    // For point_right (GS):
    //   TL(rounded) -> TR -> TIP -> BR -> BL(rounded)
    // For point_left (ALT):
    //   TIP -> TR(rounded) -> BR(rounded) -> BL -> back to TIP

    if (point_right) {
        // Top-left rounded corner
        float tlx = x + cr, tly = y;
        for (int i = 0; i <= segs; i++) {
            float a1 = PI + (PI / 2.0f) * (float)i / segs;
            float a2 = PI + (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = PI + PI / 2.0f;
            DrawLineEx(
                (Vector2){x + cr + cosf(a1) * cr, y + cr + sinf(a1) * cr},
                (Vector2){x + cr + cosf(a2) * cr, y + cr + sinf(a2) * cr},
                thick, color);
        }
        // Top edge: from TL corner to TR
        DrawLineEx((Vector2){x + cr, y}, (Vector2){x + body_w, y}, thick, color);
        // TR to tip
        DrawLineEx((Vector2){x + body_w, y}, (Vector2){x + body_w + point_w, mid_y}, thick, color);
        // Tip to BR
        DrawLineEx((Vector2){x + body_w + point_w, mid_y}, (Vector2){x + body_w, y + h}, thick, color);
        // Bottom edge: BR to BL corner
        DrawLineEx((Vector2){x + body_w, y + h}, (Vector2){x + cr, y + h}, thick, color);
        // Bottom-left rounded corner
        for (int i = 0; i <= segs; i++) {
            float a1 = PI / 2.0f + (PI / 2.0f) * (float)i / segs;
            float a2 = PI / 2.0f + (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = PI;
            DrawLineEx(
                (Vector2){x + cr + cosf(a1) * cr, y + h - cr + sinf(a1) * cr},
                (Vector2){x + cr + cosf(a2) * cr, y + h - cr + sinf(a2) * cr},
                thick, color);
        }
        // Left edge: BL corner to TL corner
        DrawLineEx((Vector2){x, y + h - cr}, (Vector2){x, y + cr}, thick, color);
    } else {
        // Top-right rounded corner
        float rx = x + point_w + body_w;
        // Tip to TL
        DrawLineEx((Vector2){x, mid_y}, (Vector2){x + point_w, y}, thick, color);
        // Top edge: TL to TR corner
        DrawLineEx((Vector2){x + point_w, y}, (Vector2){rx - cr, y}, thick, color);
        // Top-right rounded corner
        for (int i = 0; i <= segs; i++) {
            float a1 = -PI / 2.0f + (PI / 2.0f) * (float)i / segs;
            float a2 = -PI / 2.0f + (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = 0;
            DrawLineEx(
                (Vector2){rx - cr + cosf(a1) * cr, y + cr + sinf(a1) * cr},
                (Vector2){rx - cr + cosf(a2) * cr, y + cr + sinf(a2) * cr},
                thick, color);
        }
        // Right edge
        DrawLineEx((Vector2){rx, y + cr}, (Vector2){rx, y + h - cr}, thick, color);
        // Bottom-right rounded corner
        for (int i = 0; i <= segs; i++) {
            float a1 = (PI / 2.0f) * (float)i / segs;
            float a2 = (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = PI / 2.0f;
            DrawLineEx(
                (Vector2){rx - cr + cosf(a1) * cr, y + h - cr + sinf(a1) * cr},
                (Vector2){rx - cr + cosf(a2) * cr, y + h - cr + sinf(a2) * cr},
                thick, color);
        }
        // Bottom edge: BR corner to BL
        DrawLineEx((Vector2){rx - cr, y + h}, (Vector2){x + point_w, y + h}, thick, color);
        // BL to tip
        DrawLineEx((Vector2){x + point_w, y + h}, (Vector2){x, mid_y}, thick, color);
    }
}

// ── Timer + Status (top-left) ──────────────────────────────────────────────
// Wireframe: timer at ~(27, 27), text "38:18" in st92 (#444)
static void tac_draw_timer_status(const hud_t *h, const data_source_t *src,
                                   int sw, int sh, const theme_t *theme) {
    float mx = wx(27, sw);
    float my = wy(27, sh);
    // Timer text — wireframe has it ~21px at 1080p
    float fs_timer = wy(21, sh);
    float fs_label = wy(12, sh);
    float fs_status = wy(13, sh);

    char b[16];
    int total_secs = (int)h->sim_time_s;
    int mins = total_secs / 60;
    int secs = total_secs % 60;
    if (mins >= 60)
        snprintf(b, sizeof(b), "%d:%02d:%02d", mins / 60, mins % 60, secs);
    else
        snprintf(b, sizeof(b), "%02d:%02d", mins, secs);
    DrawTextEx(h->font_value, b, (Vector2){mx, my}, fs_timer, 0.5f, theme->hud_value);
    DrawTextEx(h->font_label, "SIM",
               (Vector2){mx, my + fs_timer + wy(2, sh)}, fs_label, 0.5f, theme->hud_dim);

    // Connection
    bool connected = src->connected;
    float dot_y = my + fs_timer + fs_label + wy(12, sh);
    Color dot_c = connected ? theme->hud_connected : (Color){200, 60, 60, 255};
    DrawCircle((int)(mx + wy(4, sh)), (int)(dot_y + wy(4, sh)), wy(4, sh), dot_c);

    char status_buf[48];
    if (src->playback.duration_s > 0.0f) {
        if (!connected) snprintf(status_buf, sizeof(status_buf), "Replay ended");
        else if (src->playback.paused) snprintf(status_buf, sizeof(status_buf), "Replay paused");
        else snprintf(status_buf, sizeof(status_buf), "Replaying log...");
    } else if (connected) {
        snprintf(status_buf, sizeof(status_buf), "MAVLink connected");
    } else {
        snprintf(status_buf, sizeof(status_buf), "Waiting...");
    }
    DrawTextEx(h->font_label, status_buf,
               (Vector2){mx + wy(14, sh), dot_y}, fs_status, 0.5f,
               connected ? theme->hud_connected : theme->hud_dim);
}

// ── Ticker (top-center) ────────────────────────────────────────────────────
static void tac_draw_ticker(const hud_t *h, const playback_state_t *pb,
                             int sw, int sh, const theme_t *theme) {
    float fs = wy(16, sh);
    const char *mode_name = NULL;
    if (pb && pb->mode_changes && pb->mode_change_count > 0) {
        for (int i = pb->mode_change_count - 1; i >= 0; i--) {
            if (pb->mode_changes[i].time_s <= pb->position_s) {
                mode_name = ulog_nav_state_name(pb->mode_changes[i].nav_state);
                break;
            }
        }
    }
    if (mode_name) {
        char buf[64];
        snprintf(buf, sizeof(buf), "MODE: %s", mode_name);
        Vector2 tw = MeasureTextEx(h->font_label, buf, fs, 0.5f);
        DrawTextEx(h->font_label, buf,
                   (Vector2){sw / 2.0f - tw.x / 2, wy(27, sh)},
                   fs, 0.5f, theme->hud_warn);
    }
}

// ── Heading (top-right) — always HDG, never YAW ───────────────────────────
// Wireframe: HDG label st96, value st93 at top-right
static void tac_draw_heading(const hud_t *h, const vehicle_t *v,
                              int sw, int sh, const theme_t *theme) {
    float mx = wx(27, sw);
    float fs_label = wy(13, sh);
    float fs_value = wy(27, sh);

    // HDG label
    const char *lbl = "HDG";
    Vector2 lw = MeasureTextEx(h->font_label, lbl, fs_label, 0.5f);
    DrawTextEx(h->font_label, lbl,
               (Vector2){sw - mx - lw.x, wy(27, sh)},
               fs_label, 0.5f, theme->hud_accent_dim);

    // Heading value
    char b[8];
    snprintf(b, sizeof(b), "%03d", ((int)v->heading_deg % 360 + 360) % 360);
    Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
    DrawTextEx(h->font_value, b,
               (Vector2){sw - mx - vw.x, wy(27, sh) + fs_label + wy(2, sh)},
               fs_value, 0.5f, theme->hud_value);
}

// ── Speed Stack (GS tag, right-aligned at 25%) ────────────────────────────
// Wireframe tag: x=377-501, y=482-518 → body_w=102, height=36, point_w=22
// At 1920 wide, anchor at 25% = 480. Tag tip at 501 → tip is 21px past 480.
// Tag body from 377 to 479 = 102px. Point from 479 to 501 = 22px.
static void tac_draw_speed_stack(const hud_t *h, const vehicle_t *vehicles,
                                  int vehicle_count, int selected,
                                  int sw, int sh, const theme_t *theme) {
    float anchor = sw * 0.25f;
    float cy = sh * 0.5f;

    // Tag proportions from wireframe (at 1920x1080)
    float body_w = wx(102, sw);
    float tag_h = wy(36, sh);
    float point_w = wx(22, sw);
    float tag_total = body_w + point_w;

    float fs_label = wy(13, sh);
    float fs_value = wy(28, sh); // wireframe value text is ~28px at 1080
    float fs_unit = wy(11, sh);
    float fs_pinned = wy(19, sh);
    float line_h = wy(22, sh);

    Color border = theme->hud_border;
    Color value_c = theme->hud_value;
    Color label_c = theme->hud_accent_dim;
    Color dim_c = theme->hud_dim;

    const vehicle_t *v = &vehicles[selected];

    // Tag: body starts at anchor - tag_total, tip at anchor + point past anchor
    // Actually: the tip points right (toward drone center), so the tag goes:
    // body from (anchor - tag_total) to (anchor - point_w), then point to anchor
    float tag_x = anchor - tag_total;
    float tag_y = cy - tag_h / 2.0f;
    float cr = ws(6.66f, sw, sh); // wireframe corner radius
    draw_tag(tag_x, tag_y, body_w, tag_h, point_w, cr, true, 1.3f, border);

    // Value centered in body
    char val[16];
    snprintf(val, sizeof(val), "%.1f", v->ground_speed);
    Vector2 vw = MeasureTextEx(h->font_value, val, fs_value, 0.5f);
    DrawTextEx(h->font_value, val,
               (Vector2){tag_x + body_w / 2 - vw.x / 2, tag_y + tag_h / 2 - vw.y / 2},
               fs_value, 0.5f, value_c);

    // "GS" label above, right-aligned to body right edge
    Vector2 lw = MeasureTextEx(h->font_label, "GS", fs_label, 0.5f);
    DrawTextEx(h->font_label, "GS",
               (Vector2){tag_x + body_w - lw.x, tag_y - fs_label - wy(4, sh)},
               fs_label, 0.5f, label_c);

    // "m/s" below, right-aligned to body right edge
    Vector2 uw = MeasureTextEx(h->font_label, "m/s", fs_unit, 0.5f);
    DrawTextEx(h->font_label, "m/s",
               (Vector2){tag_x + body_w - uw.x, tag_y + tag_h + wy(4, sh)},
               fs_unit, 0.5f, dim_c);

    // Pinned values stacked upward
    float pinned_y = tag_y - fs_label - wy(4, sh) - line_h;
    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;
        char pb[16];
        snprintf(pb, sizeof(pb), "%.1f", vehicles[pidx].ground_speed);
        Vector2 pw = MeasureTextEx(h->font_value, pb, fs_pinned, 0.5f);
        DrawTextEx(h->font_value, pb,
                   (Vector2){tag_x + body_w - pw.x, pinned_y - p * line_h},
                   fs_pinned, 0.5f, vehicles[pidx].color);
    }
}

// ── Altitude Stack (ALT tag, left-aligned at 75%) ─────────────────────────
// Wireframe ALT tag: x=1419-1544, y=482-518 → mirror of GS
static void tac_draw_alt_stack(const hud_t *h, const vehicle_t *vehicles,
                                int vehicle_count, int selected,
                                int sw, int sh, const theme_t *theme) {
    float anchor = sw * 0.75f;
    float cy = sh * 0.5f;

    float body_w = wx(102, sw);
    float tag_h = wy(36, sh);
    float point_w = wx(22, sw);

    float fs_label = wy(13, sh);
    float fs_value = wy(28, sh);
    float fs_vs = wy(13, sh);
    float fs_pinned = wy(19, sh);
    float fs_pinned_vs = wy(12, sh);
    float line_h = wy(22, sh);

    Color border = theme->hud_border;
    Color value_c = theme->hud_value;
    Color label_c = theme->hud_accent_dim;
    Color dim_c = theme->hud_dim;
    Color climb_c = theme->hud_climb;
    Color warn_c = theme->hud_warn;

    const vehicle_t *v = &vehicles[selected];

    // Tag: point faces left (toward drone), body extends right from anchor
    float tag_x = anchor;
    float tag_y = cy - tag_h / 2.0f;
    float cr = ws(6.66f, sw, sh);
    draw_tag(tag_x, tag_y, body_w, tag_h, point_w, cr, false, 1.3f, border);

    // Value centered in body (body starts at tag_x + point_w)
    char val[16];
    snprintf(val, sizeof(val), "%.1f", v->altitude_rel);
    Vector2 vw = MeasureTextEx(h->font_value, val, fs_value, 0.5f);
    float body_x = tag_x + point_w;
    DrawTextEx(h->font_value, val,
               (Vector2){body_x + body_w / 2 - vw.x / 2, tag_y + tag_h / 2 - vw.y / 2},
               fs_value, 0.5f, value_c);

    // "ALT" label above, left-aligned to body left edge
    DrawTextEx(h->font_label, "ALT",
               (Vector2){body_x, tag_y - fs_label - wy(4, sh)},
               fs_label, 0.5f, label_c);

    // VS below tag
    Color vs_c = (v->vertical_speed > 0.1f) ? climb_c :
                 (v->vertical_speed < -0.1f) ? warn_c : value_c;
    const char *arrow = (v->vertical_speed > 0.1f) ? "^" :
                        (v->vertical_speed < -0.1f) ? "v" : "";
    char vs_buf[16];
    snprintf(vs_buf, sizeof(vs_buf), "%s %.1f m/s", arrow, fabsf(v->vertical_speed));
    DrawTextEx(h->font_value, vs_buf,
               (Vector2){body_x, tag_y + tag_h + wy(4, sh)},
               fs_vs, 0.5f, vs_c);

    // Pinned values stacked upward
    float pinned_y = tag_y - fs_label - wy(4, sh) - line_h;
    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;
        char pb[16];
        snprintf(pb, sizeof(pb), "%.1f", vehicles[pidx].altitude_rel);
        DrawTextEx(h->font_value, pb,
                   (Vector2){body_x, pinned_y - p * line_h},
                   fs_pinned, 0.5f, vehicles[pidx].color);

        // Inline VS arrow
        Vector2 pw = MeasureTextEx(h->font_value, pb, fs_pinned, 0.5f);
        const char *va = (vehicles[pidx].vertical_speed > 0.1f) ? "^" :
                         (vehicles[pidx].vertical_speed < -0.1f) ? "v" : "-";
        DrawTextEx(h->font_value, va,
                   (Vector2){body_x + pw.x + wx(4, sw), pinned_y - p * line_h},
                   fs_pinned_vs, 0.5f, vehicles[pidx].color);
    }
}

// ── Radar Panel (bottom-left) ──────────────────────────────────────────────
// Reuses the ortho TOP view with all 2D overlays (grid, trails, drone dots),
// then draws compass elements and radar blips on top.
// Wireframe: panel at x=95, y=692, 293x293.
static void tac_draw_radar(const hud_t *h, const vehicle_t *vehicles,
                            int vehicle_count, int selected,
                            int sw, int sh, const theme_t *theme,
                            const ortho_panel_t *ortho,
                            int trail_mode, int corr_mode) {
    float ps = wx(293, sw); // square panel
    float px = wx(95, sw);
    float py = (float)sh - wy(95, sh) - ps; // 95px from bottom edge

    // Compass center: shifted down by 1/6 panel for forward visibility
    float ccx = px + ps / 2.0f;
    float compass_offset = ps / 4.0f; // 1.5 grid squares down
    float ccy = py + ps / 2.0f + compass_offset;
    float cr = ps * 0.46f; // almost touches panel edges

    Color accent = theme->hud_accent;
    Color dim_c = theme->hud_dim;
    Color warn = theme->hud_warn;

    // Rebuilt from scratch: hud_bg ground, grid, trails, ribbons, curtains, dots, compass.
    // No ortho texture blit. No tint wash.
    DrawRectangle((int)px, (int)py, (int)ps, (int)ps, theme->hud_bg);

    if (ortho) {
        BeginScissorMode((int)px, (int)py, (int)ps, (int)ps);
        Vector3 center = vehicles[selected].position;
        float span = ortho->ortho_span;
        float grid_scale = ps / span;

        // Grid spacing
        float spacing = 10.0f;
        if (span > 200.0f) spacing = 50.0f;
        else if (span > 80.0f) spacing = 20.0f;
        else if (span < 20.0f) spacing = 2.0f;

        Color grid_minor = theme->ortho_grid_minor;
        grid_minor.a = (unsigned char)(grid_minor.a * 0.5f);
        Color grid_major = theme->ortho_grid_major;
        grid_major.a = (unsigned char)(grid_major.a * 0.5f);
        float ext = span * 0.8f; // cover full panel including compass offset
        float sx_start = floorf((center.x - ext) / spacing) * spacing;
        float sz_start = floorf((center.z - ext) / spacing) * spacing;

        for (float x = sx_start; x <= center.x + ext; x += spacing) {
            bool major = fabsf(fmodf(x, spacing * 5)) < 0.1f;
            Color gc = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            float sx1 = ccx + (x - center.x) * grid_scale;
            float sy1 = ccy + (-ext) * grid_scale;
            float sy2 = ccy + (ext) * grid_scale;
            DrawLineEx((Vector2){sx1, sy1}, (Vector2){sx1, sy2}, lw, gc);
        }
        for (float z = sz_start; z <= center.z + ext; z += spacing) {
            bool major = fabsf(fmodf(z, spacing * 5)) < 0.1f;
            Color gc = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            float sy1 = ccy + (z - center.z) * grid_scale;
            float sx1 = ccx + (-ext) * grid_scale;
            float sx2 = ccx + (ext) * grid_scale;
            DrawLineEx((Vector2){sx1, sy1}, (Vector2){sx2, sy1}, lw, gc);
        }
        // Trails — same directional coloring as ortho panels
        for (int vi = 0; vi < vehicle_count; vi++) {
            const vehicle_t *v = &vehicles[vi];
            if (trail_mode <= 0 || v->trail_count < 2) continue;
            if (!v->active && vehicle_count > 1) continue;
            int start = (v->trail_count < v->trail_capacity) ? 0 : v->trail_head;

            Color col_fwd  = theme->trail_forward;
            Color col_back = theme->trail_backward;
            Color col_up   = theme->trail_climb;
            Color col_down = theme->trail_descend;
            Color col_rp   = theme->trail_roll_pos;
            Color col_rn   = theme->trail_roll_neg;

            for (int ti = 1; ti < v->trail_count; ti++) {
                int i0 = (start + ti - 1) % v->trail_capacity;
                int i1 = (start + ti) % v->trail_capacity;
                float t = (float)ti / (float)v->trail_count;

                float cr_f, cg_f, cb_f;
                unsigned char ca;

                if (trail_mode == 1) {
                    // Directional coloring
                    cr_f = (float)col_fwd.r; cg_f = (float)col_fwd.g; cb_f = (float)col_fwd.b;
                    float pitch = v->trail_pitch[i1];
                    float vert  = v->trail_vert[i1];
                    float roll  = v->trail_roll[i1];

                    float bt = pitch / 15.0f;
                    if (bt < 0) bt = 0; if (bt > 1) bt = 1;
                    cr_f += (col_back.r - cr_f) * bt;
                    cg_f += (col_back.g - cg_f) * bt;
                    cb_f += (col_back.b - cb_f) * bt;

                    float vt = vert / 5.0f;
                    if (vt > 1) vt = 1; if (vt < -1) vt = -1;
                    if (vt > 0) { cr_f += (col_up.r - cr_f)*vt; cg_f += (col_up.g - cg_f)*vt; cb_f += (col_up.b - cb_f)*vt; }
                    else if (vt < 0) { float d=-vt; cr_f += (col_down.r - cr_f)*d; cg_f += (col_down.g - cg_f)*d; cb_f += (col_down.b - cb_f)*d; }

                    float rt = roll / 15.0f;
                    if (rt > 1) rt = 1; if (rt < -1) rt = -1;
                    if (rt > 0) { cr_f += (col_rp.r - cr_f)*rt*0.7f; cg_f += (col_rp.g - cg_f)*rt*0.7f; cb_f += (col_rp.b - cb_f)*rt*0.7f; }
                    else if (rt < 0) { float d=-rt; cr_f += (col_rn.r - cr_f)*d*0.7f; cg_f += (col_rn.g - cg_f)*d*0.7f; cb_f += (col_rn.b - cb_f)*d*0.7f; }

                    ca = (unsigned char)(t * col_fwd.a);
                } else if (trail_mode == 2) {
                    // Speed ribbon — thermal gradient
                    float max_spd = v->trail_speed_max > 1.0f ? v->trail_speed_max : 1.0f;
                    float spd = v->trail_speed[i1];
                    float heat = spd / max_spd;
                    if (heat > 1.0f) heat = 1.0f;
                    if (heat < 0.0f) heat = 0.0f;
                    Color hc = theme_heat_color(theme, heat, (unsigned char)(t * 200));
                    cr_f = hc.r; cg_f = hc.g; cb_f = hc.b;
                    ca = hc.a;
                } else {
                    // Drone color trail (mode 3)
                    cr_f = v->color.r; cg_f = v->color.g; cb_f = v->color.b;
                    ca = (unsigned char)(t * 200);
                }

                float sx0 = ccx + (v->trail[i0].x - center.x) * grid_scale;
                float sz0 = ccy + (v->trail[i0].z - center.z) * grid_scale;
                float sx1 = ccx + (v->trail[i1].x - center.x) * grid_scale;
                float sz1 = ccy + (v->trail[i1].z - center.z) * grid_scale;
                float line_w = (trail_mode == 2) ? 2.0f : 1.0f;
                DrawLineEx((Vector2){sx0, sz0}, (Vector2){sx1, sz1}, line_w,
                           (Color){(unsigned char)cr_f, (unsigned char)cg_f, (unsigned char)cb_f, ca});
            }
        }

        // Compass rings centered at (ccx, ccy)
    DrawCircleLines((int)ccx, (int)ccy, cr, (Color){accent.r, accent.g, accent.b, 100});
    DrawCircleLines((int)ccx, (int)ccy, cr * 0.66f, (Color){accent.r, accent.g, accent.b, 60});
    DrawCircleLines((int)ccx, (int)ccy, cr * 0.33f, (Color){accent.r, accent.g, accent.b, 40});

    // Heading-up bearing lines
    float heading_rad = vehicles[selected].heading_deg * DEG2RAD;
    for (int deg = 0; deg < 360; deg += 30) {
        float angle = (deg * DEG2RAD) - heading_rad - PI / 2.0f;
        float inner = cr * 0.88f;
        float outer = cr;
        Color tc = (deg % 90 == 0) ? (Color){accent.r, accent.g, accent.b, 120}
                                   : (Color){accent.r, accent.g, accent.b, 60};
        DrawLineEx(
            (Vector2){ccx + cosf(angle) * inner, ccy + sinf(angle) * inner},
            (Vector2){ccx + cosf(angle) * outer, ccy + sinf(angle) * outer},
            1.0f, tc);
    }

    // Cardinal labels
    const char *labels[] = {"N", "E", "S", "W"};
    int label_degs[] = {0, 90, 180, 270};
    float fs_card = wy(13, sh);
    for (int i = 0; i < 4; i++) {
        float angle = (label_degs[i] * DEG2RAD) - heading_rad - PI / 2.0f;
        float lr = cr + wy(10, sh);
        float lx = ccx + cosf(angle) * lr;
        float ly = ccy + sinf(angle) * lr;
        Vector2 tw = MeasureTextEx(h->font_value, labels[i], fs_card, 0.5f);
        Color lc = (i == 0) ? warn : dim_c;
        DrawTextEx(h->font_value, labels[i],
                   (Vector2){lx - tw.x / 2, ly - tw.y / 2}, fs_card, 0.5f, lc);
    }

        // Drone position dots — all relative to compass center (ccx, ccy).
        {
        Vector3 self_pos = center;
        float dot_scale = grid_scale;
        for (int vi = 0; vi < vehicle_count; vi++) {
            if (!vehicles[vi].active && vehicle_count > 1) continue;
            // TOP view: +X = right on screen, +Z = down on screen
            float dx = vehicles[vi].position.x - self_pos.x;
            float dz = vehicles[vi].position.z - self_pos.z;
            float bx = ccx + dx * dot_scale;
            float by = ccy + dz * dot_scale;
            float dot_r = (vi == selected) ? ws(5, sw, sh) : ws(4, sw, sh);
            DrawCircle((int)bx, (int)by, dot_r, vehicles[vi].color);
        }

        // Correlation: curtain (mode 2) and/or line (mode 1)
        if (corr_mode > 0 && h->pinned_count > 0) {
            for (int p = 0; p < h->pinned_count; p++) {
                int pidx = h->pinned[p];
                if (pidx < 0 || pidx >= vehicle_count || !vehicles[pidx].active || pidx == selected) continue;
                const vehicle_t *va = &vehicles[selected];
                const vehicle_t *vb = &vehicles[pidx];

                if (corr_mode == 2) {
                    // Filled curtain: 2D triangle pairs between both trails (top-down projection)
                    int n = va->trail_count < vb->trail_count ? va->trail_count : vb->trail_count;
                    if (n >= 2) {
                        int sa = (va->trail_count < va->trail_capacity) ? 0 : va->trail_head;
                        int sb = (vb->trail_count < vb->trail_capacity) ? 0 : vb->trail_head;
                        for (int i = 1; i < n; i++) {
                            int ia0 = (sa + (int)((float)(i-1)/n * va->trail_count)) % va->trail_capacity;
                            int ia1 = (sa + (int)((float)i/n * va->trail_count)) % va->trail_capacity;
                            int ib0 = (sb + (int)((float)(i-1)/n * vb->trail_count)) % vb->trail_capacity;
                            int ib1 = (sb + (int)((float)i/n * vb->trail_count)) % vb->trail_capacity;
                            float t = (float)i / (float)n;
                            unsigned char al = (unsigned char)(t * 120);
                            Color mc = {(unsigned char)((va->color.r + vb->color.r)/2),
                                        (unsigned char)((va->color.g + vb->color.g)/2),
                                        (unsigned char)((va->color.b + vb->color.b)/2), al};
                            Vector2 a0 = {ccx + (va->trail[ia0].x - self_pos.x) * dot_scale,
                                          ccy + (va->trail[ia0].z - self_pos.z) * dot_scale};
                            Vector2 a1 = {ccx + (va->trail[ia1].x - self_pos.x) * dot_scale,
                                          ccy + (va->trail[ia1].z - self_pos.z) * dot_scale};
                            Vector2 b0 = {ccx + (vb->trail[ib0].x - self_pos.x) * dot_scale,
                                          ccy + (vb->trail[ib0].z - self_pos.z) * dot_scale};
                            Vector2 b1 = {ccx + (vb->trail[ib1].x - self_pos.x) * dot_scale,
                                          ccy + (vb->trail[ib1].z - self_pos.z) * dot_scale};
                            // Both winding orders for visibility
                            DrawTriangle(a0, b0, a1, mc);
                            DrawTriangle(a0, a1, b0, mc);
                            DrawTriangle(b0, b1, a1, mc);
                            DrawTriangle(b0, a1, b1, mc);
                        }
                    }
                }

                // Current position line (both modes)
                float dx_p = vb->position.x - self_pos.x;
                float dz_p = vb->position.z - self_pos.z;
                float bx_c = ccx + dx_p * dot_scale;
                float by_c = ccy + dz_p * dot_scale;
                Color mid = {(unsigned char)((va->color.r + vb->color.r)/2),
                             (unsigned char)((va->color.g + vb->color.g)/2),
                             (unsigned char)((va->color.b + vb->color.b)/2), 200};
                DrawLineEx((Vector2){ccx, ccy}, (Vector2){bx_c, by_c}, 2.0f, mid);
            }
        }
        }

        EndScissorMode();
    }

    // Panel border
    DrawRectangleLinesEx((Rectangle){px, py, ps, ps}, 1.0f, accent);

    // RADAR label
    float fs_label = wy(12, sh);
    DrawTextEx(h->font_value, "RADAR",
               (Vector2){px + wx(8, sw), py + wy(5, sh)}, fs_label, 0.5f, accent);
}

// ── Animated Gimbal Rings (bottom-center, above transport) ─────────────────
// Three rings per drone: Pitch (red), Roll (green), Yaw (cyan).
// Each ring is a circle in 3D rotated by the drone's live attitude, projected to 2D.
// Colors are 50% tinted toward the drone's palette color.
// Matches the gimbal ring rendering from clean-hud-v2.html.
static Color blend_color(Color base, Color drone, float t) {
    return (Color){
        (unsigned char)(base.r + (drone.r - base.r) * t),
        (unsigned char)(base.g + (drone.g - base.g) * t),
        (unsigned char)(base.b + (drone.b - base.b) * t),
        base.a
    };
}

// Project a 3D point to 2D using a fixed elevated camera (30deg above, looking down)
// Returns screen-space offset from center
static Vector2 project_ring_point(float x3, float y3, float z3) {
    // Fixed view: ~30 degrees elevation, looking slightly down
    // Camera at (0, 0.5, 1) normalized, projecting orthographically
    float cam_pitch = -30.0f * DEG2RAD; // 30 degrees above horizon
    float cos_p = cosf(cam_pitch);
    float sin_p = sinf(cam_pitch);
    // Rotate around X axis to get the elevated view
    float py = y3 * cos_p - z3 * sin_p;
    float pz = y3 * sin_p + z3 * cos_p;
    // Orthographic: x stays, projected y = py
    return (Vector2){ x3, -py }; // negate Y for screen coords (Y down)
}

// Draw a 3D circle projected to 2D at screen position (cx, cy)
// The circle lies in the plane defined by axis_a and axis_b, rotated by quaternion q
static void draw_projected_ring(float cx, float cy, float radius,
                                 Quaternion q, Vector3 axis_a, Vector3 axis_b,
                                 float scale, float thick, Color color) {
    int segs = 48;
    for (int i = 0; i < segs; i++) {
        float a1 = (float)i / segs * 2.0f * PI;
        float a2 = (float)(i + 1) / segs * 2.0f * PI;
        // Point on unit circle in the ring's plane
        Vector3 local1 = {
            axis_a.x * cosf(a1) + axis_b.x * sinf(a1),
            axis_a.y * cosf(a1) + axis_b.y * sinf(a1),
            axis_a.z * cosf(a1) + axis_b.z * sinf(a1)
        };
        Vector3 local2 = {
            axis_a.x * cosf(a2) + axis_b.x * sinf(a2),
            axis_a.y * cosf(a2) + axis_b.y * sinf(a2),
            axis_a.z * cosf(a2) + axis_b.z * sinf(a2)
        };
        // Rotate by quaternion
        Vector3 w1 = Vector3RotateByQuaternion(local1, q);
        Vector3 w2 = Vector3RotateByQuaternion(local2, q);
        // Project to 2D
        Vector2 p1 = project_ring_point(w1.x * radius, w1.y * radius, w1.z * radius);
        Vector2 p2 = project_ring_point(w2.x * radius, w2.y * radius, w2.z * radius);
        DrawLineEx(
            (Vector2){cx + p1.x * scale, cy + p1.y * scale},
            (Vector2){cx + p2.x * scale, cy + p2.y * scale},
            thick, color);
    }
}

static void tac_draw_gimbal_rings(const hud_t *h, const vehicle_t *vehicles,
                                   int vehicle_count, int sw, int sh,
                                   const theme_t *theme) {
    if (h->pinned_count == 0) return;

    float ring_y = wy(998, sh);

    // Fixed size: transport width (50% screen) / 15 max drones = cell width
    // This gives consistent sizing that fills the transport bar at max capacity
    float transport_w = sw * 0.5f;
    float cell_w = transport_w / 15.0f;
    float r = cell_w * 0.38f;
    float spacing = cell_w;
    float total_w = (h->pinned_count - 1) * spacing;
    float start_x = sw / 2.0f - total_w / 2.0f;

    float fs_id = r * 0.6f;
    float tint = 0.5f;

    Color base_roll  = (Color){255, 100, 100, 180};
    Color base_pitch = (Color){ 52, 211, 153, 180};
    Color base_yaw   = (Color){  0, 180, 204, 160};

    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;

        Color dc = vehicles[pidx].color;
        float cx = start_x + p * spacing;
        float cy = ring_y;

        // Decompose using the same corrected method as the 3D rings:
        // Extract yaw from forward vector, then factor out yaw to get pitch.
        Quaternion rot = vehicles[pidx].rotation;

        Vector3 fwd = Vector3RotateByQuaternion((Vector3){0, 0, -1}, rot);
        float yaw = atan2f(-fwd.x, -fwd.z);
        Quaternion q_yaw = QuaternionFromAxisAngle((Vector3){0, 1, 0}, yaw);

        Quaternion q_yaw_inv = QuaternionInvert(q_yaw);
        Quaternion q_remainder = QuaternionMultiply(q_yaw_inv, rot);
        Vector3 fwd_rem = Vector3RotateByQuaternion((Vector3){0, 0, -1}, q_remainder);
        float pitch = atan2f(fwd_rem.y, -fwd_rem.z);
        Quaternion q_pitch = QuaternionFromAxisAngle((Vector3){1, 0, 0}, pitch);

        Quaternion q_yaw_pitch = QuaternionMultiply(q_yaw, q_pitch);
        Quaternion q_full = rot;

        // Yaw ring (cyan, outermost) — yaw only, horizontal plane
        Color yc = blend_color(base_yaw, dc, tint);
        draw_projected_ring(cx, cy, 1.15f, q_yaw,
                            (Vector3){1, 0, 0}, (Vector3){0, 0, -1},
                            r, 1.0f, yc);

        // North indicator on yaw ring
        {
            Vector3 north_local = {cosf(0) * 1.15f, 0, -sinf(0) * 1.15f};
            Vector3 nw = Vector3RotateByQuaternion(north_local, q_yaw);
            Vector2 np = project_ring_point(nw.x * r, nw.y * r, nw.z * r);
            DrawCircle((int)(cx + np.x), (int)(cy + np.y),
                       ws(2.5f, sw, sh), (Color){255, 68, 68, 220});
        }

        // Pitch ring (green, middle) — yaw + pitch, vertical plane
        Color pc = blend_color(base_pitch, dc, tint);
        draw_projected_ring(cx, cy, 1.0f, q_yaw_pitch,
                            (Vector3){0, 0, -1}, (Vector3){0, 1, 0},
                            r, 1.2f, pc);

        // Roll ring (red, innermost) — full rotation, XY plane (perpendicular to forward axis)
        Color rc = blend_color(base_roll, dc, tint);
        draw_projected_ring(cx, cy, 0.85f, q_full,
                            (Vector3){1, 0, 0}, (Vector3){0, 1, 0},
                            r, 1.2f, rc);

        // Center dot
        DrawCircle((int)cx, (int)cy, ws(2, sw, sh), (Color){dc.r, dc.g, dc.b, 130});

        // Drone ID label below
        char id_buf[8];
        snprintf(id_buf, sizeof(id_buf), "D%d", pidx + 1);
        Vector2 tw = MeasureTextEx(h->font_value, id_buf, fs_id, 0.5f);
        DrawTextEx(h->font_value, id_buf,
                   (Vector2){cx - tw.x / 2, cy + r * 1.2f + wy(4, sh)},
                   fs_id, 0.5f, dc);
    }
}

// ── Transport Bar (bottom center, 25%-75%) ─────────────────────────────────
static void tac_draw_transport(const hud_t *h, const data_source_t *src,
                                int sw, int sh, const theme_t *theme) {
    const playback_state_t *pb = &src->playback;
    if (pb->duration_s <= 0.0f) return;

    bool connected = src->connected;
    float tr_y = wy(1053, sh); // wireframe transport at y~1053
    float tr_left = wx(480, sw);
    float tr_right = wx(1440, sw);

    Color accent = theme->hud_accent;
    Color value_c = theme->hud_value;
    Color dim_c = theme->hud_dim;
    float fs = wy(12, sh);
    float icon_sz = wy(8, sh);
    float cx = tr_left;

    // Play/Pause
    if (pb->paused || !connected) {
        DrawTriangle(
            (Vector2){cx, tr_y - icon_sz * 0.5f},
            (Vector2){cx, tr_y + icon_sz * 0.5f},
            (Vector2){cx + icon_sz * 0.7f, tr_y},
            connected ? accent : dim_c);
    } else {
        float bw = icon_sz * 0.25f;
        float bg_val = icon_sz * 0.12f;
        DrawRectangle((int)cx, (int)(tr_y - icon_sz * 0.4f),
                      (int)bw, (int)(icon_sz * 0.8f), accent);
        DrawRectangle((int)(cx + bw + bg_val * 2), (int)(tr_y - icon_sz * 0.4f),
                      (int)bw, (int)(icon_sz * 0.8f), accent);
    }
    cx += icon_sz + wx(8, sw);

    // Speed
    char spd_buf[16];
    snprintf(spd_buf, sizeof(spd_buf), "%.1fx", pb->speed);
    DrawTextEx(h->font_value, spd_buf, (Vector2){cx, tr_y - fs * 0.45f},
               fs, 0.5f, (pb->speed != 1.0f) ? accent : value_c);
    cx += MeasureTextEx(h->font_value, spd_buf, fs, 0.5f).x + wx(8, sw);

    // Current time
    int pos_s = (int)pb->position_s;
    char pos_buf[16];
    snprintf(pos_buf, sizeof(pos_buf), "%d:%02d", pos_s / 60, pos_s % 60);
    DrawTextEx(h->font_value, pos_buf, (Vector2){cx, tr_y - fs * 0.45f},
               fs, 0.5f, value_c);
    cx += MeasureTextEx(h->font_value, pos_buf, fs, 0.5f).x + wx(8, sw);

    // Duration
    int dur_s = (int)pb->duration_s;
    char dur_buf[16];
    snprintf(dur_buf, sizeof(dur_buf), "%d:%02d", dur_s / 60, dur_s % 60);
    Vector2 dur_w = MeasureTextEx(h->font_value, dur_buf, fs, 0.5f);

    // Progress bar
    float icons_w = (pb->looping ? wx(20, sw) : 0) + wx(20, sw);
    float right_reserved = dur_w.x + wx(8, sw) + icons_w + wx(8, sw);
    float prog_x = cx;
    float prog_w = tr_right - cx - right_reserved;
    if (prog_w < wx(40, sw)) prog_w = wx(40, sw);
    float prog_h = wy(4, sh);
    float prog_y = tr_y - prog_h / 2.0f;

    DrawRectangleRounded(
        (Rectangle){prog_x, prog_y, prog_w, prog_h},
        0.5f, 4, (Color){accent.r, accent.g, accent.b, 40});
    if (pb->progress > 0.0f) {
        float fill_w = prog_w * pb->progress;
        if (fill_w < prog_h) fill_w = prog_h;
        DrawRectangleRounded(
            (Rectangle){prog_x, prog_y, fill_w, prog_h},
            0.5f, 4, accent);
    }
    float dot_x = prog_x + prog_w * pb->progress;
    DrawCircle((int)dot_x, (int)(prog_y + prog_h / 2.0f), wy(4, sh), accent);

    // Mode markers
    if (pb->mode_changes && pb->mode_change_count > 0) {
        for (int i = 0; i < pb->mode_change_count; i++) {
            float t = pb->mode_changes[i].time_s / pb->duration_s;
            if (t < 0.0f || t > 1.0f) continue;
            float mx = prog_x + prog_w * t;
            bool past = (t <= pb->progress);
            Color tc = past ? (Color){255, 255, 255, 220}
                           : (Color){accent.r, accent.g, accent.b, 80};
            DrawCircle((int)mx, (int)(prog_y + prog_h / 2.0f), wy(2.5f, sh), tc);
        }
    }

    cx = prog_x + prog_w + wx(8, sw);
    DrawTextEx(h->font_value, dur_buf, (Vector2){cx, tr_y - fs * 0.45f},
               fs, 0.5f, dim_c);
    cx += dur_w.x + wx(8, sw);

    if (pb->looping) {
        float lr = wy(5, sh);
        float loop_cx = cx + lr;
        for (int d = 0; d < 300; d += 15) {
            float a1 = (float)d * DEG2RAD;
            float a2 = (float)(d + 15) * DEG2RAD;
            DrawLineEx(
                (Vector2){loop_cx + cosf(a1) * lr, tr_y + sinf(a1) * lr},
                (Vector2){loop_cx + cosf(a2) * lr, tr_y + sinf(a2) * lr},
                1.5f, accent);
        }
        cx += wx(14, sw);
    }

    Color interp_c = pb->interpolation ? accent : dim_c;
    DrawTextEx(h->font_value, "I", (Vector2){cx, tr_y - fs * 0.45f}, fs, 0.5f, interp_c);
}

// ── Toast ──────────────────────────────────────────────────────────────────
static void tac_draw_toast(const hud_t *h, int sw, int sh, const theme_t *theme) {
    if (h->toast_timer <= 0.0f) return;
    float fs = wy(14, sh);
    Vector2 tw = MeasureTextEx(h->font_label, h->toast_text, fs, 0.5f);
    float fade = (h->toast_timer < 0.5f) ? h->toast_timer / 0.5f : 1.0f;
    Color base = (h->toast_color.a > 0) ? h->toast_color : theme->hud_climb;
    Color tc = (Color){base.r, base.g, base.b, (unsigned char)(fade * 255)};
    DrawTextEx(h->font_label, h->toast_text,
               (Vector2){sw / 2.0f - tw.x / 2, wy(50, sh)}, fs, 0.5f, tc);
}

// ── Warnings ───────────────────────────────────────────────────────────────
static void tac_draw_warnings(const hud_t *h, bool has_tier3, bool has_awaiting_gps,
                               int sw, int sh, const theme_t *theme) {
    float fs = wy(13, sh);
    float base_y = wy(70, sh);
    if (has_tier3) {
        const char *t = "ESTIMATED POSITION";
        Vector2 tw = MeasureTextEx(h->font_label, t, fs, 0.5f);
        DrawTextEx(h->font_label, t,
                   (Vector2){sw / 2.0f - tw.x / 2, base_y}, fs, 0.5f, theme->hud_warn);
        base_y += tw.y + wy(4, sh);
    }
    if (has_awaiting_gps) {
        const char *t = "AWAITING GPS";
        Vector2 tw = MeasureTextEx(h->font_label, t, fs, 0.5f);
        DrawTextEx(h->font_label, t,
                   (Vector2){sw / 2.0f - tw.x / 2, base_y}, fs, 0.5f, theme->hud_warn);
    }
}

// ── Reticle ────────────────────────────────────────────────────────────────
static void tac_draw_reticle(int sw, int sh, const theme_t *theme) {
    float cx = sw / 2.0f;
    float cy = sh / 2.0f;
    Color r = (Color){theme->hud_value.r, theme->hud_value.g, theme->hud_value.b, 90};
    float l = ws(12, sw, sh);
    float g = ws(5, sw, sh);
    DrawLineEx((Vector2){cx - l, cy}, (Vector2){cx - g, cy}, 1.3f, r);
    DrawLineEx((Vector2){cx + g, cy}, (Vector2){cx + l, cy}, 1.3f, r);
    DrawCircle((int)cx, (int)cy, ws(2.5f, sw, sh), r);
}

// ── Main Draw ──────────────────────────────────────────────────────────────
void tactical_hud_draw(const hud_t *h, const vehicle_t *vehicles,
                       const data_source_t *sources, int vehicle_count,
                       int selected, int screen_w, int screen_h,
                       const theme_t *theme, bool ghost_mode,
                       bool has_tier3, bool has_awaiting_gps,
                       const ortho_panel_t *ortho,
                       int trail_mode, int corr_mode) {

    const data_source_t *src = &sources[selected];

    tac_draw_timer_status(h, src, screen_w, screen_h, theme);
    tac_draw_heading(h, &vehicles[selected], screen_w, screen_h, theme);
    tac_draw_ticker(h, &src->playback, screen_w, screen_h, theme);
    tac_draw_toast(h, screen_w, screen_h, theme);
    tac_draw_warnings(h, has_tier3, has_awaiting_gps, screen_w, screen_h, theme);
    tac_draw_speed_stack(h, vehicles, vehicle_count, selected,
                         screen_w, screen_h, theme);
    tac_draw_alt_stack(h, vehicles, vehicle_count, selected,
                       screen_w, screen_h, theme);
    tac_draw_radar(h, vehicles, vehicle_count, selected,
                   screen_w, screen_h, theme, ortho, trail_mode, corr_mode);
    tac_draw_gimbal_rings(h, vehicles, vehicle_count, screen_w, screen_h, theme);
    tac_draw_transport(h, src, screen_w, screen_h, theme);
}
