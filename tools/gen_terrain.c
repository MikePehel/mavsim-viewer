// Standalone terrain texture generator — outputs terrain.png
// Usage: cc -O2 -o gen_terrain gen_terrain.c -lm && ./gen_terrain
//
// Generates a 1024x1024 atlas with 4 tiles (2x2 grid, 512x512 each).
// Double domain-warped FBM + Voronoi pebbles + warm color variation.
// Torus-mapped noise for seamless tile edges.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── stb_image_write (PNG only) ──────────────────────────────────────────────
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ── Config ──────────────────────────────────────────────────────────────────
#define TILE_SIZE   256
#define ATLAS_COLS  8
#define ATLAS_ROWS  4
#define VARIANTS    (ATLAS_COLS * ATLAS_ROWS)  // 32
#define ATLAS_W     (ATLAS_COLS * TILE_SIZE)   // 2048
#define ATLAS_H     (ATLAS_ROWS * TILE_SIZE)   // 1024

// ── Hash functions ──────────────────────────────────────────────────────────
static float gtex_hash(int x, int y) {
    unsigned int h = (unsigned int)(x * 12979 + y * 87811 + 52967);
    h ^= h >> 13; h *= 0x5bd1e995; h ^= h >> 15;
    return (float)(h & 0xFFFF) / 65535.0f;
}

static void gtex_hash2(int x, int y, float *ox, float *oy) {
    unsigned int h1 = (unsigned int)(x * 12979 + y * 87811 + 52967);
    h1 ^= h1 >> 13; h1 *= 0x5bd1e995; h1 ^= h1 >> 15;
    unsigned int h2 = (unsigned int)(x * 73939 + y * 14197 + 81239);
    h2 ^= h2 >> 13; h2 *= 0x5bd1e995; h2 ^= h2 >> 15;
    *ox = (float)(h1 & 0xFFFF) / 65535.0f;
    *oy = (float)(h2 & 0xFFFF) / 65535.0f;
}

static float gtex_hash4(int x, int y, int z, int w) {
    unsigned int h = (unsigned int)(x * 12979 + y * 87811 + z * 45053 + w * 67867 + 52967);
    h ^= h >> 13; h *= 0x5bd1e995; h ^= h >> 15;
    return (float)(h & 0xFFFF) / 65535.0f;
}

// ── 4D value noise ──────────────────────────────────────────────────────────
static float gtex_vnoise4d(float x, float y, float z, float w) {
    int ix = (int)floorf(x), iy = (int)floorf(y);
    int iz = (int)floorf(z), iw = (int)floorf(w);
    float fx = x - ix, fy = y - iy, fz = z - iz, fw = w - iw;
    fx = fx*fx*(3-2*fx); fy = fy*fy*(3-2*fy);
    fz = fz*fz*(3-2*fz); fw = fw*fw*(3-2*fw);

    float result = 0;
    for (int dw = 0; dw <= 1; dw++)
        for (int dz = 0; dz <= 1; dz++)
            for (int dy = 0; dy <= 1; dy++)
                for (int dx = 0; dx <= 1; dx++) {
                    float val = gtex_hash4(ix+dx, iy+dy, iz+dz, iw+dw);
                    float wx2 = dx ? fx : (1-fx);
                    float wy2 = dy ? fy : (1-fy);
                    float wz2 = dz ? fz : (1-fz);
                    float ww2 = dw ? fw : (1-fw);
                    result += val * wx2 * wy2 * wz2 * ww2;
                }
    return result;
}

// ── Tileable noise via torus mapping ────────────────────────────────────────
static float gtex_tnoise(float u, float v, float freq) {
    float a = u * 2.0f * (float)M_PI;
    float b = v * 2.0f * (float)M_PI;
    return gtex_vnoise4d(
        cosf(a) * freq, sinf(a) * freq,
        cosf(b) * freq, sinf(b) * freq);
}

static float gtex_fbm(float u, float v) {
    return gtex_tnoise(u, v, 1.0f) * 0.50f
         + gtex_tnoise(u, v, 2.0f) * 0.30f
         + gtex_tnoise(u, v, 4.0f) * 0.20f;
}

// ── Tileable Voronoi ────────────────────────────────────────────────────────
static float gtex_voronoi_f1(float u, float v, int cells) {
    float x = u * cells, y = v * cells;
    int ix = (int)floorf(x), iy = (int)floorf(y);
    float fx = x - ix, fy = y - iy;
    float f1 = 99.0f;
    for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
            int cx = ((ix + i) % cells + cells) % cells;
            int cy = ((iy + j) % cells + cells) % cells;
            float ox, oy;
            gtex_hash2(cx, cy, &ox, &oy);
            float dx = (float)i + ox - fx, dy = (float)j + oy - fy;
            float d = sqrtf(dx*dx + dy*dy);
            if (d < f1) f1 = d;
        }
    }
    return f1;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ── Main ────────────────────────────────────────────────────────────────────
int main(void) {
    unsigned char *pixels = calloc(ATLAS_W * ATLAS_H * 3, 1);  // RGB
    if (!pixels) { fprintf(stderr, "alloc failed\n"); return 1; }

    float tile_inv = 1.0f / (float)TILE_SIZE;

    float seed_offsets[VARIANTS][2] = {
        {  0.0f,  0.0f }, { 17.3f, 41.7f }, { 63.2f, 11.9f }, { 34.8f, 78.1f },
        { 91.4f, 23.6f }, { 48.7f, 55.3f }, { 12.1f, 89.4f }, { 76.5f, 37.2f },
        { 25.9f, 64.3f }, { 82.1f, 19.8f }, { 53.7f, 72.4f }, { 39.6f, 46.1f },
        {  7.8f, 95.2f }, { 68.4f, 31.5f }, { 44.2f, 83.7f }, { 15.6f, 58.9f },
        { 88.3f, 10.4f }, { 29.7f, 67.8f }, { 71.5f, 42.3f }, { 56.1f, 26.9f },
        {  3.4f, 81.6f }, { 62.8f, 14.7f }, { 47.3f, 93.5f }, { 19.2f, 52.6f },
        { 85.7f, 38.1f }, { 33.4f, 75.9f }, { 74.6f, 21.3f }, { 58.2f, 49.8f },
        { 11.5f, 86.4f }, { 66.9f, 33.2f }, { 41.8f, 70.6f }, { 97.1f, 16.5f },
    };

    printf("Generating %dx%d terrain atlas (%d tiles at %dx%d)...\n",
           ATLAS_W, ATLAS_H, VARIANTS, TILE_SIZE, TILE_SIZE);

    for (int tile = 0; tile < VARIANTS; tile++) {
        int tile_ox = (tile % ATLAS_COLS) * TILE_SIZE;
        int tile_oy = (tile / ATLAS_COLS) * TILE_SIZE;
        float sox = seed_offsets[tile][0];
        float soy = seed_offsets[tile][1];

        for (int ty = 0; ty < TILE_SIZE; ty++) {
            for (int tx = 0; tx < TILE_SIZE; tx++) {
                float u = (float)tx * tile_inv;
                float v = (float)ty * tile_inv;

                float wu = sinf(u * (float)M_PI);
                float wv = sinf(v * (float)M_PI);
                float window = wu * wu * wv * wv;

                float qx = gtex_fbm(u, v);
                float qy = gtex_fbm(u + 0.52f, v + 0.13f);
                float base_warped = gtex_fbm(u + 0.15f*qx, v + 0.15f*qy);

                float dx = gtex_fbm(u + sox, v + soy);
                float dy = gtex_fbm(u + sox + 0.52f, v + soy + 0.13f);
                float detail_warped = gtex_fbm(u + 0.4f*dx + sox, v + 0.4f*dy + soy);
                float detail = (detail_warped - 0.5f) * window;

                float warped = base_warped + detail * 1.2f;

                float f1_base = gtex_voronoi_f1(u, v, 18);
                float f1_detail = gtex_voronoi_f1(u + sox, v + soy, 14);
                float f1 = f1_base * (1.0f - window * 0.8f) + f1_detail * window * 0.8f;
                float pebble = powf(clampf(1.0f - f1 * 2.5f, 0, 1), 4.0f) * 0.5f;

                float grit = gtex_tnoise(u + sox, v + soy, 8.0f);

                float brightness = 32.0f;
                brightness += (warped - 0.5f) * 16.0f;
                brightness += pebble * 12.0f;
                brightness += (grit - 0.5f) * 3.0f;
                brightness = clampf(brightness, 20.0f, 46.0f);

                float warm = (qx + detail * 0.6f - 0.5f) * 0.12f;
                float r = brightness * (1.0f + warm) + 2.0f;
                float g = brightness + 1.0f;
                float b = brightness * (1.0f - warm);

                int px = tile_ox + tx;
                int py = tile_oy + ty;
                int idx = (py * ATLAS_W + px) * 3;
                pixels[idx + 0] = (unsigned char)clampf(r, 0, 255);
                pixels[idx + 1] = (unsigned char)clampf(g, 0, 255);
                pixels[idx + 2] = (unsigned char)clampf(b, 0, 255);
            }
        }
        printf("  tile %d/%d done\n", tile + 1, VARIANTS);
    }

    const char *outpath = "terrain_1024.png";
    if (stbi_write_png(outpath, ATLAS_W, ATLAS_H, 3, pixels, ATLAS_W * 3)) {
        printf("Wrote %s\n", outpath);
    } else {
        fprintf(stderr, "Failed to write %s\n", outpath);
    }

    free(pixels);
    return 0;
}
