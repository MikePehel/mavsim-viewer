#version 330

/*
 * Fragment shader for the clipmap terrain rings.
 *
 * Wireframe presentation: theme-coloured lines over a near-black fill,
 * with anti-aliased edge detection, soft glow, and distance fog.
 * Backface culling stays ON (raylib default).
 *
 * Edge detection uses a per-quad UV
 * derived from `fract(fragMeshUV * meshResolution)`. The vertex shader
 * forwards the raw mesh-local UV (0..1 across the patch mesh, before the
 * heightmap subregion remap). Multiplying by `meshResolution = RING_VERTS - 1`
 * gives a sawtooth that resets at every vertex, so the fract is in [0, 1]
 * within each mesh quad; values near 0 or 1 are quad edges. We also detect
 * the triangle diagonal (`abs(q.x - q.y) ≈ 0`) so the wireframe shows the
 * actual triangulation, not just quad outlines.
 *
 * keep-band discard:
 *   To eliminate the inner+outer ring overlap that produced T-junctions /
 *   visible double-mesh in the inner ring's coverage zone, each ring
 *   declares its keep-band [keepBandMin, keepBandMax] in world metres from
 *   the camera. Fragments outside that band discard.
 *
 * The alpha-fade crossfade (ringFadeStart / ringFadeEnd) is
 * preserved as a uniform but the renderer sets values that keep
 * smoothstep at 0 (no fade) under architecture.
 */

in vec3  fragWorldPos;
in vec2  fragTexCoord;
in float fragRingDist;
in vec3  fragWorldNormal;
in vec2  fragMeshUV;

uniform vec3  camPos;
uniform vec4  baseColor;        /* theme ground tint -- unused under wireframe; kept for the renderer's uniform-set loop */
uniform vec3  lightDir;         /* unused under wireframe */
uniform float ambient;          /* unused under wireframe */
uniform float ringFadeStart;    /* vestigial */
uniform float ringFadeEnd;      /* vestigial */
uniform float keepBandMin;      /* discard if r < keepBandMin (world metres) */
uniform float keepBandMax;      /* discard if r > keepBandMax (world metres) */
uniform float meshResolution;   /* RING_VERTS - 1 (currently 254) */
uniform vec3  wireColorA;       /* low-elevation wireframe colour */
uniform vec3  wireColorB;       /* high-elevation wireframe colour */
uniform float wireYMin;         /* gradient low end (world metres Y) */
uniform float wireYMax;         /* gradient high end (world metres Y) */
uniform vec3  fillColor;        /* theme-driven dark fill colour */
uniform vec3  fogColor;         /* theme-driven distance fog colour */
uniform int   uSolidMode;       /* 0 = wireframe path; 1 = Lambertian solid */

/*
 * Shading sub-mode (only consulted when uSolidMode != 0).
 *
 *   0  Curvature AO   — Lambert + 4-tap heightmap concavity term; works
 *                       with the ground-grid texture under it
 *   1  Toon (3 bands) — stylized cel shading with hard band edges;
 *                       scene.c suppresses the ground texture under it
 *
 * Both modes share the per-fragment normal computation below (4
 * heightmap lookups), so the visual baseline is the same — only the
 * BRDF / occlusion model varies.
 */
uniform int   uShadingMode;

/*
 * Per-fragment normal recomputation needs the heightmap + the ring's
 * world-space scale to convert texel offsets into metres. Same uniforms
 * the vertex shader uses (raylib's uniform binding is shared across stages
 * by name, so declaring them again here is free).
 */
uniform sampler2D heightTex;
uniform float     ringScale;

out vec4 finalColor;

void main() {
    /* keep-band: drop fragments outside this ring's territory. */
    float distToCam = length(fragWorldPos.xz - camPos.xz);
    if (distToCam < keepBandMin) discard;
    if (distToCam > keepBandMax) discard;

    /*
     * Hide terrain mesh that's at or near the grid-plane altitude.
     * Two reasons:
     *   - terrain.c's FLAT_R_M = 5 m disk forces terrain() to exactly
     *     0 around home, producing a flat patch coplanar with the grid.
     *   - Anywhere fBm output is within a few decimetres of 0 the mesh
     *     visually fights the grid (thin sliver triangles at the
     *     y=0 plane).
     * Discard fragments whose world-space Y is within FLAT_HIDE_M of
     * the grid altitude (0). The threshold trades off how much
     * near-flat terrain is hidden against how much "real" gentle
     * terrain is preserved. 0.5 m hides z-fight without dropping any
     * meaningful elevation given AMP=20 fBm scales.
     */
    const float FLAT_HIDE_M = 0.5;
    if (abs(fragWorldPos.y) < FLAT_HIDE_M) discard;

    /*
     * solid Lambertian mode (F-key in scene.c). When
     * active we skip the wireframe path entirely and output the theme's
     * fillColor (= theme.ground) shaded by a basic Lambertian against
     * the heightmap-derived normal. The grid pattern is provided by the
     * grid_plane underneath the terrain mesh; the terrain mesh is
     * intentionally untextured. Distance fog still applies so the LOD
     * -> billboard horizon hand-off stays smooth.
     */
    if (uSolidMode != 0) {
        /*
         * Per-fragment normal from heightmap central differences. The
         * vertex-shader-computed `fragWorldNormal` is Gouraud-interpolated
         * across each triangle, which (combined with sharp shading)
         * makes the mesh quads visible. Recomputing here from the bilinear
         * heightmap sample gives a normal that varies smoothly across the
         * entire surface, hiding the mesh tessellation.
         */
        vec2 texelSize = 1.0 / vec2(textureSize(heightTex, 0));
        float stepM_x  = (2.0 * ringScale) * texelSize.x;
        float stepM_z  = (2.0 * ringScale) * texelSize.y;
        float h0  = texture(heightTex, fragTexCoord).r;
        float hMx = texture(heightTex, fragTexCoord - vec2(texelSize.x, 0.0)).r;
        float hPx = texture(heightTex, fragTexCoord + vec2(texelSize.x, 0.0)).r;
        float hMz = texture(heightTex, fragTexCoord - vec2(0.0, texelSize.y)).r;
        float hPz = texture(heightTex, fragTexCoord + vec2(0.0, texelSize.y)).r;
        vec3 tangentX = vec3(2.0 * stepM_x, hPx - hMx, 0.0);
        vec3 tangentZ = vec3(0.0,           hPz - hMz, 2.0 * stepM_z);
        vec3 n = normalize(cross(tangentZ, tangentX));
        vec3 l = normalize(lightDir);

        vec3 col;

        if (uShadingMode == 0) {
            /* Mode 0 — Curvature AO. Compare h0 to the avg of its 4
             * cardinal neighbours; if h0 is below the avg, fragment is
             * in a concavity → darken. Cheap AO that costs no extra
             * texture lookups (reuses the 4 normal-calc samples) and
             * gives the strongest sense of valley depth. Lambert
             * still does the directional lighting. */
            float avgN = 0.25 * (hMx + hPx + hMz + hPz);
            float concavity = clamp((avgN - h0) / max(stepM_x, 1.0), -1.0, 1.0);
            float ao        = 1.0 - 0.6 * max(concavity, 0.0);
            float lambert   = clamp(dot(n, l), 0.0, 1.0);
            lambert         = pow(lambert, 1.5);
            float shade     = (ambient * 0.5 + (1.0 - ambient * 0.5) * lambert) * ao;
            col             = fillColor * shade;

        } else /* uShadingMode == 1 */ {
            /* Mode 1 — Toon (3-band cel) with HARD edges + deeper shadows.
             * Pre-shape Lambert with pow(1.3) to push more of the surface
             * into shadow, then quantise into 3 brightness bands via
             * floor() for crisp staircase transitions. Shadow floor 0.15
             * gives real shadow weight without going pitch-black. The
             * scene-side gate suppresses the ground-grid texture under
             * this mode so the terrain reads as flat blocks of colour. */
            float lambert = clamp(dot(n, l), 0.0, 1.0);
            lambert       = pow(lambert, 1.3);
            float bands   = floor(lambert * 3.0) / 2.0;     /* hard 0 / 0.5 / 1 */
            float shade   = 0.15 + 0.85 * bands;            /* shadow floor 0.15 */
            col           = fillColor * shade;
        }

        float fogS = smoothstep(500.0, 4000.0, distToCam);
        finalColor = vec4(mix(col, fogColor, fogS), 1.0);
        return;
    }

    /* wireframe: per-quad UV → triangle edges via fwidth-based AA. */
    vec2 q = fract(fragMeshUV * meshResolution);
    float minQuadEdge = min(min(q.x, q.y), min(1.0 - q.x, 1.0 - q.y));
    /* raylib's GenMeshPlane triangulates each quad with the diagonal from
     * (i, j) to (i+1, j+1), so q.x == q.y is the diagonal line. */
    float minDiagEdge = abs(q.x - q.y);
    float minEdge = min(minQuadEdge, minDiagEdge);

    float aa  = fwidth(minEdge);
    float wireMask = 1.0 - smoothstep(0.0, aa * 1.5, minEdge);
    float glowMask = 1.0 - smoothstep(0.0, aa * 5.0, minEdge);

    /* elevation gradient: blend wireA -> wireB across the terrain
     * amplitude band. When wireA == wireB (non-gradient themes) the mix
     * collapses to a single colour. */
    float yBlend = smoothstep(wireYMin, wireYMax, fragWorldPos.y);
    vec3 wireColor = mix(wireColorA, wireColorB, yBlend);

    vec3 color = mix(fillColor, wireColor, wireMask);
    color = mix(color, wireColor, glowMask * 0.2);

    /* Distance fog: fade to the theme's fog colour between 500 m and 4 km.
     * The middle ring's outer edge sits at 1024 m so it remains clear; the
     * outer ring (1024..3000 m) blends to fog; the billboard at 5 km picks
     * up at full fog. */
    float fog = smoothstep(500.0, 4000.0, distToCam);
    color = mix(color, fogColor, fog);

    /* Vestigial ring-edge alpha fade (renderer sets it to a no-op). */
    float ringAlpha = 1.0 - smoothstep(ringFadeStart, ringFadeEnd, fragRingDist);

    finalColor = vec4(color, ringAlpha);
}
