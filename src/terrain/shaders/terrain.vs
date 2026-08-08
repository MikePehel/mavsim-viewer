#version 330

/*
 * Vertex shader for the clipmap terrain rings.
 *
 * The CPU pre-evaluates the procedural terrain function into a per-ring
 * R32F heightmap; this shader samples the heightmap for displacement and
 * emits world-space position in raylib-native coordinates.
 *
 * Frame convention (matches the rest of Hawkeye: see vehicle.c:546-555):
 *   raylib +X = East, +Y = Up, +Z = -North (i.e. South).
 *   The CPU evaluator converts to NED before calling terrain(N, E), so the
 *   shader itself never sees N/E — it works entirely in raylib XZ space.
 *
 * Patch model:
 *   The shared mesh covers [-1, +1]^2 in local space. A "patch" is a
 *   sub-region of the ring drawn by one draw call; patchOffset + patchScale
 *   map mesh-local [-1, +1] coords into the appropriate sub-region of the
 *   ring's local [-1, +1] coords. The inner ring uses one patch with
 *   patchScale = 1.0 / patchOffset = 0; the outer ring uses 4 patches
 *   (one per quadrant) with patchScale = 0.5 / patchOffset = +/- 0.5.
 *   The same offset/scale pair drives the heightmap texCoord remap.
 *
 * Geomorph:
 *   When drawing the inner ring (geomorphActive == 1), vertices in the
 *   outer 15 % of the ring (fragRingDist > 0.85) blend their displacement
 *   from the inner ring's heightmap toward the outer ring's heightmap
 *   sampled at the same world XZ. At fragRingDist == 1.0 the inner ring's
 *   edge vertices have Y equal to outerH(worldXZ) so they exactly match
 *   the surface the outer ring would render at that point. Combined with
 *   the fragment-shader keep-band discard that prevents either ring from
 *   drawing in the other's territory, this gives a single continuous
 *   surface with no T-junctions and no Y discontinuity at the boundary.
 */

in vec3 vertexPosition;     /* x in [-1,1], y=0, z in [-1,1] (raylib local) */
in vec2 vertexTexCoord;     /* (u, v) in [0, 1] — local mesh tex coord */
in vec4 vertexColor;

uniform mat4 mvp;
uniform mat4 matModel;      /* identity here; kept for raylib's auto-binding */

uniform float     ringScale;        /* ring half-extent in metres */
uniform vec2      ringCenterXZ;     /* world raylib (x, z) of ring centre */
uniform vec2      patchOffset;      /* ring-local offset of the patch centre, in [-1, +1] */
uniform float     patchScale;       /* ring-local half-extent of this patch, in [0, 1] */
uniform vec2      texCoordOffset;   /* heightmap subregion origin, in [0, 1] */
uniform float     texCoordScale;    /* heightmap subregion half-extent, in [0, 1] */
uniform sampler2D heightTex;        /* R32F heightmap, RING_VERTS x RING_VERTS */
uniform vec3      camPos;           /* world camera position (raylib Y-up) */

/* geomorph uniforms. Only meaningful when geomorphActive == 1. */
uniform int       geomorphActive;
uniform sampler2D outerHeightTex;
uniform vec2      outerRingCenterXZ;
uniform float     outerRingScale;

out vec3  fragWorldPos;
out vec2  fragTexCoord;
out float fragRingDist;             /* unitless [0..~1.41] — vertex distance from ring centre */
out vec3  fragWorldNormal;
out vec2  fragMeshUV;               /* raw mesh-local UV [0,1]^2 across the patch mesh
                                       (pre-patchOffset/Scale remap), used by the
                                       wireframe fragment shader for quad-edge
                                       detection. */

void main() {
    /* Heightmap address: sample the ring's heightmap at the patch's sub-region. */
    vec2 ringTexCoord = texCoordOffset + vertexTexCoord * texCoordScale;
    float h = texture(heightTex, ringTexCoord).r;

    /*
     * Normal estimate via heightmap central differences. One ring texel
     * spans (2 * ringScale) / RING_VERTS metres regardless of which patch
     * we're inside, so the gradient uses the full-ring texel size.
     */
    vec2 texelSize = 1.0 / vec2(textureSize(heightTex, 0));
    float stepM_x = (2.0 * ringScale) * texelSize.x;
    float stepM_z = (2.0 * ringScale) * texelSize.y;
    float hPlusX  = texture(heightTex, ringTexCoord + vec2(texelSize.x, 0.0)).r;
    float hPlusZ  = texture(heightTex, ringTexCoord + vec2(0.0, texelSize.y)).r;
    vec3 tangentX = vec3(stepM_x, hPlusX - h, 0.0);
    vec3 tangentZ = vec3(0.0,     hPlusZ - h, stepM_z);
    vec3 worldNormal = normalize(cross(tangentZ, tangentX));

    /*
     * Ring-local position: map mesh-local [-1, +1] into the patch's
     * sub-region of the ring's [-1, +1] coordinate space, then scale up
     * to ring half-extent in metres.
     */
    vec2 ringLocal = patchOffset + vertexPosition.xz * patchScale;
    vec2 worldXZ = ringCenterXZ + ringLocal * ringScale;
    float ringDist = length(ringLocal);

    /*
     * geomorph: in the inner ring's outer 15 %, blend Y toward the
     * outer ring's heightmap value at the same world XZ. The smoothstep
     * weights 0 at ringDist=0.85 and 1 at ringDist=1.0; at the edge
     * (ringDist=1.0) the inner ring's vertex Y is fully from the outer
     * heightmap, so it agrees with the outer ring's surface there.
     *
     * Compute the outer heightmap's UV for this XZ. The outer ring covers
     * [outerCenter - outerScale, outerCenter + outerScale] in each axis;
     * UV is in [0, 1] across that range.
     */
    if (geomorphActive == 1 && ringDist > 0.85) {
        vec2 outerUV = (worldXZ - outerRingCenterXZ) / (2.0 * outerRingScale) + 0.5;
        if (outerUV.x >= 0.0 && outerUV.x <= 1.0 &&
            outerUV.y >= 0.0 && outerUV.y <= 1.0) {
            float outerH = texture(outerHeightTex, outerUV).r;
            float blend = smoothstep(0.85, 1.0, ringDist);
            h = mix(h, outerH, blend);
        }
    }

    vec3 worldPos = vec3(worldXZ.x, h, worldXZ.y);

    fragWorldPos    = worldPos;
    fragTexCoord    = ringTexCoord;
    fragRingDist    = ringDist;
    fragWorldNormal = worldNormal;
    fragMeshUV      = vertexTexCoord;

    gl_Position = mvp * vec4(worldPos, 1.0);
}
