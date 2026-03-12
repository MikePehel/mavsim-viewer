#version 330

in vec3 fragWorldPos;
in vec2 fragTexCoord;
in vec4 fragColor;

uniform vec4 colGround;
uniform vec4 colMinor;
uniform vec4 colMajor;
uniform vec4 colAxisX;
uniform vec4 colAxisZ;
uniform float spacing;
uniform float majorEvery;
uniform float axisWidth;

uniform int texEnabled;
uniform sampler2D groundTex;
uniform vec4 colFog;
uniform vec4 colTint;

// Underwater uniforms
uniform int isUnderwater;
uniform float uTime;

// Fog distance control
uniform float fogStart;  // default 400
uniform float fogEnd;    // default 800

out vec4 finalColor;

float caustic(vec2 uv, float t) {
    vec2 uv1 = uv * 8.0 + vec2(t * 0.3, t * 0.2);
    vec2 uv2 = uv * 6.0 + vec2(-t * 0.2, t * 0.35);

    float c1 = sin(uv1.x + sin(uv1.y * 0.8))
             * sin(uv1.y + sin(uv1.x * 0.6));
    float c2 = sin(uv2.x + sin(uv2.y * 0.7))
             * sin(uv2.y + sin(uv2.x * 0.9));

    float c = (c1 + c2) * 0.5;
    return clamp(c * c, 0.0, 1.0);
}

void main() {
    vec2 coord = fragWorldPos.xz;
    float dist = length(coord);

    if (isUnderwater != 0) {
        // ── Underwater: caustic pattern + grid lines ──
        float c = caustic(coord * 0.01, uTime);

        // Pool-style caustics: bright web lines around small darker cells
        // Invert pattern so lines (edges) are bright, cell interiors are dark
        float web = 1.0 - c;
        float bright = smoothstep(0.997, 0.9999, web);
        // White-hot core at the very peak
        float white = smoothstep(0.999993, 0.999999, web);

        float causticOn = float(1 - texEnabled);
        vec3 base = colGround.rgb;
        vec3 highlight = mix(colMinor.rgb * 0.5, colMinor.rgb * 1.5, white) * bright * causticOn;
        vec4 color = vec4(base + highlight, 1.0);

        // Grid lines on top of caustics
        vec2 grid = abs(fract(coord / spacing - 0.5) - 0.5) / fwidth(coord / spacing);
        float lineMinor = 1.0 - clamp(min(grid.x, grid.y), 0.0, 1.0);

        float majorSpacing = spacing * majorEvery;
        vec2 gridMajor = abs(fract(coord / majorSpacing - 0.5) - 0.5) / fwidth(coord / majorSpacing);
        float lineMajor = 1.0 - clamp(min(gridMajor.x, gridMajor.y), 0.0, 1.0);

        vec2 axisGrid = abs(coord) / fwidth(coord);
        float axisXLine = 1.0 - clamp(axisGrid.y / axisWidth, 0.0, 1.0);
        float axisZLine = 1.0 - clamp(axisGrid.x / axisWidth, 0.0, 1.0);

        color = mix(color, vec4(colMinor.rgb * 0.3, 1.0), lineMinor * 0.4);
        color = mix(color, vec4(colMinor.rgb * 0.5, 1.0), lineMajor * 0.6);
        color = mix(color, colAxisX, axisXLine * colAxisX.a * 0.5);
        color = mix(color, colAxisZ, axisZLine * colAxisZ.a * 0.5);

        // Same fog distance as normal mode
        float fade = 1.0 - smoothstep(fogStart, fogEnd, dist);
        color.rgb = mix(colFog.rgb, color.rgb, fade);

        color.a = 1.0;
        finalColor = color;
        return;
    }

    // ── Normal grid rendering ──
    // Ground color: flat or terrain-textured with distance fog
    vec4 ground = colGround;
    if (texEnabled != 0) {
        // Each world tile is 10x10 meters
        vec2 tileCoord = floor(coord / 10.0);
        vec2 tileUV = fract(coord / 10.0);

        // Hash tile coordinate to pick one of 8 atlas variants (0-7)
        float h = fract(sin(dot(tileCoord, vec2(127.1, 311.7))) * 43758.5453);
        int variant = int(h * 8.0);
        variant = clamp(variant, 0, 7);

        // Map UV into correct atlas cell (4 cols × 2 rows)
        vec2 atlasOffset = vec2(float(variant % 4) / 4.0, float(variant / 4) / 2.0);
        vec2 texUV = atlasOffset + tileUV * vec2(1.0 / 4.0, 1.0 / 2.0);

        vec4 texColor = texture(groundTex, texUV);
        // Normalize texture luminance to a detail signal centered on 1.0
        float lum = dot(texColor.rgb, vec3(0.299, 0.587, 0.114));
        float detail = (lum - 0.125) / 0.125;
        // Apply detail as brightness variation on tint color
        vec3 tinted = colTint.rgb + colTint.rgb * detail * 0.35;
        // Fog: texture visible up close, fades to fog color at distance
        float texFade = 1.0 - smoothstep(80.0, 200.0, dist);
        ground = mix(colFog, vec4(tinted, 1.0), texFade);
    }

    // Minor grid lines
    vec2 grid = abs(fract(coord / spacing - 0.5) - 0.5) / fwidth(coord / spacing);
    float lineMinor = 1.0 - clamp(min(grid.x, grid.y), 0.0, 1.0);

    // Major grid lines
    float majorSpacing = spacing * majorEvery;
    vec2 gridMajor = abs(fract(coord / majorSpacing - 0.5) - 0.5) / fwidth(coord / majorSpacing);
    float lineMajor = 1.0 - clamp(min(gridMajor.x, gridMajor.y), 0.0, 1.0);

    // Axis lines (world origin)
    vec2 axisGrid = abs(coord) / fwidth(coord);
    float axisXLine = 1.0 - clamp(axisGrid.y / axisWidth, 0.0, 1.0); // Z=0 line (X axis)
    float axisZLine = 1.0 - clamp(axisGrid.x / axisWidth, 0.0, 1.0); // X=0 line (Z axis)

    // Compose: ground -> minor -> major -> axes
    vec4 color = ground;
    color = mix(color, colMinor, lineMinor * colMinor.a);
    color = mix(color, colMajor, lineMajor * colMajor.a);
    color = mix(color, colAxisX, axisXLine * colAxisX.a);
    color = mix(color, colAxisZ, axisZLine * colAxisZ.a);

    // Distance fade — prevent aliasing at horizon
    float fade = 1.0 - smoothstep(fogStart, fogEnd, dist);
    color = mix(ground, color, fade);

    color.a = 1.0;
    finalColor = color;
}
