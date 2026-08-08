#version 330

/*
 * Fragment shader for the far-horizon panoramic billboard.
 *
 * The panorama texture stores a silhouette per column:
 *   R = terrain shading at this pixel (Lambert pre-multiplied on CPU)
 *   G = unused (reserved)
 *   B = unused (reserved)
 *   A = 1 below the silhouette (terrain), 0 above (sky)
 *
 * `baseColor` is the Hawkeye theme ground tint; we modulate the pre-baked
 * shading by it. `skyColor` is unused here — alpha=0 lets the scene's
 * existing sky show through.
 */

in vec3 fragWorldPos;
in vec2 fragTexCoord;

uniform sampler2D panoramaTex;
uniform vec4      baseColor;

out vec4 finalColor;

void main() {
    vec4 t = texture(panoramaTex, fragTexCoord);
    if (t.a < 0.05) discard;  /* sky: let the existing scene background show through */

    vec3 col = baseColor.rgb * t.r;
    finalColor = vec4(col, t.a * baseColor.a);
}
