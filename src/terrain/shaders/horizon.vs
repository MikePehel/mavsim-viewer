#version 330

/*
 * Vertex shader for the far-horizon panoramic billboard.
 *
 * The billboard is a 12-quad cylindrical strip anchored to the camera
 * position (translated each frame), at radius ~9 km and height ~+/- 400 m.
 * The CPU pre-renders a 1024 x 256 RGBA panoramic texture by sampling
 * `terrain()` along angular bearings and computing silhouette heights;
 * this shader does nothing more than pass through the vertex and unwrap
 * the (u, v) texcoord. Sky fill / alpha discard live in horizon.fs.
 */

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;

uniform mat4 mvp;
uniform mat4 matModel;
uniform vec3 cylCenterXYZ;   /* world position the cylinder is anchored to */

out vec3 fragWorldPos;
out vec2 fragTexCoord;

void main() {
    vec3 worldPos = vertexPosition + cylCenterXYZ;
    fragWorldPos  = worldPos;
    fragTexCoord  = vertexTexCoord;
    gl_Position   = mvp * vec4(worldPos, 1.0);
}
