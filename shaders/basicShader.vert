attribute vec3 aPosition;
attribute vec4 aColor;

uniform mat4 uMVP;

varying vec4 vColor;

void main() {
    gl_Position = uMVP * vec4(aPosition, 1.0);
    vColor 		= aColor;
}
