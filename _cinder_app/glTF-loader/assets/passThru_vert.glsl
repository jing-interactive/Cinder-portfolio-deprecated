attribute vec4 JOINT;
attribute vec3 NORMAL;
attribute vec3 POSITION;
attribute vec2 TEXCOORD_0;
attribute vec3 WEIGHT;

varying vec2 TEXCOORD;

void main()
{
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = ftransform();
}
