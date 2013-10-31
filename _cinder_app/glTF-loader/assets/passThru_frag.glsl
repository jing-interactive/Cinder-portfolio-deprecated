attribute vec4 JOINT;
attribute vec3 NORMAL;
attribute vec3 POSITION;
attribute vec2 TEXCOORD_0;
attribute vec3 WEIGHT;

void main()
{
	// set the first color attachment to green
	gl_FragData[0] = vec4( 0.0, 1.0, 0.0, 1.0 );
	// set the second to blue
	gl_FragData[1] = vec4( 0.0, 0.0, 1.0, 1.0 );
}
