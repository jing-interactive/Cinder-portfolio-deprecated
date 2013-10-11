uniform sampler2D texture;
uniform float spread;
uniform float size;
uniform vec2 axis;

void main()
{
	vec4 total;
	for( int i = -10; i < 10; i++) {
		float blur = float(i) * size;
		vec2 offset = vec2( blur, blur ) * axis;
		total += texture2D( texture, gl_TexCoord[0].st + offset ) * spread;
	}
	gl_FragColor = total ;
}