uniform sampler2D gridTexture;
uniform sampler2D colorMaskTexture;
uniform sampler2D distortionMaskTexture;

uniform vec2 vanishingPoint;
uniform float depthMultiplier;
uniform float focalLength;

void main()
{
	vec4 color = texture2D( colorMaskTexture, gl_TexCoord[0].st );
	float depth = texture2D( distortionMaskTexture, gl_TexCoord[0].st ).r * depthMultiplier;
	
	float scale = focalLength / ( focalLength + depth );
	vec2 bulgedOffset = vanishingPoint + (gl_TexCoord[0].st - vanishingPoint) * scale;
	
	gl_FragColor = texture2D( gridTexture, bulgedOffset ) * color;
}