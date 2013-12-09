// https://github.com/num3ric/Cinder-Skinning/blob/master/resources/skinning_frag_normals.glsl
uniform sampler2D diffuse;
uniform sampler2D normalMap;

uniform vec3 ambient;
uniform vec3 emission;
uniform float shininess;
uniform vec3 specular;

varying vec3 aPositionW;
varying vec3 aNormal;
varying vec2 aTexcoord;

void main()
{
    vec4 clr = texture2D(diffuse, aTexcoord) * vec4(ambient, 1);
    gl_FragColor = clr;
}
