attribute vec4 JOINT;
attribute vec3 NORMAL;
attribute vec3 POSITION;
attribute vec2 TEXCOORD_0;
attribute vec3 WEIGHT;

varying vec3 aPositionW;
varying vec3 aNormal;
varying vec2 aTexcoord;

void main()
{
    aPositionW = vec3(gl_ModelViewMatrix * vec4(POSITION, 1.0));       
    aNormal = normalize(gl_NormalMatrix * NORMAL);
    aTexcoord = TEXCOORD_0;
    
    vec4 jo = JOINT;
    vec3 wegith = WEIGHT;
	
	gl_Position = gl_ModelViewProjectionMatrix * vec4(POSITION, 1.0);
}
