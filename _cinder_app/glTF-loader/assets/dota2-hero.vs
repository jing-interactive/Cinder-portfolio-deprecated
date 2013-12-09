// https://github.com/num3ric/Cinder-Skinning/blob/master/resources/skinning_vert_normals.glsl
const int MAXBONES = 128;

uniform vec4 uBoneMatrices[MAXBONES];

attribute vec3 POSITION;
attribute vec3 NORMAL;
attribute vec2 TEXCOORD_0;
attribute vec4 JOINT;
attribute vec3 WEIGHT;

varying vec3 aPositionW;
varying vec3 aNormal;
varying vec2 aTexcoord;

void main()
{
    vec4 pos = vec4(POSITION, 1.0);
    pos = uBoneMatrices[int(JOINT.x)] * pos * WEIGHT.x +
        uBoneMatrices[int(JOINT.y)] * pos * WEIGHT.y +
        uBoneMatrices[int(JOINT.z)] * pos * WEIGHT.z +
        uBoneMatrices[int(JOINT.w)] * pos * (1.0 - WEIGHT.x - WEIGHT.y - WEIGHT.z);
    pos.w = 1.0;

    aPositionW = (gl_ModelViewMatrix * pos).xyz;
    aNormal = normalize(gl_NormalMatrix * NORMAL); // TODO: transform normal
    aTexcoord = TEXCOORD_0;
    
    gl_Position = gl_ModelViewProjectionMatrix * pos;
}
