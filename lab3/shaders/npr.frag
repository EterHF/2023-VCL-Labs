#version 410 core

layout(location = 0) in  vec3 v_Position;
layout(location = 1) in  vec3 v_Normal;

layout(location = 0) out vec4 f_Color;

struct Light {
    vec3  Intensity;
    vec3  Direction;   // For spot and directional lights.
    vec3  Position;    // For point and spot lights.
    float CutOff;      // For spot lights.
    float OuterCutOff; // For spot lights.
};

layout(std140) uniform PassConstants {
    mat4  u_Projection;
    mat4  u_View;
    vec3  u_ViewPosition;
    vec3  u_AmbientIntensity;
    Light u_Lights[4];
    int   u_CntPointLights;
    int   u_CntSpotLights;
    int   u_CntDirectionalLights;
};

uniform vec3 u_CoolColor;
uniform vec3 u_WarmColor;

vec3 Shade (vec3 lightDir, vec3 normal) {
    // your code here:
    vec3 l=normalize(lightDir),n=normalize(normal);
    float k1=(1+dot(l,n))*0.5;
    float k2=(1-dot(l,n))*0.5;
    // if(k1<0.4&&k2>0.6){
    //     k1=0;k2=1;
    // }
    // else if(k1>0.8&&k2<0.35){
    //     k1=1;k2=0;
    // }
    // else{
    //     k1=0.5;k2=0.5;
    // }
    // int kk1=int(k1*10);
    // int kk2=int(k2*10);
    // k1=kk1*0.1;
    // k2=kk2*0.1;
    return k1*u_CoolColor+k2*u_WarmColor;
}

void main() {
    // your code here:
    float gamma = 2.2;
    vec3 total = Shade(u_Lights[0].Direction, v_Normal);
    f_Color = vec4(pow(total, vec3(1. / gamma)), 1.);
}
