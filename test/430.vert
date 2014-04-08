#version 430 core

layout(location = 4) uniform vec4 uv4;

out gl_PerVertex {
    float gl_ClipDistance[];
};

const float cx = 4.20;
const float dx = 4.20;
in vec4 bad[10];
highp in vec4 badorder;
out invariant vec4 badorder2;
out flat vec4 badorder3;

uniform float f;

void main()
{
    gl_ClipDistance[2] = 3.7;

    if (bad[0].x == cx.x)
        badorder3 = bad[0];

    gl_ClipDistance[0] = f.x;
}

layout(binding = 3) uniform boundblock { int aoeu; } boundInst;
layout(binding = 7) uniform anonblock { int aoeu; } ;
layout(binding = 4) uniform sampler2D sampb1;
layout(binding = 5) uniform sampler2D sampb2[10];
layout(binding = 31) uniform sampler2D sampb4;
