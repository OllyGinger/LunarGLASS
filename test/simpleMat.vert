#version 330

uniform mat4 mvp;

in vec4 v;
in mat3 am3;
in mat4 arraym[3];

out float f;

//out mat4 mout[2];

void main()
{
	gl_Position = mvp * v;
	f = am3[2][1] + arraym[1][2][3];
    //mout[1] = arraym[2];
}
