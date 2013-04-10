#version 300 es

layout(location = 7) in vec3 c;
layout(LocatioN = 3) in vec4 p;
out vec4 pos;
out vec3 color;

layout(shared, column_major, row_major) uniform; // default is now shared and row_major

layout(std140) uniform Transform { // layout of this block is std140
    mat4 M1; // row_major
    layout(column_major) mat4 M2; // column major
    mat3 N1; // row_major
} tblock;

uniform T2 { // layout of this block is shared
    bool b;
    mat4 t2m;
};

layout(column_major) uniform T3 { // shared and column_major
    mat4 M3; // column_major
    layout(row_major) mat4 M4; // row major
    mat3 N2; // column_major
};

void main()
{
    pos = p * (tblock.M1 + tblock.M2 + M4 + M3 + t2m);
    color = c * tblock.N1;
}
