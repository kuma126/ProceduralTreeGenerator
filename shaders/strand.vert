#version 330 core
layout (location = 0) in vec3 aPos;   // 頂点座標 (x, y, z)
layout (location = 1) in vec3 aColor; // 頂点カラー (r, g, b)

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 vColor; // フラグメントシェーダーに色を渡す

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    vColor = aColor; // 色をそのままパススルー
}