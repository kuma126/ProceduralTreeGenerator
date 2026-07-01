#version 330 core
out vec4 FragColor;

uniform vec3 u_Color;

void main()
{
    // •`‰æ‚·‚éü‚ÌF
    FragColor = vec4(u_Color, 1.0);
}
