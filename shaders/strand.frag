#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main()
{
    // 点の中心からの距離を計算 (0.0 ～ 1.0)
    vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    
    // 半径1.0 (円の外側) なら描画を捨てる
    if (dot(circCoord, circCoord) > 1.0) {
        discard;
    }
    // 受け取った頂点の色で描画
    FragColor = vec4(vColor, 1.0);
}