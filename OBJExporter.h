#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

struct MeshData {
    std::string name;
    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;
};

class OBJExporter {
public:
    // 静的メソッドとして定義（インスタンス化せずに使えるようにする）
    static bool save(const std::string& filename,
        const std::vector<glm::vec3>& vertices,
        const std::vector<unsigned int>& indices);

    //複数メッシュ用
    static bool saveMultiple(const std::string& filename,
        const std::vector<MeshData>& meshes);
};