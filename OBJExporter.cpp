#include "OBJExporter.h"
#include <fstream>
#include <iostream>

bool OBJExporter::save(const std::string& filename,
    const std::vector<glm::vec3>& vertices,
    const std::vector<unsigned int>& indices) {

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "ERROR::OBJ_EXPORTER::Failed to open file: " << filename << std::endl;
        return false;
    }

    std::cout << "Exporting mesh to " << filename << " ..." << std::endl;

    // 1. 頂点の書き出し
    for (const auto& v : vertices) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }

    // 2. 面（三角形）の書き出し
    // OBJ形式はインデックスが1から始まるため +1 する
    for (size_t i = 0; i < indices.size(); i += 3) {
        unsigned int idx1 = indices[i] + 1;
        unsigned int idx2 = indices[i + 1] + 1;
        unsigned int idx3 = indices[i + 2] + 1;
        out << "f " << idx1 << " " << idx2 << " " << idx3 << "\n";
    }

    out.close();
    std::cout << "Export finished successfully!" << std::endl;
    return true;
}

bool OBJExporter::saveMultiple(const std::string& filename, const std::vector<MeshData>& meshes) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "ERROR::OBJ_EXPORTER::Failed to open file: " << filename << std::endl;
        return false;
    }

    std::cout << "Exporting multiple meshes to " << filename << " ..." << std::endl;

    unsigned int globalVertexOffset = 1; // OBJのインデックスは1から始まる

    for (const auto& mesh : meshes) {
        // オブジェクト名の指定
        out << "o " << mesh.name << "\n";

        // 1. 頂点の書き出し
        for (const auto& v : mesh.vertices) {
            out << "v " << v.x << " " << v.y << " " << v.z << "\n";
        }

        // 2. 面の書き出し
        for (size_t i = 0; i < mesh.indices.size(); i += 3) {
            // 現在のメッシュ内でのインデックス + グローバルオフセット
            unsigned int idx1 = mesh.indices[i] + globalVertexOffset;
            unsigned int idx2 = mesh.indices[i + 1] + globalVertexOffset;
            unsigned int idx3 = mesh.indices[i + 2] + globalVertexOffset;

            out << "f " << idx1 << " " << idx2 << " " << idx3 << "\n";
        }

        // 次のメッシュのためにオフセットを進める
        globalVertexOffset += static_cast<unsigned int>(mesh.vertices.size());
    }

    out.close();
    std::cout << "Export finished successfully!" << std::endl;
    return true;
}