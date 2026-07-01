#pragma once

#include <glad/glad.h>
#include <string>
#include <glm/glm.hpp>

class Shader {
public:
	//シェーダープログラムのID
	unsigned int ID;

	//コンストラクタ:
	// 頂点シェーダーとフラグメントシェーダーのファイルパスからシェーダーをビルド
	Shader(const char* vertexPath, const char* fragmentPath);

	//シェーダープログラムをアクティブにする
	void use();

	void setFloat(const std::string& name, float value);
	void setBool(const std::string& name ,bool value);

	void setInt(const std::string& name, int value);
	void setMat4(const std::string& name, const glm::mat4& mat);

	void setVec3(const std::string& name, const glm::vec3& value);
	void setVec3(const std::string& name, float x, float y, float z);

private:
	//シェーダーとプログラムのコンパイル/リンク時のエラーを確認するユーティリティ関数
	void checkCompileErrors(unsigned int shader, std::string type);

};
