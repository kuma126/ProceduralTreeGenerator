#pragma once

#include "Geometry.h"
#include <glad/glad.h>
#include <glfw/glfw3.h>
#include <glm/glm.hpp>

class Shader;
class UIManager;
class TreeSystem;

class Application {

public:

	Application(unsigned int width, unsigned int height, const char* title);	~Application();
	void run();

	void exportOBJ();

private:

	void init();
	void processInput();
	void update();
	void render();
	void cleanup();

	GLFWwindow* m_Window;
	unsigned int m_Width;
	unsigned int m_Height;

	//カメラ・ビュー設定
	glm::mat4 m_Projection;
	glm::mat4 m_View;
	glm::vec3 m_CameraPos;
	glm::vec3 m_CameraTarget;
	glm::vec3 m_CameraUp;

	//アプリケーションが所有するコンポーネント
	Shader* m_pShader; //シェーダー
	Shader* m_pStrandShader; //ストランド用シェーダー
	UIManager* m_pUIManager; //ImGui UI
	TreeSystem* m_pTreeSystem; //木のジオメトリ

	float m_ProfileRadius;
	int m_ParticlesPerTip;

};