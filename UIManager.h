#pragma once
#include <glad/glad.h>
#include <glfw/glfw3.h>

class UIManager {
public:
	UIManager();
	~UIManager();

	//UIの描画処理をまとめた関数
	void init(GLFWwindow* window);
	void cleanup();

	void preRender(); //ImGui::NewFrame()などを呼び出す
	void render(); //UIの定義
	void postRender(); //実際の描画 

	//UIで設定されたプロファイル半径を取得
	float getProfileRadius() const { return m_ProfileRadius; }

	//粒子数を外部から取得
	int getParticlesPerTip() const { return m_ParticlesPerTip; }

	//障害物のONとOFF
	bool getObstaclesEnabled() const { return m_ObstaclesEnabled; }


	// ゲッター
	bool isPlaying() const { return m_IsPlaying; }
	bool isStepRequested();  // 呼ぶとフラグを下ろす
	bool isResetRequested(); // 呼ぶとフラグを下ろす


	bool isExportRequested();
	void resetExportRequest();

private:
	float m_ProfileRadius;
	int m_ParticlesPerTip;
	bool m_ExportRequested;

	bool m_IsPlaying;
	bool m_StepRequested;
	bool m_ResetRequested;

	bool m_ObstaclesEnabled = true;
};