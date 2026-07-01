#include "UIManager.h"

// ImGuiのヘッダ
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>

UIManager::UIManager():
    m_ProfileRadius(0.2f),
    m_ParticlesPerTip(3),//先端ストランド粒子数
    m_ExportRequested(false),
    m_IsPlaying(false),
    m_StepRequested(false),
    m_ResetRequested(false)
{}

UIManager::~UIManager() {}

void UIManager::init(GLFWwindow* window) {
    // ImGuiのコンテキストを作成
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // キーボード操作を有効化
    //io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // ドッキングを有効化

    // ImGuiのスタイルを設定
    ImGui::StyleColorsDark();

    // GLFWとOpenGLのためのバックエンドを初期化
    const char* glsl_version = "#version 330 core";
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        std::cerr << "ERROR::IMGUI::ImGui_ImplGlfw_InitForOpenGL FAILED" << std::endl;
        return;
    }
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
        std::cerr << "ERROR::IMGUI::ImGui_ImplOpenGL3_Init FAILED" << std::endl;
        return;
    }
    std::cout << "ImGui initialized successfully." << std::endl;
}

void UIManager::cleanup() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void UIManager::preRender() {
    // ImGuiの新しいフレームを開始
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void UIManager::render() {
    ImGui::Begin("Control Panel");

    /*
    if (ImGui::Button("Reset")) {
        m_ResetRequested = true;
    }
    ImGui::SameLine();


    // Play/Pause はチェックボックスまたはボタンでトグル
    if (ImGui::Checkbox("Play Simulation", &m_IsPlaying)) {
        // 
    }

    if (!m_IsPlaying) {
        ImGui::SameLine();
        if (ImGui::Button("Step >")) {
            m_StepRequested = true;
        }
    }*/
    // 障害物のON/OFFチェックボックス
    ImGui::Checkbox("Enable Obstacles", &m_ObstaclesEnabled);

    if (ImGui::Button("Export to OBJ")) {
        m_ExportRequested = true; // フラグを立てる
    }

    //プロファイルの半径を変更するスライダー
    ImGui::SliderFloat("Profile B Radius", &m_ProfileRadius, 0.05f, 3.0f);
    //粒子数を変更するスライダー
    ImGui::SliderInt("Particles Per Tip", &m_ParticlesPerTip, 3, 100);

    ImGui::End();

    // ImGuiのデモウィンドウ（参考用）
    // ImGui::ShowDemoWindow();
}

void UIManager::postRender() {
    // ImGuiの描画データを生成し、OpenGLで描画
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool UIManager::isStepRequested() {
    bool ret = m_StepRequested;
    m_StepRequested = false;
    return ret;
}

bool UIManager::isResetRequested() {
    bool ret = m_ResetRequested;
    m_ResetRequested = false;
    return ret;
}

bool UIManager::isExportRequested() {
    return m_ExportRequested;
}

void UIManager::resetExportRequest() {
    m_ExportRequested = false;
}
