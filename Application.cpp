#include "Application.h"
#include "Shader.h"
#include "UIManager.h"
#include "TreeSystem.h"

// GLM
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// 標準ライブラリ
#include <iostream>

// glm::vec3 を cout で表示するためのオーバーロード
std::ostream& operator<<(std::ostream& os, const glm::vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

Application::Application(unsigned int width, unsigned int height, const char* title)
    : m_Width(width), m_Height(height), m_Window(nullptr),
    m_pShader(nullptr),
    m_pStrandShader(nullptr),
    m_pUIManager(nullptr),
    m_pTreeSystem(nullptr),
    m_ProfileRadius(0.2f),    // 初期化
    m_ParticlesPerTip(3)      // 初期化
{
    /*
    // カメラの初期位置
    m_CameraPos = glm::vec3(0.0f, 1.5f, 7.0f);
    m_CameraTarget = glm::vec3(0.0f, 1.0f, 0.0f);
    m_CameraUp = glm::vec3(0.0f, 1.0f, 0.0f);*/
    
    // --- 巨大な木全体が見える位置 ---
    m_CameraPos = glm::vec3(0.0f, 40.0f, 50.0f);   
    m_CameraTarget = glm::vec3(0.0f, 10.0f, 0.0f); 
    m_CameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
    
}

Application::~Application() {
    // cleanup()でリソースが解放される
}

void Application::init() {
    // 1. GLFWの初期化
    if (!glfwInit()) {
        std::cerr << "ERROR::GLFW::INITIALIZATION_FAILED" << std::endl;
        return;
    }

    // 計測開始
    auto start_load = std::chrono::high_resolution_clock::now();

    // OpenGL 3.3 Core Profile を使用
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 2. ウィンドウの作成
    m_Window = glfwCreateWindow(m_Width, m_Height, "Tree Application", NULL, NULL);
    if (m_Window == NULL) {
        std::cerr << "ERROR::GLFW::WINDOW_CREATION_FAILED" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(m_Window);
    glfwSwapInterval(1); // VSyncを有効化

    // 3. GLADの初期化
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "ERROR::GLAD::INITIALIZATION_FAILED" << std::endl;
        return;
    }

    // 4. OpenGLの基本設定
    glViewport(0, 0, m_Width, m_Height);
    glEnable(GL_DEPTH_TEST); // 深度テストを有効化
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f); // 背景色

    // 5. シェーダーの初期化
    m_pShader = new Shader("./Debug/shaders/simple.vert", "./Debug/shaders/simple.frag");
    m_pStrandShader = new Shader("./Debug/shaders/strand.vert", "./Debug/shaders/strand.frag");

    // 6. TreeSystemの初期化
    m_pTreeSystem = new TreeSystem();
    m_pTreeSystem->init();

    // 計測終了
    auto end_load = std::chrono::high_resolution_clock::now();
    double elapsed_load = std::chrono::duration<double, std::milli>(end_load - start_load).count();
    std::cout << "[App] Init Load Time: " << elapsed_load << " ms" << std::endl;

    // 7. UIManager (ImGui) の初期化
    m_pUIManager = new UIManager();
    m_pUIManager->init(m_Window);

    // UIの初期値を反映
    m_ProfileRadius = m_pUIManager->getProfileRadius();
    m_ParticlesPerTip = m_pUIManager->getParticlesPerTip();

    // 8. カメラ・射影行列のセットアップ
    m_View = glm::lookAt(m_CameraPos, m_CameraTarget, m_CameraUp);
    //m_Projection = glm::perspective(glm::radians(45.0f), (float)m_Width / (float)m_Height, 0.1f, 100.0f);
    m_Projection = glm::perspective(glm::radians(45.0f), (float)m_Width / (float)m_Height, 1.0f, 200.0f);
}

void Application::run() {
    init(); 

    while (!glfwWindowShouldClose(m_Window)) {
        processInput();
        update();
        render();
    }

    cleanup(); 
}

void Application::processInput() {
    // ESCキーでウィンドウを閉じる
    if (glfwGetKey(m_Window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(m_Window, true);

    // 'E' キーでエクスポート
    if (glfwGetKey(m_Window, GLFW_KEY_E) == GLFW_PRESS) {
        m_pTreeSystem->exportToOBJ("tree_model.obj");
    }

    
}

void Application::update() {
    // UIから最新の値を取得
    m_ProfileRadius = m_pUIManager->getProfileRadius();
    m_ParticlesPerTip = m_pUIManager->getParticlesPerTip();
    
    //障害物の有効無効を取得
    bool obsEnabled = m_pUIManager->getObstaclesEnabled();

    // TreeSystemの更新 (VBO更新含む)
    m_pTreeSystem->update(m_ProfileRadius, m_ParticlesPerTip, obsEnabled);
}

void Application::render() {
    // スクリーンをクリア
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ImGuiのフレームを開始
    m_pUIManager->preRender();

   bool obsEnabled = m_pUIManager->getObstaclesEnabled();
   
    // シーンの描画
    m_pShader->use();

    glm::mat4 model = glm::mat4(1.0f);
    m_pShader->setMat4("projection", m_Projection);
    m_pShader->setMat4("view", m_View);
    m_pShader->setMat4("model", model);

    // エッジ (白)
    m_pShader->setVec3("u_Color", 1.0f, 1.0f, 1.0f);
    m_pTreeSystem->drawEdges();

    // ノード 
    m_pShader->setVec3("u_Color", 0.0f, 1.0f, 0.0f);
    m_pTreeSystem->drawNodes();

    // 追加ノード 
    m_pShader->setVec3("u_Color", 0.0f, 1.0f, 1.0f);
    m_pTreeSystem->drawSubNodes();

    // プロファイル円 (白) - 全ノード分が一括で描画される
    m_pShader->setVec3("u_Color", 1.0f, 1.0f, 1.0f);
    m_pTreeSystem->drawProfile();

    /*
    //ストランド線を描画
    m_pShader->setVec3("u_Color", 0.5f, 0.5f, 0.5f); // Gray
    m_pTreeSystem->drawStrandCurves();*/
    
    /*
    //スライス境界 を描画
    m_pShader->setVec3("u_Color", 0.0f, 0.5f, 1.0f);
    m_pTreeSystem->drawDebugSlices();*/
   
    /*
    //ドロネー三角形を描画
    m_pShader->setVec3("u_Color", 0.0f, 0.5f, 1.0f); // 青
    m_pTreeSystem->drawDebugTriangles();*/

    /*
    //メッシュを描画
    m_pShader->use();
    m_pShader->setVec3("u_Color", 0.0f, 1.0f, 0.0f); // Green
    m_pTreeSystem->drawMesh();*/
    
    if (obsEnabled) {
        //障害物を描画
        m_pShader->setVec3("u_Color", 1.0f, 0.0f, 0.0f);
        m_pTreeSystem->drawObstacle();
    }

    //ストランド描画
    m_pStrandShader->use();

    m_pStrandShader->setMat4("projection", m_Projection);
    m_pStrandShader->setMat4("view", m_View);
    m_pStrandShader->setMat4("model", model);

    // ストランド粒子 (全ノード分が一括で描画される)
    m_pTreeSystem->drawStrands();

    // 5. ImGuiのUIを描画
    m_pUIManager->render();
    if (m_pUIManager->isExportRequested()) {
        m_pTreeSystem->exportToOBJ("tree_model.obj");
        m_pUIManager->resetExportRequest(); // フラグを下ろす
    }
    m_pUIManager->postRender();

    // 6. バッファを交換し、イベントをポーリング
    glfwSwapBuffers(m_Window);
    glfwPollEvents();
}

void Application::exportOBJ() {
    if (m_pTreeSystem) {
        m_pTreeSystem->exportToOBJ("tree_model.obj");
    }
}

void Application::cleanup() {
    // リソースの解放
    delete m_pTreeSystem;
    delete m_pUIManager;
    delete m_pShader;
    delete m_pStrandShader;

    // GLFWの終了
    glfwDestroyWindow(m_Window);
    glfwTerminate();
}