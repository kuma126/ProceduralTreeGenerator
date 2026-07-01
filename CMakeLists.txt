cmake_minimum_required(VERSION 3.10)
project(TestDelaunayTriangulation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------
# 依存ライブラリの管理 (vcpkg)
# --------------------------------------------------------------------
# vcpkgでインストールしたライブラリを探すようにCMakeに指示
find_package(CGAL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED) #find_packageでimguiを探す
find_package(OpenGL REQUIRED)



# --------------------------------------------------------------------
# プロジェクトのソースファイルと実行ファイルの定義
# --------------------------------------------------------------------

# 実行可能ファイルを作成
add_executable(packing_app
    main.cpp
    external/glad/src/glad.c
    Application.cpp
    Shader.cpp
    UIManager.cpp 
    TreeSystem.cpp
    ParticleSystem.cpp
    OBJExporter.cpp
    CurveGenerator.cpp
     "external/test.cpp" "Parameter.h")


# packing_app ターゲット全体で GLM_ENABLE_EXPERIMENTAL を定義する
target_compile_definitions(packing_app PUBLIC GLM_ENABLE_EXPERIMENTAL)

# 実行ファイルに必要なインクルードパスを設定
target_include_directories(packing_app
    PUBLIC 
    external/glad/include
    shaders)

# --------------------------------------------------------------------
# ライブラリのリンク
# --------------------------------------------------------------------
# 実行ファイルに必要なライブラリをすべてリンクする
target_link_libraries(packing_app 
    PUBLIC
    CGAL::CGAL
    glfw          # vcpkgが見つけたglfw3
    glm::glm      # vcpkgが見つけたglm
    imgui::imgui
    ${OPENGL_LIBRARIES}
)

add_custom_command(TARGET packing_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    $<TARGET_FILE_DIR:packing_app>/shaders
)
