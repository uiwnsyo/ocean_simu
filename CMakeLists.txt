cmake_minimum_required(VERSION 3.20)
project(AG_3D_Engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# 외부 의존성 (vcpkg 또는 직접 설치)
#   vcpkg install glm glad glfw3 --triplet x64-windows
# ============================================================
# 로컬 GLM 헤더 설정 (Header-only)
add_library(glm INTERFACE)
target_include_directories(glm INTERFACE ${CMAKE_SOURCE_DIR})
add_library(glm::glm ALIAS glm)

# ============================================================
# 공통 소스
# ============================================================
set(ENGINE_SOURCES
    Mesh.cpp
    GpuMesh.cpp
    Primitives.cpp
    CollisionDetection.cpp
    PhysicsWorld.cpp
    OceanSimulator.cpp
    FluidCoupler.cpp
    GraphicsCore.cpp
    Renderer.cpp
)

# ============================================================
# 실행 파일 1 : 엔진 통합 데모 (AppMain.cpp)
# ============================================================
add_executable(AG_Engine
    AppMain.cpp
    ${ENGINE_SOURCES}
)
target_link_libraries(AG_Engine PRIVATE
    glm::glm
)
target_include_directories(AG_Engine PRIVATE ${CMAKE_SOURCE_DIR})

# ============================================================
# 실행 파일 2 : 카메라 단독 데모
# ============================================================
add_executable(CameraDemo
    CameraDemo.cpp
)
target_link_libraries(CameraDemo PRIVATE glm::glm)
target_include_directories(CameraDemo PRIVATE ${CMAKE_SOURCE_DIR})

# ============================================================
# 실행 파일 3 : Mesh 단독 데모
# ============================================================
add_executable(MeshDemo
    MeshDemo.cpp
    Mesh.cpp
    GpuMesh.cpp
    Primitives.cpp
)
target_link_libraries(MeshDemo PRIVATE glm::glm)
target_include_directories(MeshDemo PRIVATE ${CMAKE_SOURCE_DIR})

# ============================================================
# Windows: 콘솔 서브시스템 유지
# ============================================================
if(WIN32)
    set_target_properties(AG_Engine CameraDemo MeshDemo
        PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

# ============================================================
# 빌드 후 셰이더 디렉토리 복사
# ============================================================
add_custom_command(TARGET AG_Engine POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/shaders"
        "$<TARGET_FILE_DIR:AG_Engine>/shaders"
    COMMENT "Copying shaders to build directory"
)
