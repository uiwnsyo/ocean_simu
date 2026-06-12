/**
 * @file AppMain.cpp
 * @brief AG 3D 엔진 통합 진입점.
 *
 * 실행 내용:
 *  1. PhysicsThread 시작 (오션 FFT + 리지드 바디 + 커플링)
 *  2. OrthographicCamera 기본 카메라 초기화
 *  3. Primitive 메시 등록 (Cube, Sphere, Plane)
 *  4. 렌더 루프 (stub 렌더러) — 실제 GL 창은 GLFW 연동 시 활성화
 *  5. PhysicsThread 정지
 *
 * GL 창을 열려면 USE_GLFW=1 매크로를 정의하세요.
 */

#include "PhysicsThread.h"
#include "Primitives.h"
#include "Renderer.h"
#include "OrthographicCamera.h"
#include "GpuMesh.h"
#include "OpenGLDevice.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

using namespace AG;

// ─── 종료 신호 ──────────────────────────────────────────────
static std::atomic<bool> g_running{true};
static void SigHandler(int) { g_running = false; }

int main()
{
    std::signal(SIGINT, SigHandler);

    std::cout << "================================================\n";
    std::cout << "  AG 3D Engine — 통합 실행 데모\n";
    std::cout << "  Ctrl+C 로 종료\n";
    std::cout << "================================================\n\n";

    // ============================================================
    // 1. 오션 SharedSimulationBuffer 초기화 (256×256)
    // ============================================================
    SharedSimulationBuffer::Get().Initialize(256, 256);
    std::cout << "[Init] SharedSimulationBuffer 256×256 OK\n";

    // ============================================================
    // 2. Physics Thread 시작
    //    (Ocean FFT → FluidCoupler → PhysicsWorld → Splash)
    // ============================================================
    PhysicsThread physThread;

    // 오션 파라미터 설정
    OceanParameters oceanParams;
    oceanParams.windSpeed  = 14.0f;
    oceanParams.windDirDeg = 45.0f;
    oceanParams.amplitude  = 2.0f;
    oceanParams.patchSize  = 256.0f;
    oceanParams.choppiness = 1.2f;
    physThread.ocean.SetParameters(oceanParams);

    // 리지드 바디 추가 — 물에 뜨는 박스
    auto box = std::make_shared<RigidBody>();
    box->shape     = std::make_shared<BoxShape>(float3(0.5f, 0.25f, 0.5f));
    box->mass      = 80.0f;
    box->position  = float3(0.0f, 3.0f, 0.0f); // 수면 위에서 낙하
    box->restitution = 0.1f;
    box->friction    = 0.4f;
    RigidBodyId boxId = physThread.world.AddBody(box);

    // 바닥 평면 (static)
    auto floor = std::make_shared<RigidBody>();
    floor->shape  = std::make_shared<BoxShape>(float3(50.0f, 0.5f, 50.0f));
    floor->position = float3(0.0f, -10.0f, 0.0f);
    floor->isStatic = true;
    physThread.world.AddBody(floor);

    std::cout << "[Physics] RigidBody 2개 등록 (동적 Box + 정적 Floor)\n";

    uint64_t lastStep = 0;
    physThread.Start([&](){
        uint64_t s = physThread.GetStepCount();
        if (s % 120 == 0) { // 1초마다 출력
            auto* b = physThread.world.GetBody(boxId);
            if (b) {
                std::cout << "[t=" << physThread.GetSimTime()
                          << "s] Box pos=("
                          << b->position.x << ", "
                          << b->position.y << ", "
                          << b->position.z << ")"
                          << " vel.y=" << b->linearVel.y << "\n";
            }
        }
    });
    std::cout << "[Physics] Thread 시작 (1/120s 고정 타임스텝)\n\n";

    // ============================================================
    // 3. Stub 디바이스 + 렌더러 초기화 (GL 없이도 구조 검증)
    // ============================================================
    OpenGLDevice device; // Stub — 실제 GL 없이 동작

    RenderSettings rs;
    rs.viewportWidth   = 1920;
    rs.viewportHeight  = 1080;
    rs.orthoZoom       = 12.0f;
    rs.taaEnabled      = true;
    rs.bloomEnabled    = true;
    rs.ssaoEnabled     = true;
    rs.oceanEnabled    = true;
    rs.bloomThreshold  = 1.0f;
    rs.bloomIntensity  = 0.4f;

    Renderer renderer(&device, rs);
    std::cout << "[Renderer] OrthographicCamera zoom=" << rs.orthoZoom << "\n";
    std::cout << "[Renderer] TAA=" << rs.taaEnabled
              << " Bloom=" << rs.bloomEnabled
              << " SSAO=" << rs.ssaoEnabled << "\n\n";

    // ============================================================
    // 4. Primitive Mesh 등록
    // ============================================================
    auto cube    = PrimitiveBuilder::CreateCube(1.0f);
    auto sphereUV= PrimitiveBuilder::CreateSphereUV(0.5f, 16, 32);
    auto plane   = PrimitiveBuilder::CreatePlane(20.0f, 20.0f, 16, 16, "OceanPlane");
    auto torus   = PrimitiveBuilder::CreateTorus(1.0f, 0.25f);

    std::cout << "[Mesh] 등록된 Primitive: "
              << MeshRegistry::Get().Size() << "개\n";
    std::cout << "       Cube verts="    << cube->GetVertexCount()
              << "  tri=" << cube->GetIndices(0).size()/3 << "\n";
    std::cout << "       SphereUV verts=" << sphereUV->GetVertexCount()
              << "  tri=" << sphereUV->GetIndices(0).size()/3 << "\n";
    std::cout << "       Plane(Ocean) verts=" << plane->GetVertexCount()
              << "  tri=" << plane->GetIndices(0).size()/3 << "\n\n";

    // ============================================================
    // 5. 렌더 루프 (stub: 60fps 시뮬레이션)
    // ============================================================
    std::cout << "[Loop] 렌더 루프 시작 (5초 실행 후 자동 종료)\n";
    using Clock = std::chrono::steady_clock;
    auto startTime = Clock::now();
    auto prevTime  = startTime;

    int  frameCount = 0;
    auto fpsTimer   = startTime;

    while (g_running) {
        auto now = Clock::now();
        float dt = std::chrono::duration<float>(now - prevTime).count();
        prevTime = now;
        float elapsed = std::chrono::duration<float>(now - startTime).count();

        // 5초 후 자동 종료
        if (elapsed > 5.0f) { g_running = false; break; }

        // 카메라 업데이트
        renderer.GetCamera()->Update(dt);

        // 오브젝트 제출 (LOD 자동 선택)
        auto* camPtr = renderer.GetCamera();
        MeshRegistry::Get().ForEach([&](const std::string&, Mesh& m){
            float3 center = float3(0.0f);
            float  distSq = glm::length2(center - camPtr->GetPosition());
            uint32_t lod  = m.SelectLod(distSq);
            // (실제 GL 환경에서는 GpuMesh::Draw(lod) 호출)
        });

        // 렌더 프레임
        renderer.RenderFrame(dt);
        physThread.NotifyRenderDone();

        ++frameCount;

        // 1초마다 FPS 출력
        float fpsElapsed = std::chrono::duration<float>(now - fpsTimer).count();
        if (fpsElapsed >= 1.0f) {
            std::cout << "[Frame " << frameCount
                      << "] FPS=" << (int)(frameCount / fpsElapsed)
                      << "  SimTime=" << physThread.GetSimTime() << "s\n";
            frameCount = 0;
            fpsTimer   = now;
        }

        // 60fps cap
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // ============================================================
    // 6. 정리
    // ============================================================
    std::cout << "\n[Shutdown] Physics Thread 정지 중...\n";
    physThread.Stop();

    std::cout << "[Shutdown] 총 물리 스텝: "
              << physThread.GetStepCount() << " ("
              << physThread.GetSimTime() << "s)\n";
    std::cout << "[Shutdown] 완료.\n";
    return 0;
}
