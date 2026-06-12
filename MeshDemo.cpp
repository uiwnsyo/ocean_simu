/**
 * @file MeshDemo.cpp
 * @brief 모델링 시스템 통합 데모 — Primitives + MeshRegistry + GpuMesh + LOD.
 *
 * 컴파일: g++ -std=c++17 -I. Mesh.cpp GpuMesh.cpp Primitives.cpp MeshDemo.cpp -o MeshDemo
 */

#include "Primitives.h"
#include "GpuMesh.h"
#include "MeshRegistry.h"
#include <iostream>
#include <iomanip>

using namespace AG;

// 카메라에서 물체까지의 거리 제곱을 흉내
static void PrintLodSelection(const Mesh& mesh, float distSq)
{
    uint32_t lod = mesh.SelectLod(distSq);
    size_t   tri = mesh.GetIndices(lod).size() / 3;
    std::cout << "  dist=" << std::setw(6) << std::sqrt(distSq)
              << "  → LOD" << lod
              << "  tri=" << tri << "\n";
}

int main()
{
    std::cout << "===== AG Mesh System Demo =====\n\n";

    // ----------------------------------------------------------------
    // 1. Primitive 생성 & Registry 자동 등록
    // ----------------------------------------------------------------
    auto cube     = PrimitiveBuilder::CreateCube(1.0f);
    auto sphereUV = PrimitiveBuilder::CreateSphereUV(0.5f, 16, 32);
    auto sphereIco= PrimitiveBuilder::CreateSphereIco(0.5f, 3);
    auto cylinder = PrimitiveBuilder::CreateCylinder(0.3f, 0.5f, 1.5f);
    auto plane    = PrimitiveBuilder::CreatePlane(10.0f, 10.0f, 8, 8);
    auto torus    = PrimitiveBuilder::CreateTorus(1.0f, 0.25f, 24, 12);
    auto capsule  = PrimitiveBuilder::CreateCapsule(0.4f, 1.0f);

    std::cout << "\n[Registry] 등록 수: " << MeshRegistry::Get().Size() << "\n\n";

    // ----------------------------------------------------------------
    // 2. LOD 선택 테스트
    // ----------------------------------------------------------------
    std::cout << "--- Sphere(UV) LOD 선택 ---\n";
    PrintLodSelection(*sphereUV,  5.0f  *  5.0f);
    PrintLodSelection(*sphereUV, 40.0f  * 40.0f);
    PrintLodSelection(*sphereUV, 100.0f * 100.0f);
    PrintLodSelection(*sphereUV, 300.0f * 300.0f);

    std::cout << "\n--- Torus LOD 선택 ---\n";
    PrintLodSelection(*torus,  10.0f *  10.0f);
    PrintLodSelection(*torus,  80.0f *  80.0f);
    PrintLodSelection(*torus, 200.0f * 200.0f);

    // ----------------------------------------------------------------
    // 3. GPU Mesh 업로드 (stub GL)
    // ----------------------------------------------------------------
    std::cout << "\n--- GpuMesh 업로드 ---\n";
    GpuMesh gpuCube(*cube);
    GpuMesh gpuSphere(*sphereUV);
    GpuMesh gpuPlane(*plane);

    // ----------------------------------------------------------------
    // 4. Procedural 파라미터 변경 → 재생성 → dirty flush
    // ----------------------------------------------------------------
    std::cout << "\n--- Procedural 재생성 (Plane 분할 수 변경) ---\n";
    auto planeLow = PrimitiveBuilder::CreatePlane(10.0f, 10.0f, 2, 2, "Plane");
    std::cout << "LOD0 삼각형: " << planeLow->GetIndices(0).size() / 3 << "\n";

    GpuMesh gpuPlaneLow(*planeLow);
    // dirty flag 시뮬레이션
    planeLow->MarkDirty();
    gpuPlaneLow.FlushIfDirty(); // GPU 재업로드

    // ----------------------------------------------------------------
    // 5. 공유 접근 검증 (Ocean simulation 모듈에서 접근 시나리오)
    // ----------------------------------------------------------------
    std::cout << "\n--- Registry 공유 접근 (Ocean Module 시뮬레이션) ---\n";
    auto found = MeshRegistry::Get().Find("Plane");
    if (found) {
        std::cout << "Mesh '" << found->GetName()
                  << "' 공유 접근 성공. 정점수=" << found->GetVertexCount() << "\n";
    }

    // ----------------------------------------------------------------
    // 6. Mesh 통계
    // ----------------------------------------------------------------
    std::cout << "\n--- Mesh 통계 ---\n";
    auto PrintStats = [](const std::string& n, const Mesh& m) {
        std::cout << std::left << std::setw(14) << n
                  << " verts=" << std::setw(6) << m.GetVertexCount()
                  << " tri="   << std::setw(6) << m.GetIndices(0).size() / 3
                  << " he="    << m.GetHalfEdges().size()
                  << "\n";
    };

    MeshRegistry::Get().ForEach([&](const std::string& nm, Mesh& m) {
        PrintStats(nm, m);
    });

    std::cout << "\n===== Demo Complete =====\n";
    return 0;
}
