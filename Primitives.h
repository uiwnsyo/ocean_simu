#pragma once
/**
 * @file Primitives.h
 * @brief 파라미터 기반 기본 도형 생성기.
 *
 * 모든 생성 함수는 공유 Mesh 인스턴스를 반환하며
 * MeshRegistry에 자동 등록한다.
 *
 * 파라미터를 변경하면 동일 이름의 Mesh 가 재생성되어
 * MeshRegistry 에 덮어써진다 (런타임 Procedural 지원).
 */

#include "Mesh.h"
#include "MeshRegistry.h"
#include <string>

namespace AG {

// ============================================================================
// PrimitiveBuilder — static factory
// ============================================================================

/**
 * @brief Primitive 도형 생성기.
 *
 * 각 함수는 생성된 Mesh를 MeshRegistry 에 등록하고
 * shared_ptr<Mesh> 를 반환한다.
 *
 * 좌표계 : Y-up, Right-hand
 * 인덱스  : uint32_t (CCW 권선)
 */
class PrimitiveBuilder {
public:
    PrimitiveBuilder() = delete;

    // ------------------------------------------------------------------
    // Cube
    // ------------------------------------------------------------------

    /**
     * @brief 원점 중심의 Cube 메시를 생성한다.
     *
     * 각 면이 독립 정점을 공유하지 않아 법선이 정확하다 (24 정점).
     *
     * @param size   한 변의 길이 (기본 1.0)
     * @param name   MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreateCube(float size = 1.0f,
                                             const std::string& name = "Cube");

    // ------------------------------------------------------------------
    // Sphere (UV)
    // ------------------------------------------------------------------

    /**
     * @brief 위도/경도 분할 방식의 UV-Sphere 를 생성한다.
     *
     * @param radius       반지름 (기본 0.5)
     * @param stacks       위도 분할 수 (최소 3)
     * @param slices       경도 분할 수 (최소 3)
     * @param name         MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreateSphereUV(float radius = 0.5f,
                                                  uint32_t stacks = 16,
                                                  uint32_t slices = 32,
                                                  const std::string& name = "SphereUV");

    // ------------------------------------------------------------------
    // Sphere (Icosphere)
    // ------------------------------------------------------------------

    /**
     * @brief 정이십면체 기반 Icosphere 를 생성한다.
     *
     * @param radius       반지름 (기본 0.5)
     * @param subdivisions 세분화 반복 횟수 (0~5)
     * @param name         MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreateSphereIco(float radius = 0.5f,
                                                   uint32_t subdivisions = 3,
                                                   const std::string& name = "SphereIco");

    // ------------------------------------------------------------------
    // Cylinder
    // ------------------------------------------------------------------

    /**
     * @brief 상·하 캡 포함 Cylinder 를 생성한다.
     *
     * @param radiusTop    윗면 반지름
     * @param radiusBottom 아랫면 반지름
     * @param height       높이
     * @param slices       경도 분할 수 (최소 3)
     * @param stacks       측면 스택 수 (최소 1)
     * @param name         MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreateCylinder(float radiusTop    = 0.5f,
                                                  float radiusBottom = 0.5f,
                                                  float height       = 1.0f,
                                                  uint32_t slices    = 20,
                                                  uint32_t stacks    = 1,
                                                  const std::string& name = "Cylinder");

    // ------------------------------------------------------------------
    // Plane
    // ------------------------------------------------------------------

    /**
     * @brief XZ 평면 상의 분할 Plane 을 생성한다.
     *
     * @param width     X 축 방향 폭
     * @param depth     Z 축 방향 깊이
     * @param divisionsX X 분할 수 (최소 1)
     * @param divisionsZ Z 분할 수 (최소 1)
     * @param name      MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreatePlane(float width      = 1.0f,
                                              float depth      = 1.0f,
                                              uint32_t divisionsX = 1,
                                              uint32_t divisionsZ = 1,
                                              const std::string& name = "Plane");

    // ------------------------------------------------------------------
    // Torus
    // ------------------------------------------------------------------

    /**
     * @brief Torus (도넛) 를 생성한다.
     *
     * @param majorRadius  링 중심선 반지름
     * @param minorRadius  튜브 반지름
     * @param majorSegments 링 분할 수
     * @param minorSegments 튜브 분할 수
     * @param name         MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreateTorus(float majorRadius    = 1.0f,
                                              float minorRadius    = 0.25f,
                                              uint32_t majorSegments = 24,
                                              uint32_t minorSegments = 12,
                                              const std::string& name = "Torus");

    // ------------------------------------------------------------------
    // Capsule
    // ------------------------------------------------------------------

    /**
     * @brief 캡슐 (실린더 + 반구 두 개) 을 생성한다.
     *
     * @param radius       캡슐 반지름
     * @param height       원통 부분 높이 (전체 높이 = height + 2*radius)
     * @param slices       경도 분할 수
     * @param hemisphereStacks 반구 위도 스택 수
     * @param name         MeshRegistry 등록 이름
     * @return 생성된 Mesh shared_ptr
     */
    static std::shared_ptr<Mesh> CreateCapsule(float radius              = 0.5f,
                                                float height              = 1.0f,
                                                uint32_t slices           = 20,
                                                uint32_t hemisphereStacks = 8,
                                                const std::string& name   = "Capsule");

private:
    // ------------------------------------------------------------------
    // Icosphere helpers
    // ------------------------------------------------------------------

    /** @brief 단위 구 위의 두 점 중점을 정규화하여 반환한다. */
    static float3 IcoMidpoint(const float3& a, const float3& b) {
        return glm::normalize((a + b) * 0.5f);
    }

    /** @brief 인덱스를 이용해 Icosphere 삼각형을 세분한다. */
    static void IcoSubdivide(std::vector<float3>& verts,
                              std::vector<uint32_t>& idx);
};

} // namespace AG
