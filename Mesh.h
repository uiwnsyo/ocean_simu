#pragma once
/**
 * @file Mesh.h
 * @brief Half-Edge 기반 Mesh 자료구조.
 *
 * 버텍스 속성을 분리 스트림으로 관리하며 LOD 4단계 자동 생성,
 * dirty flag 방식의 GPU 업로드를 지원한다.
 * 오션 시뮬레이션 및 리지드 바디 모듈과 공유 가능하도록
 * MeshRegistry에 등록 인터페이스를 제공한다.
 *
 * 좌표계: Y-up, Right-hand
 * 인덱스: uint32_t
 */

#include "AGMath.h"
#include <vector>
#include <string>
#include <array>
#include <memory>
#include <unordered_map>

namespace AG {

// ============================================================================
// Constants
// ============================================================================
static constexpr uint32_t INVALID_INDEX  = 0xFFFFFFFFu; ///< 무효 인덱스 sentinel
static constexpr uint32_t LOD_COUNT      = 4;            ///< LOD 단계 수

// ============================================================================
// Vertex Streams  (분리 스트림 – 서로 다른 VBO에 바인딩)
// ============================================================================

/** @brief 스트림 0 – 위치 */
struct VertexPosition {
    float3 position; ///< 월드 공간 정점 위치 (Y-up, RH)
};

/** @brief 스트림 1 – 법선 + 탄젠트 */
struct VertexNormal {
    float3 normal;   ///< 단위 법선 벡터
    float4 tangent;  ///< xyz = 탄젠트, w = bitangent 부호 (+1 / -1)
};

/** @brief 스트림 2 – 텍스처 좌표 */
struct VertexUV {
    float2 uv;       ///< [0,1] UV 좌표
};

// ============================================================================
// Half-Edge Data Structure
// ============================================================================

/**
 * @brief Half-Edge 단일 레코드.
 *
 * 각 HalfEdge는 자신의 출발 정점(vertex), 속한 face,
 * 같은 face 의 다음 half-edge(next), 반대 방향 twin을 가리킨다.
 */
struct HalfEdge {
    uint32_t next   = INVALID_INDEX; ///< 같은 face 내 다음 HalfEdge 인덱스
    uint32_t twin   = INVALID_INDEX; ///< 반대 방향 HalfEdge (경계면 = INVALID)
    uint32_t vertex = INVALID_INDEX; ///< 출발 정점 인덱스 (position stream)
    uint32_t face   = INVALID_INDEX; ///< 소속 Face 인덱스
};

/**
 * @brief Mesh Face (삼각형 1개).
 */
struct MeshFace {
    uint32_t halfEdge = INVALID_INDEX; ///< 이 face 에 속한 임의의 HalfEdge 인덱스
};

// ============================================================================
// LOD Submesh
// ============================================================================

/**
 * @brief 단일 LOD 레벨의 인덱스 버퍼와 거리 임계값.
 */
struct LodLevel {
    std::vector<uint32_t> indices;    ///< 삼각형 인덱스 (3개씩)
    float                 maxDist;    ///< 이 LOD가 사용되는 최대 카메라 거리 (월드 단위)
};

// ============================================================================
// Mesh
// ============================================================================

/**
 * @brief 엔진 공통 Mesh 자료구조.
 *
 * Half-Edge 위상 구조와 분리 버텍스 스트림을 함께 유지하며,
 * LOD 자동 생성 및 GPU 업로드 dirty flag를 관리한다.
 *
 * @note 인스턴스는 MeshRegistry::Register() 를 통해 등록해야
 *       오션·리지드 바디 모듈에서 공유 접근이 가능하다.
 */
class Mesh {
public:
    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /** @brief 이름을 부여하여 Mesh를 생성한다. */
    explicit Mesh(std::string name = "Unnamed");

    /** @brief 소멸자 – 연결된 GPU 리소스는 GpuMesh 가 별도 관리한다. */
    ~Mesh() = default;

    // Non-copyable, movable
    Mesh(const Mesh&)            = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&)                 = default;
    Mesh& operator=(Mesh&&)      = default;

    // ------------------------------------------------------------------
    // Build API
    // ------------------------------------------------------------------

    /**
     * @brief 모든 스트림·위상 버퍼를 초기화한다.
     *
     * Primitive 생성기가 새 메시를 채우기 전에 호출해야 한다.
     */
    void Clear();

    /**
     * @brief 위치 정점을 추가하고 인덱스를 반환한다.
     * @param pos  정점 위치 (Y-up, RH)
     * @return 추가된 정점의 인덱스
     */
    uint32_t AddVertex(const float3& pos);

    /**
     * @brief 법선·탄젠트 스트림을 설정한다 (정점 인덱스 기준).
     * @param idx     정점 인덱스
     * @param normal  단위 법선
     * @param tangent xyz = 탄젠트, w = bitangent 부호
     */
    void SetNormalTangent(uint32_t idx, const float3& normal, const float4& tangent);

    /**
     * @brief UV 스트림을 설정한다.
     * @param idx  정점 인덱스
     * @param uv   텍스처 좌표
     */
    void SetUV(uint32_t idx, const float2& uv);

    /**
     * @brief 삼각형 Face를 추가하고 Half-Edge 위상을 구성한다.
     * @param i0 첫 번째 정점 인덱스 (CCW 순서, Y-up RH)
     * @param i1 두 번째 정점 인덱스
     * @param i2 세 번째 정점 인덱스
     */
    void AddTriangle(uint32_t i0, uint32_t i1, uint32_t i2);

    /**
     * @brief Half-Edge twin 연결을 일괄 계산한다.
     *
     * AddTriangle() 으로 모든 면을 추가한 후 한 번만 호출한다.
     * O(E) 해시 맵 방식으로 처리된다.
     */
    void BuildTwins();

    /**
     * @brief 법선이 설정되지 않은 정점에 면 법선을 자동 계산하여 평균 적용한다.
     */
    void ComputeSmoothNormals();

    /**
     * @brief MikkTSpace 규칙에 따라 탄젠트·바이탄젠트를 계산한다.
     *
     * UV 스트림이 올바르게 설정되어 있어야 한다.
     */
    void ComputeTangents();

    // ------------------------------------------------------------------
    // LOD
    // ------------------------------------------------------------------

    /**
     * @brief 분리형 LOD 인덱스 버퍼를 직접 설정한다.
     * @param level   LOD 레벨 (0 = 최고 품질)
     * @param indices 삼각형 인덱스 목록
     * @param maxDist 이 LOD 가 사용되는 최대 카메라 거리
     */
    void SetLod(uint32_t level, std::vector<uint32_t> indices, float maxDist);

    /**
     * @brief 파라미터 기반 Primitive용 자동 LOD 생성.
     *
     * LOD0 인덱스를 기준으로 분할 수를 줄여 LOD1~3 을 생성한다.
     * 각 레벨 거리: [0, 20], [20, 60], [60, 150], [150, ∞]
     *
     * @param genFn  분할 수(segments)를 받아 인덱스 버퍼를 채우는 functor.
     *               signature: void(uint32_t segments, std::vector<uint32_t>& out)
     */
    void AutoGenerateLods(std::function<void(uint32_t, std::vector<uint32_t>&)> genFn);

    /**
     * @brief 카메라 거리에 가장 적합한 LOD 레벨을 반환한다 (0~LOD_COUNT-1).
     * @param distanceSq 카메라와의 거리 제곱 (불필요한 sqrt 방지)
     */
    uint32_t SelectLod(float distanceSq) const;

    /**
     * @brief 지정 LOD 레벨의 인덱스 버퍼를 반환한다.
     */
    const std::vector<uint32_t>& GetIndices(uint32_t lod = 0) const;

    // ------------------------------------------------------------------
    // Dirty Flag / GPU 상태
    // ------------------------------------------------------------------

    /** @brief 스트림 또는 인덱스가 변경되었을 때 GPU 재업로드를 예약한다. */
    void MarkDirty();

    /** @brief GPU 업로드 후 GpuMesh 가 호출하여 dirty 를 해제한다. */
    void ClearDirty();

    /** @brief 현재 dirty 상태를 반환한다. */
    bool IsDirty() const { return m_dirty; }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    const std::string&                    GetName()       const { return m_name; }
    const std::vector<VertexPosition>&    GetPositions()  const { return m_positions; }
    const std::vector<VertexNormal>&      GetNormals()    const { return m_normals; }
    const std::vector<VertexUV>&          GetUVs()        const { return m_uvs; }
    const std::vector<HalfEdge>&          GetHalfEdges()  const { return m_halfEdges; }
    const std::vector<MeshFace>&          GetFaces()      const { return m_faces; }
    const std::array<LodLevel, LOD_COUNT>& GetLods()      const { return m_lods; }
    uint32_t                              GetVertexCount()const { return (uint32_t)m_positions.size(); }

private:
    std::string                     m_name;

    // Vertex streams
    std::vector<VertexPosition>     m_positions;
    std::vector<VertexNormal>       m_normals;
    std::vector<VertexUV>           m_uvs;

    // Half-Edge topology
    std::vector<HalfEdge>           m_halfEdges;
    std::vector<MeshFace>           m_faces;

    // LOD levels [0=highest quality .. 3=lowest]
    std::array<LodLevel, LOD_COUNT> m_lods;

    bool                            m_dirty = true;
};

} // namespace AG
