#pragma once
/**
 * @file GpuMesh.h
 * @brief VAO/VBO/EBO 자동 관리 및 dirty flag 업로드 레이어.
 *
 * Mesh 의 분리 버텍스 스트림을 각각의 VBO에 업로드하고
 * LOD 별 EBO 를 관리한다. OpenGL DSA (glNamedBuffer*) API 기준.
 *
 * 바인딩 포인트:
 * - layout 0 : position  (VertexPosition)
 * - layout 1 : normal+tangent (VertexNormal)
 * - layout 2 : uv        (VertexUV)
 */

#include "Mesh.h"
#include <array>
#include <cstdint>

namespace AG {

/**
 * @brief GPU 메시 오브젝트.
 *
 * Mesh::IsDirty() 가 true 인 경우 FlushIfDirty() 호출 시
 * 모든 스트림 VBO 와 EBO(LOD 0)를 재업로드한다.
 *
 * @note 이 클래스는 GL 컨텍스트가 유효한 스레드에서만 사용해야 한다.
 */
class GpuMesh {
public:
    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /**
     * @brief VAO/VBO/EBO 를 생성하고 초기 업로드를 수행한다.
     * @param mesh 소유권은 외부에서 유지; GpuMesh는 포인터만 참조한다.
     */
    explicit GpuMesh(Mesh& mesh);

    /** @brief GPU 오브젝트를 해제한다. */
    ~GpuMesh();

    GpuMesh(const GpuMesh&)            = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    // ------------------------------------------------------------------
    // Upload
    // ------------------------------------------------------------------

    /**
     * @brief Mesh가 dirty인 경우 GPU 버퍼를 재업로드한다.
     *
     * 렌더 루프 시작 시 또는 RenderGraph pass 진입 전에 호출한다.
     * dirty 가 false 이면 즉시 반환한다 (zero cost).
     */
    void FlushIfDirty();

    /**
     * @brief 특정 LOD 의 EBO 를 강제 재업로드한다.
     * @param lod LOD 레벨 (0~LOD_COUNT-1)
     */
    void UploadLod(uint32_t lod);

    // ------------------------------------------------------------------
    // Draw
    // ------------------------------------------------------------------

    /**
     * @brief VAO 를 바인딩하고 지정 LOD 로 인덱스 드로우를 실행한다.
     * @param lod LOD 레벨 (기본값 0)
     * @param instanceCount 인스턴싱 수 (기본값 1)
     */
    void Draw(uint32_t lod = 0, uint32_t instanceCount = 1) const;

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    /** @brief OpenGL VAO 핸들 (GL identifier) */
    uint32_t GetVAO() const { return m_vao; }

    /** @brief 위치 스트림 VBO 핸들 */
    uint32_t GetVBOPosition() const { return m_vboPosition; }

    /** @brief 법선·탄젠트 스트림 VBO 핸들 */
    uint32_t GetVBONormal() const { return m_vboNormal; }

    /** @brief UV 스트림 VBO 핸들 */
    uint32_t GetVBOUV() const { return m_vboUV; }

    /** @brief 지정 LOD EBO 핸들 */
    uint32_t GetEBO(uint32_t lod = 0) const { return m_ebos[lod]; }

private:
    /** @brief 전체 스트림을 GPU에 업로드한다 (내부 사용). */
    void UploadAll();

    Mesh&    m_mesh;

    uint32_t m_vao         = 0;
    uint32_t m_vboPosition = 0;
    uint32_t m_vboNormal   = 0;
    uint32_t m_vboUV       = 0;

    std::array<uint32_t, LOD_COUNT> m_ebos = {};
    std::array<uint32_t, LOD_COUNT> m_indexCounts = {};
};

} // namespace AG
