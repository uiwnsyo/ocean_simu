/**
 * @file Mesh.cpp
 * @brief Mesh 클래스 구현부.
 */

#include "Mesh.h"
#include <stdexcept>
#include <cassert>
#include <cmath>

namespace AG {

// ============================================================================
// Mesh — Lifecycle
// ============================================================================

Mesh::Mesh(std::string name)
    : m_name(std::move(name))
{
    // LOD 거리 기본값: [0,20] [20,60] [60,150] [150,∞]
    m_lods[0].maxDist = 20.0f;
    m_lods[1].maxDist = 60.0f;
    m_lods[2].maxDist = 150.0f;
    m_lods[3].maxDist = 1e9f;
}

// ============================================================================
// Mesh — Build API
// ============================================================================

void Mesh::Clear()
{
    m_positions.clear();
    m_normals.clear();
    m_uvs.clear();
    m_halfEdges.clear();
    m_faces.clear();
    for (auto& lod : m_lods) lod.indices.clear();
    m_dirty = true;
}

uint32_t Mesh::AddVertex(const float3& pos)
{
    uint32_t idx = (uint32_t)m_positions.size();
    m_positions.push_back({ pos });
    m_normals.push_back({ float3(0.0f), float4(0.0f) });
    m_uvs.push_back({ float2(0.0f) });
    m_dirty = true;
    return idx;
}

void Mesh::SetNormalTangent(uint32_t idx, const float3& normal, const float4& tangent)
{
    assert(idx < m_normals.size() && "Vertex index out of range");
    m_normals[idx].normal  = glm::normalize(normal);
    m_normals[idx].tangent = tangent;
    m_dirty = true;
}

void Mesh::SetUV(uint32_t idx, const float2& uv)
{
    assert(idx < m_uvs.size() && "Vertex index out of range");
    m_uvs[idx].uv = uv;
    m_dirty = true;
}

void Mesh::AddTriangle(uint32_t i0, uint32_t i1, uint32_t i2)
{
    // LOD 0 인덱스에 추가
    m_lods[0].indices.push_back(i0);
    m_lods[0].indices.push_back(i1);
    m_lods[0].indices.push_back(i2);

    // Half-Edge 3개 생성
    uint32_t heBase = (uint32_t)m_halfEdges.size();
    uint32_t faceIdx = (uint32_t)m_faces.size();

    m_halfEdges.push_back({ heBase + 1, INVALID_INDEX, i0, faceIdx });
    m_halfEdges.push_back({ heBase + 2, INVALID_INDEX, i1, faceIdx });
    m_halfEdges.push_back({ heBase + 0, INVALID_INDEX, i2, faceIdx });

    MeshFace face;
    face.halfEdge = heBase;
    m_faces.push_back(face);

    m_dirty = true;
}

void Mesh::BuildTwins()
{
    // 에지 (v_from, v_to) → HalfEdge 인덱스 맵
    std::unordered_map<uint64_t, uint32_t> edgeMap;
    edgeMap.reserve(m_halfEdges.size());

    auto MakeKey = [](uint32_t a, uint32_t b) -> uint64_t {
        return ((uint64_t)a << 32) | (uint64_t)b;
    };

    // 1차 패스: 등록
    for (uint32_t i = 0; i < (uint32_t)m_halfEdges.size(); ++i) {
        const HalfEdge& he   = m_halfEdges[i];
        const HalfEdge& next = m_halfEdges[he.next];
        edgeMap[MakeKey(he.vertex, next.vertex)] = i;
    }

    // 2차 패스: twin 연결
    for (uint32_t i = 0; i < (uint32_t)m_halfEdges.size(); ++i) {
        HalfEdge& he   = m_halfEdges[i];
        const HalfEdge& next = m_halfEdges[he.next];
        auto it = edgeMap.find(MakeKey(next.vertex, he.vertex));
        if (it != edgeMap.end()) {
            he.twin = it->second;
        }
    }
}

void Mesh::ComputeSmoothNormals()
{
    // 누적 초기화
    for (auto& n : m_normals) n.normal = float3(0.0f);

    // 각 삼각형의 면 법선을 누적
    const auto& idx = m_lods[0].indices;
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        uint32_t i0 = idx[i], i1 = idx[i+1], i2 = idx[i+2];
        float3 v0 = m_positions[i0].position;
        float3 v1 = m_positions[i1].position;
        float3 v2 = m_positions[i2].position;
        float3 faceNormal = glm::cross(v1 - v0, v2 - v0); // 길이 = 2×면적 (가중치)
        m_normals[i0].normal += faceNormal;
        m_normals[i1].normal += faceNormal;
        m_normals[i2].normal += faceNormal;
    }

    // 정규화
    for (auto& n : m_normals) {
        float len = glm::length(n.normal);
        if (len > 1e-6f) n.normal /= len;
    }
    m_dirty = true;
}

void Mesh::ComputeTangents()
{
    // MikkTSpace 경량 근사 (UV 기반 탄젠트)
    // 탄젠트 누적 버퍼
    std::vector<float3> tan1(m_positions.size(), float3(0.0f));
    std::vector<float3> tan2(m_positions.size(), float3(0.0f));

    const auto& idx = m_lods[0].indices;
    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        uint32_t i0 = idx[i], i1 = idx[i+1], i2 = idx[i+2];

        float3 v0 = m_positions[i0].position;
        float3 v1 = m_positions[i1].position;
        float3 v2 = m_positions[i2].position;

        float2 w0 = m_uvs[i0].uv, w1 = m_uvs[i1].uv, w2 = m_uvs[i2].uv;

        float3 e1 = v1 - v0, e2 = v2 - v0;
        float2 d1 = w1 - w0, d2 = w2 - w0;

        float r = d1.x * d2.y - d2.x * d1.y;
        if (std::abs(r) < 1e-8f) continue;
        float inv = 1.0f / r;

        float3 st = (e1 * d2.y - e2 * d1.y) * inv;
        float3 sb = (e2 * d1.x - e1 * d2.x) * inv;

        tan1[i0] += st; tan1[i1] += st; tan1[i2] += st;
        tan2[i0] += sb; tan2[i1] += sb; tan2[i2] += sb;
    }

    for (size_t i = 0; i < m_positions.size(); ++i) {
        const float3& n  = m_normals[i].normal;
        const float3& t  = tan1[i];
        // Gram-Schmidt 정규화
        float3 tangent = glm::normalize(t - n * glm::dot(n, t));
        float  sign    = (glm::dot(glm::cross(n, t), tan2[i]) < 0.0f) ? -1.0f : 1.0f;
        m_normals[i].tangent = float4(tangent, sign);
    }
    m_dirty = true;
}

// ============================================================================
// Mesh — LOD
// ============================================================================

void Mesh::SetLod(uint32_t level, std::vector<uint32_t> indices, float maxDist)
{
    assert(level < LOD_COUNT && "LOD level out of range");
    m_lods[level].indices  = std::move(indices);
    m_lods[level].maxDist  = maxDist;
    m_dirty = true;
}

void Mesh::AutoGenerateLods(std::function<void(uint32_t, std::vector<uint32_t>&)> genFn)
{
    // LOD0 는 이미 채워져 있는 것으로 가정. LOD1~3 생성.
    // 분할 수: LOD0=N, LOD1=N/2, LOD2=N/4, LOD3=N/8 (최소 3)
    constexpr uint32_t BASE_SEG = 32;
    constexpr uint32_t DIVIDERS[LOD_COUNT] = { 1, 2, 4, 8 };
    constexpr float    DISTS[LOD_COUNT]    = { 20.0f, 60.0f, 150.0f, 1e9f };

    for (uint32_t lod = 0; lod < LOD_COUNT; ++lod) {
        uint32_t segs = std::max(3u, BASE_SEG / DIVIDERS[lod]);
        m_lods[lod].indices.clear();
        genFn(segs, m_lods[lod].indices);
        m_lods[lod].maxDist = DISTS[lod];
    }
    m_dirty = true;
}

uint32_t Mesh::SelectLod(float distanceSq) const
{
    float dist = std::sqrt(distanceSq);
    for (uint32_t i = 0; i < LOD_COUNT; ++i) {
        if (dist <= m_lods[i].maxDist)
            return i;
    }
    return LOD_COUNT - 1;
}

const std::vector<uint32_t>& Mesh::GetIndices(uint32_t lod) const
{
    assert(lod < LOD_COUNT);
    // LOD 인덱스가 비어있으면 LOD0 으로 fallback
    if (!m_lods[lod].indices.empty()) return m_lods[lod].indices;
    return m_lods[0].indices;
}

// ============================================================================
// Mesh — Dirty Flag
// ============================================================================

void Mesh::MarkDirty() { m_dirty = true; }
void Mesh::ClearDirty() { m_dirty = false; }

} // namespace AG
