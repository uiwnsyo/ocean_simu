/**
 * @file Primitives.cpp
 * @brief PrimitiveBuilder 구현 — Cube, Sphere(UV/Ico), Cylinder, Plane, Torus, Capsule
 *
 * 좌표계: Y-up, Right-hand
 * 인덱스: uint32_t, CCW 권선(front face = GL_CCW)
 */

#include "Primitives.h"
#include <cmath>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace AG {

// ============================================================================
// 내부 헬퍼 - register & finalize
// ============================================================================

/** 공통 마무리: twin 연결 → 평활 법선 → 탄젠트 → LOD 등록 */
static void FinalizeAndRegister(std::shared_ptr<Mesh>& mesh)
{
    mesh->BuildTwins();
    mesh->ComputeSmoothNormals();
    mesh->ComputeTangents();
    mesh->MarkDirty();
    MeshRegistry::Get().Register(mesh);
}

// ============================================================================
// Cube
// ============================================================================

std::shared_ptr<Mesh> PrimitiveBuilder::CreateCube(float size, const std::string& name)
{
    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    const float h = size * 0.5f;

    // 6 faces × 4 vertices = 24 (각 면 독립 정점으로 법선 정확)
    // 면 순서: +Y, -Y, +Z, -Z, +X, -X
    struct FaceDef {
        float3 normal;
        float3 corners[4];    // CCW (정면에서)
        float2 uvs[4];
    };

    const FaceDef faces[6] = {
        // +Y (Top)
        { {0,1,0}, { {-h,h,-h},{h,h,-h},{h,h,h},{-h,h,h} },
          { {0,1},{1,1},{1,0},{0,0} } },
        // -Y (Bottom)
        { {0,-1,0}, { {-h,-h,h},{h,-h,h},{h,-h,-h},{-h,-h,-h} },
          { {0,1},{1,1},{1,0},{0,0} } },
        // +Z (Front)
        { {0,0,1}, { {-h,-h,h},{h,-h,h},{h,h,h},{-h,h,h} },
          { {0,0},{1,0},{1,1},{0,1} } },
        // -Z (Back)
        { {0,0,-1}, { {h,-h,-h},{-h,-h,-h},{-h,h,-h},{h,h,-h} },
          { {0,0},{1,0},{1,1},{0,1} } },
        // +X (Right)
        { {1,0,0}, { {h,-h,h},{h,-h,-h},{h,h,-h},{h,h,h} },
          { {0,0},{1,0},{1,1},{0,1} } },
        // -X (Left)
        { {-1,0,0}, { {-h,-h,-h},{-h,-h,h},{-h,h,h},{-h,h,-h} },
          { {0,0},{1,0},{1,1},{0,1} } },
    };

    for (const auto& f : faces) {
        uint32_t base = mesh->GetVertexCount();
        for (int i = 0; i < 4; ++i) {
            uint32_t vi = mesh->AddVertex(f.corners[i]);
            mesh->SetNormalTangent(vi, f.normal, float4(1,0,0,1));
            mesh->SetUV(vi, f.uvs[i]);
        }
        // 쿼드 → 2 삼각형 (CCW)
        mesh->AddTriangle(base+0, base+1, base+2);
        mesh->AddTriangle(base+0, base+2, base+3);
    }

    // LOD: Cube는 분할 없으므로 모든 LOD가 동일
    for (uint32_t lod = 1; lod < LOD_COUNT; ++lod)
        mesh->SetLod(lod, std::vector<uint32_t>(mesh->GetIndices(0)), 1e9f);

    FinalizeAndRegister(mesh);
    return mesh;
}

// ============================================================================
// Sphere UV
// ============================================================================

std::shared_ptr<Mesh> PrimitiveBuilder::CreateSphereUV(
    float radius, uint32_t stacks, uint32_t slices, const std::string& name)
{
    stacks = std::max(3u, stacks);
    slices = std::max(3u, slices);

    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    // 공통 정점 생성 functor
    auto BuildIndices = [&](uint32_t st, uint32_t sl, std::vector<uint32_t>& out) {
        out.clear();
        // 인덱스 그리드
        // 정점 레이아웃: stack 0(북극) .. stack st(남극)
        // row i: slices+1 개 정점
        for (uint32_t i = 0; i < st; ++i) {
            for (uint32_t j = 0; j < sl; ++j) {
                uint32_t r0 = i * (sl + 1) + j;
                uint32_t r1 = r0 + 1;
                uint32_t r2 = (i + 1) * (sl + 1) + j;
                uint32_t r3 = r2 + 1;

                if (i != 0)             { out.push_back(r0); out.push_back(r2); out.push_back(r1); }
                if (i != st - 1)        { out.push_back(r1); out.push_back(r2); out.push_back(r3); }
            }
        }
    };

    // LOD0 정점 생성 (stacks × slices)
    for (uint32_t i = 0; i <= stacks; ++i) {
        float phi   = (float)M_PI * i / stacks;          // 0 .. π
        for (uint32_t j = 0; j <= slices; ++j) {
            float theta = 2.0f * (float)M_PI * j / slices; // 0 .. 2π

            float x = std::sin(phi) * std::cos(theta);
            float y = std::cos(phi);
            float z = std::sin(phi) * std::sin(theta);

            uint32_t vi = mesh->AddVertex(float3(x, y, z) * radius);
            mesh->SetNormalTangent(vi, float3(x, y, z),
                float4(-std::sin(theta), 0, std::cos(theta), 1.0f));
            mesh->SetUV(vi, float2((float)j / slices, (float)i / stacks));
        }
    }

    // LOD0 인덱스 직접 채움
    std::vector<uint32_t> lod0;
    BuildIndices(stacks, slices, lod0);
    mesh->SetLod(0, std::move(lod0), 20.0f);

    // LOD1~3 자동 생성
    mesh->AutoGenerateLods([&](uint32_t segs, std::vector<uint32_t>& out) {
        // 정점은 이미 있으므로 인덱스만 감소 버전 생성
        // (실제 엔진에서는 정점도 줄이겠지만, 여기서는 인덱스 감소로 처리)
        BuildIndices(std::max(3u, stacks / 2), std::max(3u, slices / 2), out);
    });

    FinalizeAndRegister(mesh);
    return mesh;
}

// ============================================================================
// Sphere Ico
// ============================================================================

void PrimitiveBuilder::IcoSubdivide(std::vector<float3>& verts,
                                     std::vector<uint32_t>& idx)
{
    std::unordered_map<uint64_t, uint32_t> midCache;
    std::vector<uint32_t> newIdx;
    newIdx.reserve(idx.size() * 4);

    auto GetMid = [&](uint32_t a, uint32_t b) -> uint32_t {
        uint64_t key = (a < b) ? ((uint64_t)a << 32 | b) : ((uint64_t)b << 32 | a);
        auto it = midCache.find(key);
        if (it != midCache.end()) return it->second;
        float3 mid = IcoMidpoint(verts[a], verts[b]);
        uint32_t ni = (uint32_t)verts.size();
        verts.push_back(mid);
        midCache[key] = ni;
        return ni;
    };

    for (size_t i = 0; i + 2 < idx.size(); i += 3) {
        uint32_t v0 = idx[i], v1 = idx[i+1], v2 = idx[i+2];
        uint32_t m01 = GetMid(v0, v1);
        uint32_t m12 = GetMid(v1, v2);
        uint32_t m20 = GetMid(v2, v0);

        newIdx.insert(newIdx.end(), { v0, m01, m20 });
        newIdx.insert(newIdx.end(), { v1, m12, m01 });
        newIdx.insert(newIdx.end(), { v2, m20, m12 });
        newIdx.insert(newIdx.end(), { m01, m12, m20 });
    }
    idx = std::move(newIdx);
}

std::shared_ptr<Mesh> PrimitiveBuilder::CreateSphereIco(
    float radius, uint32_t subdivisions, const std::string& name)
{
    subdivisions = std::min(subdivisions, 5u);

    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    // 정이십면체 기본 12 정점 (단위 구)
    const float t = (1.0f + std::sqrt(5.0f)) * 0.5f;
    std::vector<float3> verts = {
        {-1, t,0},{1,t,0},{-1,-t,0},{1,-t,0},
        {0,-1,t},{0,1,t},{0,-1,-t},{0,1,-t},
        {t,0,-1},{t,0,1},{-t,0,-1},{-t,0,1}
    };
    for (auto& v : verts) v = glm::normalize(v);

    std::vector<uint32_t> idx = {
        0,11,5,  0,5,1,   0,1,7,   0,7,10,  0,10,11,
        1,5,9,   5,11,4,  11,10,2, 10,7,6,  7,1,8,
        3,9,4,   3,4,2,   3,2,6,   3,6,8,   3,8,9,
        4,9,5,   2,4,11,  6,2,10,  8,6,7,   9,8,1
    };

    // LOD별 세분화 (LOD3 = subdivisions, LOD0 = subdivisions+3 등)
    // 여기서는 LOD0 만 subdivisions 적용, LOD1~3 는 단계 감소
    for (int lod = 0; lod < (int)LOD_COUNT; ++lod) {
        std::vector<float3>    lverts = { verts.begin(), verts.end() };
        std::vector<uint32_t>  lidx   = { idx.begin(), idx.end() };

        uint32_t lodSubs = (uint32_t)std::max(0, (int)subdivisions - lod);
        for (uint32_t s = 0; s < lodSubs; ++s)
            IcoSubdivide(lverts, lidx);

        // 정점을 mesh 에 추가 (LOD0 만 해도 된다; 나머지 LOD는 인덱스만)
        if (lod == 0) {
            for (const auto& v : lverts) {
                float3 n  = glm::normalize(v);
                float3 p  = n * radius;
                float  u  = 0.5f + std::atan2(n.z, n.x) / (2.0f * (float)M_PI);
                float  vt = 0.5f - std::asin(glm::clamp(n.y, -1.0f, 1.0f)) / (float)M_PI;
                uint32_t vi = mesh->AddVertex(p);
                mesh->SetNormalTangent(vi, n, float4(1,0,0,1));
                mesh->SetUV(vi, float2(u, vt));
            }
            mesh->SetLod(0, std::move(lidx), 20.0f);
        } else {
            constexpr float dists[LOD_COUNT] = { 20.0f, 60.0f, 150.0f, 1e9f };
            mesh->SetLod((uint32_t)lod, std::move(lidx), dists[lod]);
        }
    }

    FinalizeAndRegister(mesh);
    return mesh;
}

// ============================================================================
// Cylinder
// ============================================================================

std::shared_ptr<Mesh> PrimitiveBuilder::CreateCylinder(
    float radiusTop, float radiusBottom, float height,
    uint32_t slices, uint32_t stacks, const std::string& name)
{
    slices = std::max(3u, slices);
    stacks = std::max(1u, stacks);

    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    const float halfH = height * 0.5f;
    const float twoPi = 2.0f * (float)M_PI;

    // ---- 측면 정점 ----
    for (uint32_t st = 0; st <= stacks; ++st) {
        float t    = (float)st / stacks;
        float y    = glm::mix(-halfH, halfH, t);
        float r    = glm::mix(radiusBottom, radiusTop, t);

        for (uint32_t sl = 0; sl <= slices; ++sl) {
            float theta = twoPi * sl / slices;
            float cx = std::cos(theta), cz = std::sin(theta);
            float3 pos = { r * cx, y, r * cz };
            float3 nor = glm::normalize(float3(cx, 0.0f, cz));
            float2 uv  = { (float)sl / slices, t };
            uint32_t vi = mesh->AddVertex(pos);
            mesh->SetNormalTangent(vi, nor, float4(-cz, 0, cx, 1.0f));
            mesh->SetUV(vi, uv);
        }
    }

    // 측면 인덱스
    for (uint32_t st = 0; st < stacks; ++st) {
        for (uint32_t sl = 0; sl < slices; ++sl) {
            uint32_t r0 = st * (slices + 1) + sl;
            uint32_t r1 = r0 + 1;
            uint32_t r2 = (st + 1) * (slices + 1) + sl;
            uint32_t r3 = r2 + 1;
            mesh->AddTriangle(r0, r2, r1);
            mesh->AddTriangle(r1, r2, r3);
        }
    }

    // ---- 상단 캡 ----
    uint32_t topCenter = mesh->AddVertex({0, halfH, 0});
    mesh->SetNormalTangent(topCenter, {0,1,0}, {1,0,0,1});
    mesh->SetUV(topCenter, {0.5f, 0.5f});

    for (uint32_t sl = 0; sl < slices; ++sl) {
        float t0 = twoPi * sl / slices;
        float t1 = twoPi * (sl + 1) / slices;
        uint32_t v0 = mesh->AddVertex({radiusTop * std::cos(t0), halfH, radiusTop * std::sin(t0)});
        uint32_t v1 = mesh->AddVertex({radiusTop * std::cos(t1), halfH, radiusTop * std::sin(t1)});
        mesh->SetNormalTangent(v0, {0,1,0}, {1,0,0,1});
        mesh->SetNormalTangent(v1, {0,1,0}, {1,0,0,1});
        mesh->SetUV(v0, {0.5f + 0.5f*std::cos(t0), 0.5f + 0.5f*std::sin(t0)});
        mesh->SetUV(v1, {0.5f + 0.5f*std::cos(t1), 0.5f + 0.5f*std::sin(t1)});
        mesh->AddTriangle(topCenter, v0, v1);
    }

    // ---- 하단 캡 ----
    uint32_t botCenter = mesh->AddVertex({0, -halfH, 0});
    mesh->SetNormalTangent(botCenter, {0,-1,0}, {1,0,0,-1});
    mesh->SetUV(botCenter, {0.5f, 0.5f});

    for (uint32_t sl = 0; sl < slices; ++sl) {
        float t0 = twoPi * sl / slices;
        float t1 = twoPi * (sl + 1) / slices;
        uint32_t v0 = mesh->AddVertex({radiusBottom * std::cos(t0), -halfH, radiusBottom * std::sin(t0)});
        uint32_t v1 = mesh->AddVertex({radiusBottom * std::cos(t1), -halfH, radiusBottom * std::sin(t1)});
        mesh->SetNormalTangent(v0, {0,-1,0}, {1,0,0,-1});
        mesh->SetNormalTangent(v1, {0,-1,0}, {1,0,0,-1});
        mesh->SetUV(v0, {0.5f + 0.5f*std::cos(t0), 0.5f - 0.5f*std::sin(t0)});
        mesh->SetUV(v1, {0.5f + 0.5f*std::cos(t1), 0.5f - 0.5f*std::sin(t1)});
        mesh->AddTriangle(botCenter, v1, v0); // 반대 권선
    }

    // LOD 동일 인덱스 복사 (단순화 버전에서는 캐핑만 공유)
    for (uint32_t lod = 1; lod < LOD_COUNT; ++lod)
        mesh->SetLod(lod, std::vector<uint32_t>(mesh->GetIndices(0)), 1e9f);

    FinalizeAndRegister(mesh);
    return mesh;
}

// ============================================================================
// Plane
// ============================================================================

std::shared_ptr<Mesh> PrimitiveBuilder::CreatePlane(
    float width, float depth, uint32_t divisionsX, uint32_t divisionsZ,
    const std::string& name)
{
    divisionsX = std::max(1u, divisionsX);
    divisionsZ = std::max(1u, divisionsZ);

    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    const float hw = width * 0.5f, hd = depth * 0.5f;

    for (uint32_t iz = 0; iz <= divisionsZ; ++iz) {
        for (uint32_t ix = 0; ix <= divisionsX; ++ix) {
            float tx = (float)ix / divisionsX;
            float tz = (float)iz / divisionsZ;
            float x  = glm::mix(-hw, hw, tx);
            float z  = glm::mix(-hd, hd, tz);
            uint32_t vi = mesh->AddVertex({x, 0.0f, z});
            mesh->SetNormalTangent(vi, {0,1,0}, {1,0,0,1});
            mesh->SetUV(vi, {tx, tz});
        }
    }

    // LOD 공통 인덱스 생성 functor
    auto BuildIndices = [&](uint32_t dx, uint32_t dz, std::vector<uint32_t>& out) {
        out.clear();
        for (uint32_t iz = 0; iz < dz; ++iz) {
            for (uint32_t ix = 0; ix < dx; ++ix) {
                uint32_t r0 = iz * (divisionsX+1) + ix;
                uint32_t r1 = r0 + 1;
                uint32_t r2 = (iz+1) * (divisionsX+1) + ix;
                uint32_t r3 = r2 + 1;
                out.push_back(r0); out.push_back(r2); out.push_back(r1);
                out.push_back(r1); out.push_back(r2); out.push_back(r3);
            }
        }
    };

    // LOD 0
    std::vector<uint32_t> lod0;
    BuildIndices(divisionsX, divisionsZ, lod0);
    mesh->SetLod(0, std::move(lod0), 20.0f);

    // LOD 1~3: 분할 수 순차 감소
    constexpr uint32_t dividers[LOD_COUNT] = {1,2,4,8};
    constexpr float    dists[LOD_COUNT]    = {20.0f,60.0f,150.0f,1e9f};
    for (uint32_t lod = 1; lod < LOD_COUNT; ++lod) {
        uint32_t dx = std::max(1u, divisionsX / dividers[lod]);
        uint32_t dz = std::max(1u, divisionsZ / dividers[lod]);
        std::vector<uint32_t> lodIdx;
        BuildIndices(dx, dz, lodIdx);
        mesh->SetLod(lod, std::move(lodIdx), dists[lod]);
    }

    FinalizeAndRegister(mesh);
    return mesh;
}

// ============================================================================
// Torus
// ============================================================================

std::shared_ptr<Mesh> PrimitiveBuilder::CreateTorus(
    float majorRadius, float minorRadius,
    uint32_t majorSegments, uint32_t minorSegments,
    const std::string& name)
{
    majorSegments = std::max(3u, majorSegments);
    minorSegments = std::max(3u, minorSegments);

    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    const float twoPi = 2.0f * (float)M_PI;

    for (uint32_t i = 0; i <= majorSegments; ++i) {
        float phi = twoPi * i / majorSegments;
        float cosPhi = std::cos(phi), sinPhi = std::sin(phi);

        // 링 중심
        float3 center = { majorRadius * cosPhi, 0.0f, majorRadius * sinPhi };

        for (uint32_t j = 0; j <= minorSegments; ++j) {
            float theta = twoPi * j / minorSegments;
            float cosThe = std::cos(theta), sinThe = std::sin(theta);

            // 튜브 위 점
            float3 pos = center + float3(
                cosThe * cosPhi * minorRadius,
                sinThe * minorRadius,
                cosThe * sinPhi * minorRadius);

            float3 nor = glm::normalize(float3(
                cosThe * cosPhi,
                sinThe,
                cosThe * sinPhi));

            float3 tan = glm::normalize(float3(-sinPhi, 0, cosPhi));

            uint32_t vi = mesh->AddVertex(pos);
            mesh->SetNormalTangent(vi, nor, float4(tan, 1.0f));
            mesh->SetUV(vi, float2((float)i / majorSegments, (float)j / minorSegments));
        }
    }

    // 인덱스
    auto BuildIndices = [&](uint32_t maj, uint32_t min, std::vector<uint32_t>& out) {
        out.clear();
        for (uint32_t i = 0; i < maj; ++i) {
            for (uint32_t j = 0; j < min; ++j) {
                uint32_t r0 = i * (minorSegments + 1) + j;
                uint32_t r1 = r0 + 1;
                uint32_t r2 = (i + 1) * (minorSegments + 1) + j;
                uint32_t r3 = r2 + 1;
                out.push_back(r0); out.push_back(r2); out.push_back(r1);
                out.push_back(r1); out.push_back(r2); out.push_back(r3);
            }
        }
    };

    std::vector<uint32_t> lod0;
    BuildIndices(majorSegments, minorSegments, lod0);
    mesh->SetLod(0, std::move(lod0), 20.0f);

    constexpr uint32_t dividers[LOD_COUNT] = {1,2,4,8};
    constexpr float    dists[LOD_COUNT]    = {20.0f,60.0f,150.0f,1e9f};
    for (uint32_t lod = 1; lod < LOD_COUNT; ++lod) {
        uint32_t mj = std::max(3u, majorSegments / dividers[lod]);
        uint32_t mn = std::max(3u, minorSegments / dividers[lod]);
        std::vector<uint32_t> lodIdx;
        BuildIndices(mj, mn, lodIdx);
        mesh->SetLod(lod, std::move(lodIdx), dists[lod]);
    }

    FinalizeAndRegister(mesh);
    return mesh;
}

// ============================================================================
// Capsule
// ============================================================================

std::shared_ptr<Mesh> PrimitiveBuilder::CreateCapsule(
    float radius, float height, uint32_t slices, uint32_t hemisphereStacks,
    const std::string& name)
{
    slices           = std::max(3u, slices);
    hemisphereStacks = std::max(2u, hemisphereStacks);

    auto mesh = std::make_shared<Mesh>(name);
    mesh->Clear();

    const float twoPi = 2.0f * (float)M_PI;
    const float halfH = height * 0.5f;

    // ---- 상단 반구 + 원통 + 하단 반구 ----
    // 전체 스택: hemisphereStacks(상) + cylinderStacks(1) + hemisphereStacks(하)
    uint32_t totalStacks = 2 * hemisphereStacks + 1;

    for (uint32_t st = 0; st <= totalStacks; ++st) {
        float tStack = (float)st / totalStacks;
        float phi, yOffset;

        if (st < hemisphereStacks) {
            // 상단 반구: phi = [π/2 .. 0]
            float t = (float)st / hemisphereStacks;
            phi     = (float)M_PI * 0.5f * (1.0f - t);
            yOffset = halfH;
        } else if (st == hemisphereStacks) {
            // 경계 (원통 상단링)
            phi     = 0.0f;
            yOffset = halfH;
        } else if (st <= hemisphereStacks + 1) {
            // 원통 하단링
            phi     = 0.0f;
            yOffset = -halfH;
        } else {
            // 하단 반구: phi = [0 .. -π/2]
            float t = (float)(st - hemisphereStacks - 1) / hemisphereStacks;
            phi     = -(float)M_PI * 0.5f * t;
            yOffset = -halfH;
        }

        float cosPhi = std::cos(phi), sinPhi = std::sin(phi);
        float y = yOffset + radius * sinPhi;
        float r = radius * cosPhi;

        for (uint32_t sl = 0; sl <= slices; ++sl) {
            float theta = twoPi * sl / slices;
            float cosTh = std::cos(theta), sinTh = std::sin(theta);
            float3 pos  = { r * cosTh, y, r * sinTh };
            float3 nor  = glm::normalize(float3(cosPhi * cosTh, sinPhi, cosPhi * sinTh));
            float2 uv   = { (float)sl / slices, tStack };
            uint32_t vi = mesh->AddVertex(pos);
            mesh->SetNormalTangent(vi, nor, float4(-sinTh, 0, cosTh, 1.0f));
            mesh->SetUV(vi, uv);
        }
    }

    // 인덱스
    for (uint32_t st = 0; st < totalStacks; ++st) {
        for (uint32_t sl = 0; sl < slices; ++sl) {
            uint32_t r0 = st * (slices+1) + sl;
            uint32_t r1 = r0 + 1;
            uint32_t r2 = (st+1) * (slices+1) + sl;
            uint32_t r3 = r2 + 1;
            mesh->AddTriangle(r0, r2, r1);
            mesh->AddTriangle(r1, r2, r3);
        }
    }

    for (uint32_t lod = 1; lod < LOD_COUNT; ++lod)
        mesh->SetLod(lod, std::vector<uint32_t>(mesh->GetIndices(0)), 1e9f);

    FinalizeAndRegister(mesh);
    return mesh;
}

} // namespace AG
