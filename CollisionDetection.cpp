/**
 * @file CollisionDetection.cpp
 * @brief SAT(Box-Box) + GJK + EPA 나로우 페이즈 구현.
 */

#include "CollisionDetection.h"
#include <limits>
#include <cmath>
#include <cassert>
#include <vector>

namespace AG {

// ============================================================================
// 내부 유틸리티
// ============================================================================

static float3 Support(const CollisionShape& shape, const float4x4& t, const float3& dir) {
    return shape.Support(dir, t);
}

// Minkowski Difference 지원 함수
static float3 SupportMD(const CollisionShape& A, const float4x4& tA,
                          const CollisionShape& B, const float4x4& tB,
                          const float3& dir) {
    return Support(A, tA, dir) - Support(B, tB, -dir);
}

// ============================================================================
// SAT — Box vs Box
// ============================================================================

namespace {
// OBB 축 (로컬 X, Y, Z) 를 월드 공간으로 추출
float3 OBBAxis(const float4x4& t, int axis) {
    return glm::normalize(float3(t[axis]));
}
// 분리축 투영 겹침 계산
bool SATAxisTest(const float3& axis,
                 const float3& relCenter,
                 float extA, float extB, float& minPen, float3& minAxis)
{
    float proj = std::abs(glm::dot(relCenter, axis));
    float pen  = extA + extB - proj;
    if (pen <= 0.0f) return false; // 분리됨
    if (pen < minPen) { minPen = pen; minAxis = axis; }
    return true;
}
} // anonymous

std::optional<ContactManifold> TestBoxBox(
    const BoxShape& a, const float4x4& tA,
    const BoxShape& b, const float4x4& tB,
    RigidBodyId idA, RigidBodyId idB)
{
    float3 posA = float3(tA[3]), posB = float3(tB[3]);
    float3 rel  = posB - posA;

    float minPen = std::numeric_limits<float>::max();
    float3 minAxis(0.0f);

    float3 axA[3] = { OBBAxis(tA,0), OBBAxis(tA,1), OBBAxis(tA,2) };
    float3 axB[3] = { OBBAxis(tB,0), OBBAxis(tB,1), OBBAxis(tB,2) };
    float3 heA    = a.halfExtents, heB = b.halfExtents;

    auto ProjectExtent = [&](const float3& axis, const float3 ax[], const float3& he) -> float {
        return std::abs(glm::dot(ax[0], axis)) * he.x
             + std::abs(glm::dot(ax[1], axis)) * he.y
             + std::abs(glm::dot(ax[2], axis)) * he.z;
    };

    // 6개 면 축
    for (int i = 0; i < 3; ++i) {
        float eA = (&heA.x)[i];
        float eB = ProjectExtent(axA[i], axB, heB);
        if (!SATAxisTest(axA[i], rel, eA, eB, minPen, minAxis)) return std::nullopt;

        eA = ProjectExtent(axB[i], axA, heA);
        eB = (&heB.x)[i];
        if (!SATAxisTest(axB[i], rel, eA, eB, minPen, minAxis)) return std::nullopt;
    }

    // 9개 에지 교차축
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float3 ax = glm::cross(axA[i], axB[j]);
            float  len = glm::length(ax);
            if (len < 1e-6f) continue;
            ax /= len;
            float eA = ProjectExtent(ax, axA, heA);
            float eB = ProjectExtent(ax, axB, heB);
            if (!SATAxisTest(ax, rel, eA, eB, minPen, minAxis)) return std::nullopt;
        }
    }

    // 법선 방향 보정 (A→B)
    if (glm::dot(minAxis, rel) < 0.0f) minAxis = -minAxis;

    // 단일 접촉점 생성 (중심 기반 근사)
    ContactManifold mf;
    mf.bodyA = idA;
    mf.bodyB = idB;
    mf.contacts[0].normal      = minAxis;
    mf.contacts[0].penetration = minPen;
    mf.contacts[0].pointA      = posA + minAxis * (glm::length(heA));
    mf.contacts[0].pointB      = mf.contacts[0].pointA + minAxis * minPen;
    mf.contactCount = 1;
    return mf;
}

// ============================================================================
// GJK
// ============================================================================

// Do-simplex: 심플렉스를 줄이고 다음 탐색 방향 d 를 반환. 원점 포함 시 true.
static bool DoSimplex(Simplex& s, float3& d)
{
    switch (s.size) {
    case 2: {
        // Line
        float3 ab = s.pts[1] - s.pts[0];
        float3 ao = -s.pts[0];
        if (glm::dot(ab, ao) > 0.0f)
            d = glm::cross(glm::cross(ab, ao), ab);
        else { s.size = 1; d = ao; }
        return false;
    }
    case 3: {
        // Triangle
        float3 ab = s.pts[1] - s.pts[0];
        float3 ac = s.pts[2] - s.pts[0];
        float3 ao = -s.pts[0];
        float3 abc = glm::cross(ab, ac);
        if (glm::dot(glm::cross(abc, ac), ao) > 0.0f) {
            s.pts[1] = s.pts[2]; s.size = 2;
            d = glm::cross(glm::cross(ac, ao), ac);
        } else if (glm::dot(glm::cross(ab, abc), ao) > 0.0f) {
            s.size = 2;
            d = glm::cross(glm::cross(ab, ao), ab);
        } else {
            d = (glm::dot(abc, ao) > 0.0f) ? abc : -abc;
        }
        return false;
    }
    case 4: {
        // Tetrahedron — 원점 포함 여부 검사
        float3 ab = s.pts[1]-s.pts[0], ac = s.pts[2]-s.pts[0], ad = s.pts[3]-s.pts[0];
        float3 ao = -s.pts[0];
        float3 abc = glm::cross(ab, ac), acd = glm::cross(ac, ad), adb = glm::cross(ad, ab);
        if (glm::dot(abc, ao) > 0.0f) { s.pts[3] = s.pts[2]; s.pts[2] = s.pts[1]; s.size=3; d=abc; }
        else if (glm::dot(acd, ao) > 0.0f) { s.pts[1] = s.pts[2]; s.pts[2] = s.pts[3]; s.size=3; d=acd; }
        else if (glm::dot(adb, ao) > 0.0f) { s.pts[2] = s.pts[1]; s.pts[1] = s.pts[3]; s.size=3; d=adb; }
        else return true; // 원점이 사면체 안
        return false;
    }
    }
    return false;
}

bool GJK_Intersects(const CollisionShape& shapeA, const float4x4& tA,
                    const CollisionShape& shapeB, const float4x4& tB,
                    Simplex& outSimplex)
{
    float3 d = float3(tB[3]) - float3(tA[3]); // 초기 방향
    if (glm::length2(d) < 1e-10f) d = float3(1,0,0);

    outSimplex.Clear();
    float3 support = SupportMD(shapeA, tA, shapeB, tB, d);
    outSimplex.Push(support);
    d = -support;

    for (int iter = 0; iter < 64; ++iter) {
        if (glm::length2(d) < 1e-10f) return true;
        support = SupportMD(shapeA, tA, shapeB, tB, d);
        if (glm::dot(support, d) < 0.0f) return false; // 원점 넘지 못함
        outSimplex.Push(support);
        if (DoSimplex(outSimplex, d)) return true;
    }
    return false;
}

// ============================================================================
// EPA
// ============================================================================

bool EPA(const Simplex& simplex,
         const CollisionShape& shapeA, const float4x4& tA,
         const CollisionShape& shapeB, const float4x4& tB,
         float3& normal, float& penetration)
{
    // 포리토프 초기화 (사면체 4정점)
    std::vector<float3> poly;
    for (uint32_t i = 0; i < simplex.size; ++i) poly.push_back(simplex.pts[i]);

    // 풀 사면체가 아닌 경우 보정
    while (poly.size() < 4) {
        float3 d = glm::normalize(float3(1,1,1) * (float)(poly.size()));
        poly.push_back(SupportMD(shapeA, tA, shapeB, tB, d));
    }

    // 간단한 EPA: 가장 가까운 삼각형 면 반복 확장
    struct EPAFace { int a, b, c; float3 normal; float dist; };

    auto MakeFace = [&](int a, int b, int c) -> EPAFace {
        float3 n = glm::normalize(glm::cross(poly[b]-poly[a], poly[c]-poly[a]));
        if (glm::dot(n, poly[a]) < 0.0f) { n=-n; std::swap(b,c); }
        return { a, b, c, n, glm::dot(n, poly[a]) };
    };

    std::vector<EPAFace> faces = {
        MakeFace(0,1,2), MakeFace(0,1,3), MakeFace(0,2,3), MakeFace(1,2,3)
    };

    for (int iter = 0; iter < 32; ++iter) {
        // 원점에 가장 가까운 면 탐색
        int   minIdx = 0;
        float minDist = faces[0].dist;
        for (int i = 1; i < (int)faces.size(); ++i) {
            if (faces[i].dist < minDist) { minDist = faces[i].dist; minIdx = i; }
        }

        float3 n    = faces[minIdx].normal;
        float3 sup  = SupportMD(shapeA, tA, shapeB, tB, n);
        float  dist = glm::dot(sup, n);

        if (dist - minDist < 1e-4f) {
            // 수렴
            normal      = n;
            penetration = minDist;
            return true;
        }

        // 새 포인트로 폴리토프 확장 (간단 버전: 보이는 면 제거 + 새 면 추가)
        int newIdx = (int)poly.size();
        poly.push_back(sup);

        std::vector<EPAFace> newFaces;
        for (auto& f : faces) {
            if (glm::dot(f.normal, sup - poly[f.a]) > 0.0f) {
                // 이 면은 sup에서 보임 → 제거, 에지로 새 면 생성
                newFaces.push_back(MakeFace(f.a, f.b, newIdx));
                newFaces.push_back(MakeFace(f.b, f.c, newIdx));
                newFaces.push_back(MakeFace(f.c, f.a, newIdx));
            } else {
                newFaces.push_back(f);
            }
        }
        faces = std::move(newFaces);
    }

    normal      = faces[0].normal;
    penetration = faces[0].dist;
    return true;
}

// ============================================================================
// DetectCollision — 통합 진입점
// ============================================================================

std::optional<ContactManifold> DetectCollision(const RigidBody& a, const RigidBody& b)
{
    if (!a.shape || !b.shape) return std::nullopt;

    float4x4 tA = a.GetTransform(), tB = b.GetTransform();

    // Box vs Box → SAT
    if (a.shape->type == ShapeType::Box && b.shape->type == ShapeType::Box) {
        return TestBoxBox(
            static_cast<const BoxShape&>(*a.shape), tA,
            static_cast<const BoxShape&>(*b.shape), tB,
            a.id, b.id);
    }

    // 일반 Convex → GJK + EPA
    Simplex simplex;
    if (!GJK_Intersects(*a.shape, tA, *b.shape, tB, simplex)) return std::nullopt;

    float3 normal; float pen;
    if (!EPA(simplex, *a.shape, tA, *b.shape, tB, normal, pen)) return std::nullopt;

    ContactManifold mf;
    mf.bodyA = a.id; mf.bodyB = b.id;
    float3 ptA = Support(*a.shape, tA, normal);
    float3 ptB = Support(*b.shape, tB, -normal);
    mf.contacts[0] = { ptA, ptB, normal, pen };
    mf.contactCount = 1;
    return mf;
}

} // namespace AG
