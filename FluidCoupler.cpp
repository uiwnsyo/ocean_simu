/**
 * @file FluidCoupler.cpp
 * @brief 리지드-유체 커플링 구현. 부력 + 파문(Splash).
 */

#include "FluidCoupler.h"
#include <cmath>
#include <algorithm>

namespace AG {

// ============================================================================
// ApplyBuoyancy
// ============================================================================

void FluidCoupler::ApplyBuoyancy(PhysicsWorld& world,
                                  const OceanSimulator& ocean,
                                  const float3& gravity)
{
    for (auto& body : world.GetBodies()) {
        if (body->isStatic) continue;

        float subVol = ComputeSubmergedVolume(*body, ocean);
        if (subVol <= 0.0f) continue;

        // F_buoyancy = ρ × |g| × V_submerged  (위쪽)
        float  buoyancyMag = waterDensity * std::abs(gravity.y) * subVol;
        float3 buoyancy    = float3(0.0f, buoyancyMag, 0.0f);
        body->ApplyForce(buoyancy);

        // 속도 감쇠 (유체 저항, 수직 성분)
        float3 drag = -body->linearVel * 0.5f * subVol;
        body->ApplyForce(drag);
    }
}

// ============================================================================
// ComputeSubmergedVolume
// ============================================================================

float FluidCoupler::ComputeSubmergedVolume(const RigidBody& body,
                                             const OceanSimulator& ocean) const
{
    // 바디 AABB XZ 영역을 격자 간격으로 샘플링
    AABB aabb = body.GetWorldAABB();
    float bodyBottom = aabb.min.y;
    float bodyHeight = aabb.max.y - bodyBottom;
    if (bodyHeight < 1e-6f) return 0.0f;

    const float patchSize = ocean.GetParams().patchSize;
    const float cellSize  = patchSize / (float)ocean.GetResolution();

    // 샘플링 셀 범위 계산
    float xStart = aabb.min.x, xEnd = aabb.max.x;
    float zStart = aabb.min.z, zEnd = aabb.max.z;
    int   nxCells = std::max(1, (int)((xEnd - xStart) / cellSize) + 1);
    int   nzCells = std::max(1, (int)((zEnd - zStart) / cellSize) + 1);

    float totalVolume = 0.0f;
    float cellArea    = cellSize * cellSize;

    for (int iz = 0; iz < nzCells; ++iz) {
        for (int ix = 0; ix < nxCells; ++ix) {
            float wx = xStart + (ix + 0.5f) * cellSize;
            float wz = zStart + (iz + 0.5f) * cellSize;

            float waterY = ocean.SampleHeightAt(wx, wz);
            float depth  = waterY - bodyBottom;
            if (depth <= 0.0f) continue;

            // 침수 깊이 = min(depth, bodyHeight)
            float subDepth = std::min(depth, bodyHeight);
            totalVolume   += cellArea * subDepth;
        }
    }

    return totalVolume;
}

// ============================================================================
// ApplySplash
// ============================================================================

void FluidCoupler::ApplySplash(const PhysicsWorld& world,
                                const OceanSimulator& ocean)
{
    auto& simBuf = SharedSimulationBuffer::Get();
    OceanDisplacementSample* buf = simBuf.AcquireWrite();
    if (!buf) return;

    const uint32_t n    = simBuf.GetResX();
    const float    L    = ocean.GetParams().patchSize;
    const float    cell = L / (float)n;

    for (const auto& body : world.GetBodies()) {
        if (body->isStatic) continue;

        // 수면 진입 여부 확인
        bool currSubmerged = ocean.SampleHeightAt(body->position.x, body->position.z)
                             > body->position.y;

        auto it = m_prevSubmerged.find(body->id);
        bool prevSub = (it != m_prevSubmerged.end()) ? it->second : false;

        bool entering = currSubmerged && !prevSub;
        m_prevSubmerged[body->id] = currSubmerged;

        if (!entering) continue;

        // 수면 진입 속도
        float speed = glm::length(body->linearVel);
        if (speed < 0.1f) continue;

        // 격자 중심 셀 인덱스
        int cx = (int)((body->position.x / L + 0.5f) * (float)n) % (int)n;
        int cz = (int)((body->position.z / L + 0.5f) * (float)n) % (int)n;

        // 인접 셀에 파문 교란 추가 (Y 변위 증폭)
        for (int dz = -splashRadius; dz <= splashRadius; ++dz) {
            for (int dx = -splashRadius; dx <= splashRadius; ++dx) {
                float dist = std::sqrt((float)(dx*dx + dz*dz));
                if (dist > splashRadius) continue;

                int xi = ((cx + dx) % (int)n + (int)n) % (int)n;
                int zi = ((cz + dz) % (int)n + (int)n) % (int)n;
                uint32_t idx = (uint32_t)zi * n + (uint32_t)xi;

                // 거리 기반 감쇠
                float falloff = 1.0f - dist / (float)splashRadius;
                float amp     = splashStrength * speed * falloff;

                buf[idx].displacement.y += amp;
                buf[idx].foam            = std::min(1.0f, buf[idx].foam + amp * 0.5f);
            }
        }
    }
    // CommitWrite는 OceanSimulator::WriteToSharedBuffer가 호출하므로
    // 여기서는 직접 접근만 수행하고 flip하지 않음
}

} // namespace AG
