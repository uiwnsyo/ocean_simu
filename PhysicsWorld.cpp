/**
 * @file PhysicsWorld.cpp
 * @brief PhysicsWorld — PGS 솔버 구현.
 */

#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace AG {

// ============================================================================
// Body Management
// ============================================================================

RigidBodyId PhysicsWorld::AddBody(std::shared_ptr<RigidBody> body)
{
    body->id = m_nextId++;
    body->ComputeInertia();
    int idx = (int)m_bodies.size();
    m_bodies.push_back(body);
    m_bodyIndexMap[body->id] = idx;

    // BVH 삽입
    int leaf = m_bvh.Insert(*body);
    m_leafMap[body->id] = leaf;

    return body->id;
}

void PhysicsWorld::RemoveBody(RigidBodyId id)
{
    auto it = m_bodyIndexMap.find(id);
    if (it == m_bodyIndexMap.end()) return;
    int idx = it->second;

    // BVH 제거
    auto leafIt = m_leafMap.find(id);
    if (leafIt != m_leafMap.end()) {
        m_bvh.Remove(leafIt->second);
        m_leafMap.erase(leafIt);
    }

    // swap-erase
    if (idx != (int)m_bodies.size() - 1) {
        m_bodies[idx] = m_bodies.back();
        m_bodyIndexMap[m_bodies[idx]->id] = idx;
    }
    m_bodies.pop_back();
    m_bodyIndexMap.erase(id);
}

RigidBody* PhysicsWorld::GetBody(RigidBodyId id)
{
    auto it = m_bodyIndexMap.find(id);
    return (it != m_bodyIndexMap.end()) ? m_bodies[it->second].get() : nullptr;
}

// ============================================================================
// Step
// ============================================================================

void PhysicsWorld::Step()
{
    const float dt = fixedDt;

    // 1. 외력 적분 (중력 포함, 속도 예측)
    IntegrateForces(dt);

    // 2. BVH 업데이트
    for (auto& body : m_bodies) {
        if (body->isStatic || body->isSleeping) continue;
        auto it = m_leafMap.find(body->id);
        if (it != m_leafMap.end())
            m_bvh.UpdateLeaf(it->second, body->GetWorldAABB());
    }
    m_bvh.Refit();

    // 3. 브로드 페이즈
    std::vector<std::pair<RigidBodyId, RigidBodyId>> pairs;
    BroadPhase(pairs);

    // 4. 나로우 페이즈
    std::vector<ContactConstraint> constraints;
    NarrowPhase(pairs, constraints);

    // 5. PGS 솔버
    SolveConstraints(constraints, dt);

    // 6. 위치 확정 적분
    IntegrateVelocities(dt);

    // 7. Sleep
    UpdateSleep();

    // 8. 힘 누적 리셋
    for (auto& body : m_bodies) body->ClearForces();
}

// ============================================================================
// Integration
// ============================================================================

void PhysicsWorld::IntegrateForces(float dt)
{
    for (auto& body : m_bodies) {
        if (body->isStatic || body->isSleeping) continue;

        // 중력
        body->forceAccum += gravity * body->mass;

        // v += F/m * dt
        body->linearVel  += body->forceAccum * body->invMass * dt;

        // ω += I⁻¹ * τ * dt
        glm::mat3 wIinv   = body->WorldInertiaTensorInv();
        body->angularVel += wIinv * body->torqueAccum * dt;

        // 선형·각 감쇠 (에너지 누출 방지)
        body->linearVel  *= std::pow(0.98f, dt * 60.0f);
        body->angularVel *= std::pow(0.95f, dt * 60.0f);
    }
}

void PhysicsWorld::IntegrateVelocities(float dt)
{
    for (auto& body : m_bodies) {
        if (body->isStatic || body->isSleeping) continue;

        // 위치 적분
        body->position += body->linearVel * dt;

        // 회전 적분 (쿼터니언 미분)
        quat omega(0.0f, body->angularVel.x * dt * 0.5f,
                         body->angularVel.y * dt * 0.5f,
                         body->angularVel.z * dt * 0.5f);
        body->orientation = glm::normalize(body->orientation + omega * body->orientation);
    }
}

// ============================================================================
// Broad Phase
// ============================================================================

void PhysicsWorld::BroadPhase(std::vector<std::pair<RigidBodyId, RigidBodyId>>& pairs)
{
    m_bvh.CollectOverlapPairs(pairs);
    // 정적 vs 정적 쌍 제거
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(),
        [&](const auto& p) {
            auto* a = GetBody(p.first);
            auto* b = GetBody(p.second);
            return !a || !b || (a->isStatic && b->isStatic);
        }), pairs.end());
}

// ============================================================================
// Narrow Phase
// ============================================================================

void PhysicsWorld::NarrowPhase(
    const std::vector<std::pair<RigidBodyId, RigidBodyId>>& pairs,
    std::vector<ContactConstraint>& constraints)
{
    for (const auto& [idA, idB] : pairs) {
        auto* a = GetBody(idA);
        auto* b = GetBody(idB);
        if (!a || !b) continue;

        auto result = DetectCollision(*a, *b);
        if (result.has_value()) {
            ContactConstraint cc;
            cc.manifold = result.value();
            constraints.push_back(cc);
        }
    }
}

// ============================================================================
// PGS Solver (Projected Gauss-Seidel)
// ============================================================================

void PhysicsWorld::SolveConstraints(
    std::vector<ContactConstraint>& constraints, float dt)
{
    const float slop = 0.005f;  // 허용 침투 여유 (Baumgarte)
    const float baumgarte = 0.2f;

    for (int iter = 0; iter < solverIterations; ++iter) {
        for (auto& cc : constraints) {
            auto& mf = cc.manifold;
            auto*  a  = GetBody(mf.bodyA);
            auto*  b  = GetBody(mf.bodyB);
            if (!a || !b) continue;

            for (uint32_t i = 0; i < mf.contactCount; ++i) {
                auto& cp = mf.contacts[i];
                float3 n  = cp.normal;
                float3 rA = cp.pointA - a->position;
                float3 rB = cp.pointB - b->position;

                // 상대 속도 (접촉점)
                float3 vA = a->linearVel + glm::cross(a->angularVel, rA);
                float3 vB = b->linearVel + glm::cross(b->angularVel, rB);
                float3 vRel = vA - vB;
                float  vn   = glm::dot(vRel, n);

                // 유효 질량 계산
                float3 rxnA = glm::cross(rA, n);
                float3 rxnB = glm::cross(rB, n);
                glm::mat3 IA = a->WorldInertiaTensorInv();
                glm::mat3 IB = b->WorldInertiaTensorInv();
                float effMass = a->invMass + b->invMass
                    + glm::dot(glm::cross(IA * rxnA, rA), n)
                    + glm::dot(glm::cross(IB * rxnB, rB), n);
                if (effMass < 1e-10f) continue;

                // Baumgarte 위치 보정
                float bias = -(baumgarte / dt) * std::max(cp.penetration - slop, 0.0f);

                // 반발 계수
                float e      = std::min(a->restitution, b->restitution);
                float target = -e * std::min(vn, 0.0f);

                float dLambda = -(vn + target + bias) / effMass;

                // 클램프: λ ≥ 0 (non-penetration)
                float oldLambda  = cc.lambdaN[i];
                cc.lambdaN[i]    = std::max(0.0f, oldLambda + dLambda);
                float actualDL   = cc.lambdaN[i] - oldLambda;

                // 충격량 적용
                float3 impulse = n * actualDL;
                a->ApplyImpulse( impulse, rA);
                b->ApplyImpulse(-impulse, rB);

                // ---- 마찰 ----
                float mu = std::sqrt(a->friction * b->friction);
                float maxFriction = mu * cc.lambdaN[i];

                // 마찰 방향 1 (tangent)
                float3 t1 = vRel - n * glm::dot(vRel, n);
                if (glm::length2(t1) > 1e-8f) t1 = glm::normalize(t1);
                else t1 = float3(1,0,0);

                float3 t2   = glm::cross(n, t1);
                auto SolveFriction = [&](const float3& tang, float& accLambda) {
                    float3 rxT_A = glm::cross(rA, tang);
                    float3 rxT_B = glm::cross(rB, tang);
                    float mT = a->invMass + b->invMass
                        + glm::dot(glm::cross(IA * rxT_A, rA), tang)
                        + glm::dot(glm::cross(IB * rxT_B, rB), tang);
                    if (mT < 1e-10f) return;
                    float vt  = glm::dot(vRel, tang);
                    float dl  = -vt / mT;
                    float old = accLambda;
                    accLambda = glm::clamp(old + dl, -maxFriction, maxFriction);
                    float3 fi = tang * (accLambda - old);
                    a->ApplyImpulse( fi, rA);
                    b->ApplyImpulse(-fi, rB);
                };

                SolveFriction(t1, cc.lambdaT1[i]);
                SolveFriction(t2, cc.lambdaT2[i]);
            }
        }
    }
}

// ============================================================================
// Sleep
// ============================================================================

void PhysicsWorld::UpdateSleep()
{
    float thSq = sleepThreshold * sleepThreshold;
    for (auto& body : m_bodies) {
        if (body->isStatic) continue;
        bool slow = glm::length2(body->linearVel) < thSq
                 && glm::length2(body->angularVel) < thSq;
        body->isSleeping = slow;
    }
}

} // namespace AG
