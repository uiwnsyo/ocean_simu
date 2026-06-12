#pragma once
/**
 * @file BVH.h
 * @brief AABB BVH (Bounding Volume Hierarchy) — Broad Phase 가속 구조.
 *
 * 동적 씬용 Surface Area Heuristic (SAH) 기반 삽입 BVH.
 * 매 스텝마다 변경된 노드만 refitting한다.
 */

#include "RigidBody.h"
#include <vector>
#include <memory>
#include <functional>

namespace AG {

// ============================================================================
// BVH Node
// ============================================================================

struct BVHNode {
    AABB     aabb;
    int      left    = -1;  ///< 왼쪽 자식 인덱스 (-1 = leaf)
    int      right   = -1;
    int      parent  = -1;
    RigidBodyId bodyId = INVALID_RIGIDBODY; ///< leaf인 경우에만 유효
    bool     isLeaf() const { return left < 0; }
};

// ============================================================================
// BVH Tree
// ============================================================================

/**
 * @brief 동적 AABB BVH.
 *
 * 삽입/삭제는 O(log N), 쿼리는 O(log N + k) — k는 결과 쌍 수.
 */
class BVHTree {
public:
    BVHTree() { m_nodes.reserve(256); }

    // ------------------------------------------------------------------
    // Build / Update
    // ------------------------------------------------------------------

    /**
     * @brief 새 리지드 바디를 BVH에 삽입한다.
     * @param body  삽입할 리지드 바디 (AABB는 이미 최신이어야 한다)
     * @return      생성된 leaf 노드 인덱스
     */
    int Insert(const RigidBody& body);

    /**
     * @brief 특정 leaf 노드를 제거한다.
     */
    void Remove(int leafIndex);

    /**
     * @brief leaf AABB를 갱신하고 필요 시 리-인서트한다.
     * @param leafIndex  대상 leaf 인덱스
     * @param newAABB    새 AABB
     */
    void UpdateLeaf(int leafIndex, const AABB& newAABB);

    /**
     * @brief 루트부터 AABB를 재계산한다 (refitting).
     * 매 스텝 호출. O(N).
     */
    void Refit();

    // ------------------------------------------------------------------
    // Query
    // ------------------------------------------------------------------

    /**
     * @brief 모든 겹치는 (bodyA, bodyB) 쌍을 수집한다.
     * @param pairs  결과 쌍 벡터 (콜백 대신 직접 수집)
     */
    void CollectOverlapPairs(std::vector<std::pair<RigidBodyId, RigidBodyId>>& pairs) const;

    /**
     * @brief 특정 AABB와 겹치는 leaf들을 수집한다.
     */
    void QueryAABB(const AABB& query,
                   std::vector<RigidBodyId>& results) const;

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    int  RootIndex()  const { return m_root; }
    bool IsEmpty()    const { return m_root < 0; }
    const std::vector<BVHNode>& Nodes() const { return m_nodes; }

private:
    // SAH 최적 형제 노드 탐색
    int  FindBestSibling(const AABB& leafAABB) const;
    // 내부 노드 AABB를 부모 방향으로 갱신
    void RefitAncestors(int nodeIdx);
    // 새 내부 노드 할당
    int  AllocNode();
    // 미사용 노드 반환
    void FreeNode(int idx);

    std::vector<BVHNode> m_nodes;
    std::vector<int>     m_freeList;
    int                  m_root = -1;

    // Overlap pair 수집 재귀 함수
    void CollectPairsRecursive(int nodeA, int nodeB,
        std::vector<std::pair<RigidBodyId, RigidBodyId>>& pairs) const;
};

} // namespace AG
