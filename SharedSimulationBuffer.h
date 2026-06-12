#pragma once
/**
 * @file SharedSimulationBuffer.h
 * @brief 오션 시뮬레이션 ↔ 렌더 스레드 간 공유 변위 데이터 버퍼.
 *
 * 시뮬레이션 스레드가 쓰고, 렌더 스레드가 읽기 전용으로 접근한다.
 * double-buffering으로 race condition 없이 매 프레임 데이터가 갱신된다.
 */

#include "AGMath.h"
#include <vector>
#include <atomic>
#include <mutex>
#include <cstdint>

namespace AG {

/** @brief 오션 격자 단일 셀 변위 데이터 */
struct OceanDisplacementSample {
    float3 displacement; ///< XYZ 변위 (m)
    float3 normal;       ///< 파면 법선 (정규화됨)
    float  foam;         ///< 거품 강도 [0, 1]
    float  _pad;         ///< std140 패딩
};

/**
 * @brief 오션 변위 공유 버퍼 (double-buffer + read-only 렌더 인터페이스).
 *
 * 사용법:
 * - Simulation Thread: `AcquireWrite()` → 샘플 배열 채움 → `CommitWrite()`
 * - Render Thread    : `AcquireRead()` → 읽기 → `ReleaseRead()`
 */
class SharedSimulationBuffer {
public:
    static SharedSimulationBuffer& Get() {
        static SharedSimulationBuffer instance;
        return instance;
    }

    SharedSimulationBuffer(const SharedSimulationBuffer&) = delete;
    SharedSimulationBuffer& operator=(const SharedSimulationBuffer&) = delete;

    /** @brief 그리드 크기를 설정하고 버퍼를 할당한다 (초기화 시 한 번만 호출). */
    void Initialize(uint32_t gridResX, uint32_t gridResZ) {
        m_resX = gridResX;
        m_resZ = gridResZ;
        size_t count = (size_t)gridResX * gridResZ;
        m_buffers[0].assign(count, {});
        m_buffers[1].assign(count, {});
        m_writeIndex.store(0);
        m_time = 0.0f;
    }

    // ---- Simulation Thread ----

    /** @brief 쓰기용 버퍼 포인터를 획득 (시뮬레이션 스레드 전용). */
    OceanDisplacementSample* AcquireWrite() {
        int w = 1 - m_writeIndex.load(std::memory_order_relaxed);
        return m_buffers[w].data();
    }

    /** @brief 쓰기를 완료하고 버퍼를 flip한다. */
    void CommitWrite(float simulationTime) {
        int w = 1 - m_writeIndex.load(std::memory_order_relaxed);
        m_time = simulationTime;
        m_writeIndex.store(w, std::memory_order_release);
    }

    // ---- Render Thread (Read-Only) ----

    /** @brief 현재 안정적인 읽기 버퍼를 잠근다. */
    const OceanDisplacementSample* AcquireRead() const {
        m_readIndex = m_writeIndex.load(std::memory_order_acquire);
        return m_buffers[m_readIndex].data();
    }

    /** @brief 읽기 완료 (현재 구현에서는 no-op, 인터페이스 대칭성). */
    void ReleaseRead() const {}

    uint32_t GetResX()   const { return m_resX; }
    uint32_t GetResZ()   const { return m_resZ; }
    float    GetTime()   const { return m_time; }

    /** @brief 격자 인덱스 → 샘플 오프셋 */
    uint32_t SampleIndex(uint32_t x, uint32_t z) const { return z * m_resX + x; }

private:
    SharedSimulationBuffer() = default;

    uint32_t                              m_resX = 0, m_resZ = 0;
    std::vector<OceanDisplacementSample>  m_buffers[2];
    std::atomic<int>                      m_writeIndex{0};
    mutable int                           m_readIndex  = 0;
    float                                 m_time       = 0.0f;
};

} // namespace AG
