#pragma once
/**
 * @file PhysicsThread.h
 * @brief 물리·시뮬레이션 고정 타임스텝 스레드.
 *
 * 실행 순서 (매 스텝 1/120초):
 *   [OceanSimulator::Update(t)]          → Phillips FFT → SharedSimulationBuffer 기록
 *   [FluidCoupler::ApplyBuoyancy(...)]   → 부력 힘 주입
 *   [PhysicsWorld::Step()]               → PGS 솔버
 *   [FluidCoupler::ApplySplash(...)]     → 파문 교란 추가
 *
 * 렌더 스레드는 SharedSimulationBuffer::AcquireRead() 로 안전하게 읽는다.
 */

#include "PhysicsWorld.h"
#include "OceanSimulator.h"
#include "FluidCoupler.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <functional>

namespace AG {

/**
 * @brief 물리 스레드 관리자.
 */
class PhysicsThread {
public:
    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    PhysicsWorld    world;         ///< 리지드 바디 월드
    OceanSimulator  ocean;         ///< 오션 시뮬레이터
    FluidCoupler    coupler;        ///< 커플링 컴포넌트

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /**
     * @brief 물리 스레드를 시작한다.
     * @param onStep  매 스텝 후 콜백 (선택, nullptr 가능)
     */
    void Start(std::function<void()> onStep = nullptr) {
        m_onStep   = std::move(onStep);
        m_running  = true;
        m_thread   = std::thread(&PhysicsThread::ThreadLoop, this);
    }

    /**
     * @brief 물리 스레드를 정지하고 join 한다.
     */
    void Stop() {
        m_running = false;
        m_cv.notify_all();
        if (m_thread.joinable()) m_thread.join();
    }

    /**
     * @brief 렌더 프레임이 완료됐음을 알려 다음 스텝을 허용한다.
     * (렌더 스레드에서 호출)
     */
    void NotifyRenderDone() {
        { std::lock_guard<std::mutex> lk(m_mutex); m_renderDone = true; }
        m_cv.notify_one();
    }

    /** @brief 현재 시뮬레이션 시간 */
    float GetSimTime() const { return m_simTime.load(); }

    /** @brief 현재 누적 스텝 수 */
    uint64_t GetStepCount() const { return m_stepCount.load(); }

private:
    void ThreadLoop() {
        using Clock    = std::chrono::steady_clock;
        using Duration = std::chrono::duration<double>;

        const double fixedDt = 1.0 / 120.0;
        double accumulator   = 0.0;
        auto   prevTime      = Clock::now();

        while (m_running) {
            // 현재 실제 경과 시간 측정
            auto  now     = Clock::now();
            double elapsed = Duration(now - prevTime).count();
            prevTime       = now;
            accumulator   += elapsed;

            // 최대 누적 방지 (spiral of death 방지)
            if (accumulator > 0.05) accumulator = 0.05;

            // 고정 타임스텝 소모
            while (accumulator >= fixedDt) {
                float t = m_simTime.load();

                // 1. 오션 FFT → SharedSimulationBuffer 기록
                ocean.Update(t);

                // 2. 부력 적용 (솔버 전)
                coupler.ApplyBuoyancy(world, ocean, world.gravity);

                // 3. 리지드 솔버
                world.Step();

                // 4. 파문 교란
                coupler.ApplySplash(world, ocean);

                m_simTime.store(t + (float)fixedDt);
                accumulator -= fixedDt;
                ++m_stepCount;

                if (m_onStep) m_onStep();
            }

            // 렌더 프레임 완료 알림 대기 (선택적 동기화)
            // 렌더가 너무 느리면 물리가 앞서가도록 허용
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    std::thread              m_thread;
    std::atomic<bool>        m_running{false};
    std::atomic<float>       m_simTime{0.0f};
    std::atomic<uint64_t>    m_stepCount{0};

    std::mutex               m_mutex;
    std::condition_variable  m_cv;
    bool                     m_renderDone = false;

    std::function<void()>    m_onStep;
};

} // namespace AG
