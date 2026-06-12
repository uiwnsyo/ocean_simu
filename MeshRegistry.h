#pragma once
/**
 * @file MeshRegistry.h
 * @brief 엔진 공통 Mesh 레지스트리.
 *
 * 오션 시뮬레이션·리지드 바디·렌더러 모듈이 같은 Mesh 인스턴스를
 * 공유 접근할 수 있도록 string ID 기반 중앙 저장소를 제공한다.
 *
 * 사용 예:
 * @code
 *   auto mesh = std::make_shared<Mesh>("Box");
 *   MeshRegistry::Get().Register(mesh);
 *   auto same = MeshRegistry::Get().Find("Box"); // 어느 모듈에서든
 * @endcode
 */

#include "Mesh.h"
#include <unordered_map>
#include <mutex>
#include <stdexcept>

namespace AG {

/**
 * @brief 전역 Mesh 레지스트리 (thread-safe 싱글톤).
 */
class MeshRegistry {
public:
    // ------------------------------------------------------------------
    // Singleton access
    // ------------------------------------------------------------------

    /** @brief 유일한 인스턴스를 반환한다. */
    static MeshRegistry& Get() {
        static MeshRegistry instance;
        return instance;
    }

    MeshRegistry(const MeshRegistry&)            = delete;
    MeshRegistry& operator=(const MeshRegistry&) = delete;

    // ------------------------------------------------------------------
    // Registration
    // ------------------------------------------------------------------

    /**
     * @brief Mesh를 이름 기준으로 등록한다. 이름 중복 시 덮어쓴다.
     * @param mesh 공유 소유권을 가지는 Mesh 포인터
     */
    void Register(std::shared_ptr<Mesh> mesh) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_registry[mesh->GetName()] = std::move(mesh);
    }

    /**
     * @brief 이름으로 등록된 Mesh를 찾는다.
     * @param name Mesh 이름
     * @return 등록된 shared_ptr, 없으면 nullptr
     */
    std::shared_ptr<Mesh> Find(const std::string& name) const {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_registry.find(name);
        return (it != m_registry.end()) ? it->second : nullptr;
    }

    /**
     * @brief 이름으로 Mesh를 제거한다.
     * @param name Mesh 이름
     */
    void Remove(const std::string& name) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_registry.erase(name);
    }

    /**
     * @brief 모든 등록된 Mesh를 순회한다.
     * @param fn  void(const std::string& name, Mesh& mesh) 형태의 functor
     */
    template<typename Fn>
    void ForEach(Fn&& fn) {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [name, mesh] : m_registry) {
            fn(name, *mesh);
        }
    }

    /**
     * @brief dirty 상태인 모든 Mesh 개수를 반환한다.
     * @return dirty Mesh 수
     */
    uint32_t DirtyCount() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        uint32_t count = 0;
        for (auto& [_, mesh] : m_registry)
            if (mesh->IsDirty()) ++count;
        return count;
    }

    /** @brief 등록된 전체 Mesh 수 */
    size_t Size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_registry.size();
    }

private:
    MeshRegistry() = default;

    mutable std::mutex                                   m_mutex;
    std::unordered_map<std::string, std::shared_ptr<Mesh>> m_registry;
};

} // namespace AG
