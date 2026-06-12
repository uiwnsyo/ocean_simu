#pragma once
#include "Camera.h"

namespace AG {

// =============================================================================
// PerspectiveCamera — FOV 기반 원근 투영
// =============================================================================
class PerspectiveCamera : public Camera {
public:
    explicit PerspectiveCamera(float fovDeg = 60.0f, float aspect = 16.0f / 9.0f,
                               float near = 0.1f, float far = 1000.0f)
        : Camera(ProjectionType::Perspective)
        , m_fovDeg(fovDeg)
    {
        m_aspect = aspect;
        m_near   = near;
        m_far    = far;
    }

    void SetFov(float fovDeg)  { m_fovDeg = glm::clamp(fovDeg, 5.0f, 170.0f); m_dirty = true; }
    float GetFov() const       { return m_fovDeg; }

protected:
    float4x4 BuildProjection(ProjectionType type) override {
        if (type == ProjectionType::Perspective) {
            return glm::perspective(glm::radians(m_fovDeg), m_aspect, m_near, m_far);
        } else {
            // Orthographic fallback (used during blend)
            float hw = 5.0f * m_aspect;
            float hh = 5.0f;
            return glm::ortho(-hw, hw, -hh, hh, m_near, m_far);
        }
    }

    float GetOrthoZoom() const override { return 0.0f; }

private:
    float m_fovDeg = 60.0f;
};

} // namespace AG
