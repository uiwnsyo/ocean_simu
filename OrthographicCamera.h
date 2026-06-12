#pragma once
#include "Camera.h"

namespace AG {

// =============================================================================
// OrthographicCamera — 기본 카메라 (Default)
//   orthoZoom = 세계 단위 height의 절반 (half-height)
//   픽셀-퍼펙트 줌: zoom은 월드 단위 height로 매핑
// =============================================================================
class OrthographicCamera : public Camera {
public:
    // orthoZoom: visible world height (unit)
    explicit OrthographicCamera(float orthoZoom = 10.0f, float aspect = 16.0f / 9.0f)
        : Camera(ProjectionType::Orthographic)
        , m_orthoZoom(orthoZoom)
    {
        m_aspect = aspect;
    }

    // ---- Orthographic-specific controls ----
    void SetOrthoZoom(float zoom) {
        m_orthoZoom = glm::max(zoom, 0.01f);
        m_dirty     = true;
    }

    void ZoomBy(float delta) {
        // Pixel-perfect zoom: scale factor
        float factor = glm::pow(1.1f, -delta);
        SetOrthoZoom(m_orthoZoom * factor);
    }

    // Pan in screen-space (world units)
    void Pan(const float2& worldDelta) {
        float3 right   = glm::normalize(glm::cross(m_target - m_position, m_worldUp));
        float3 camUp   = glm::normalize(glm::cross(right, m_target - m_position));
        float3 shift   = right * (-worldDelta.x) + camUp * worldDelta.y;
        m_position += shift;
        m_target   += shift;
        m_dirty = true;
    }

    float GetOrthoZoom() const override { return m_orthoZoom; }

    // screenSize: viewport pixel dimensions — converts pixel delta to world delta
    float2 ScreenToWorldDelta(const float2& pixelDelta, const float2& screenSize) const {
        float worldWidth  = m_orthoZoom * m_aspect;
        float worldHeight = m_orthoZoom;
        return float2(
            (pixelDelta.x / screenSize.x) * worldWidth,
            (pixelDelta.y / screenSize.y) * worldHeight
        );
    }

protected:
    float4x4 BuildProjection(ProjectionType type) override {
        if (type == ProjectionType::Orthographic) {
            float hw = m_orthoZoom * m_aspect * 0.5f;  // half-width
            float hh = m_orthoZoom * 0.5f;             // half-height
            return glm::ortho(-hw, hw, -hh, hh, m_near, m_far);
        } else {
            // Perspective fallback (used during blend)
            return glm::perspective(glm::radians(45.0f), m_aspect, m_near, m_far);
        }
    }

private:
    float m_orthoZoom = 10.0f; // visible world-unit height
};

} // namespace AG
