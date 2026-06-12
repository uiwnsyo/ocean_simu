#pragma once
#include "OrthographicCamera.h"
#include "PerspectiveCamera.h"
#include <algorithm>

namespace AG {

// =============================================================================
// Input snapshot passed to controllers every frame
// =============================================================================
struct CameraInput {
    // Mouse
    float2 mousePos       = {0.0f, 0.0f};
    float2 mouseDelta     = {0.0f, 0.0f}; // pixel delta this frame
    float  scrollDelta    = 0.0f;
    bool   mouseRightDown = false;
    bool   mouseMiddleDown= false;

    // Keyboard
    bool keyW = false, keyA = false, keyS = false, keyD = false;
    bool keyQ = false, keyE = false; // Up / Down for Fly

    // View preset shortcuts (numpad)
    bool key1 = false; // Front   (NUM 1)
    bool key3 = false; // Side    (NUM 3)
    bool key5 = false; // Toggle  (NUM 5 → Isometric)
    bool key7 = false; // Top     (NUM 7)

    // Projection toggle
    bool keyP = false; // Toggle Ortho ↔ Perspective

    float2 viewportSize = {1920.0f, 1080.0f};
    float  dt           = 0.016f;
};

// =============================================================================
// ICameraController — base interface
// =============================================================================
class ICameraController {
public:
    virtual ~ICameraController() = default;
    virtual void OnInput(const CameraInput& input, Camera& cam) = 0;
};

// =============================================================================
// OrbitController — right-drag to rotate, scroll to zoom
//   Works for both Ortho and Perspective cameras
// =============================================================================
class OrbitController : public ICameraController {
public:
    float sensitivity   = 0.4f;  // deg / pixel
    float zoomSensitivity = 1.0f;

    void OnInput(const CameraInput& input, Camera& cam) override {
        const float3 center = cam.GetTarget();

        // ---- Orbit (right drag) ----
        if (input.mouseRightDown && (input.mouseDelta.x != 0.0f || input.mouseDelta.y != 0.0f)) {
            float yaw   = -input.mouseDelta.x * sensitivity;
            float pitch = -input.mouseDelta.y * sensitivity;

            float3 offset = cam.GetPosition() - center;
            float  radius = glm::length(offset);

            // Yaw around world-up
            float4x4 yawMat   = glm::rotate(float4x4(1.0f), glm::radians(yaw),   float3(0,1,0));
            // Pitch around camera right
            float3 right = glm::normalize(glm::cross(offset, float3(0,1,0)));
            float4x4 pitchMat = glm::rotate(float4x4(1.0f), glm::radians(pitch), right);

            float3 newOffset = float3(pitchMat * yawMat * float4(offset, 0.0f));

            // Clamp pitch to avoid gimbal lock
            float angle = glm::degrees(glm::acos(glm::clamp(glm::dot(
                glm::normalize(newOffset), float3(0,1,0)), -1.0f, 1.0f)));
            if (angle < 175.0f && angle > 5.0f) {
                offset = glm::normalize(newOffset) * radius;
            } else {
                offset = glm::normalize(offset) * radius;
            }

            cam.SetPosition(center + offset);
        }

        // ---- Zoom (scroll) ----
        if (input.scrollDelta != 0.0f) {
            float3 offset = cam.GetPosition() - center;
            float radius  = glm::length(offset) * (1.0f - input.scrollDelta * 0.1f * zoomSensitivity);
            radius = glm::clamp(radius, 0.1f, 10000.0f);
            cam.SetPosition(center + glm::normalize(offset) * radius);
        }

        handlePresets(input, cam);
        handleProjectionToggle(input, cam);
    }

private:
    bool m_prevKeyP = false;

    void handlePresets(const CameraInput& in, Camera& cam) {
        if (in.key7) cam.ApplyPreset(ViewPreset::Top);
        if (in.key1) cam.ApplyPreset(ViewPreset::Front);
        if (in.key3) cam.ApplyPreset(ViewPreset::Side);
        if (in.key5) cam.ApplyPreset(ViewPreset::Isometric);
    }

    void handleProjectionToggle(const CameraInput& in, Camera& cam) {
        // Edge trigger on P key
        if (in.keyP && !m_prevKeyP) {
            ProjectionType next = (cam.GetProjType() == ProjectionType::Orthographic)
                                  ? ProjectionType::Perspective
                                  : ProjectionType::Orthographic;
            cam.SwitchProjection(next, 0.35f);
        }
        m_prevKeyP = in.keyP;
    }
};

// =============================================================================
// FlyController — WASD + mouse look (Perspective camera)
// =============================================================================
class FlyController : public ICameraController {
public:
    float moveSpeed     = 5.0f;   // units/sec
    float lookSensitivity = 0.15f; // deg/pixel
    float sprintMultiplier = 3.0f;

    void OnInput(const CameraInput& input, Camera& cam) override {
        // Mouse look (always active for fly cam)
        if (input.mouseRightDown) {
            m_yaw   += input.mouseDelta.x * lookSensitivity;
            m_pitch -= input.mouseDelta.y * lookSensitivity;
            m_pitch  = glm::clamp(m_pitch, -89.0f, 89.0f);
        }

        // Recalculate direction from yaw/pitch
        float3 dir;
        dir.x = glm::cos(glm::radians(m_yaw)) * glm::cos(glm::radians(m_pitch));
        dir.y = glm::sin(glm::radians(m_pitch));
        dir.z = glm::sin(glm::radians(m_yaw)) * glm::cos(glm::radians(m_pitch));
        dir   = glm::normalize(dir);

        float3 right = glm::normalize(glm::cross(dir, float3(0,1,0)));
        float3 up    = glm::cross(right, dir);
        float  speed = moveSpeed * (input.keyW && input.keyS ? 0.0f : 1.0f) // simple spam dodge
                       * input.dt;

        float3 pos = cam.GetPosition();
        if (input.keyW) pos += dir   * speed;
        if (input.keyS) pos -= dir   * speed;
        if (input.keyA) pos -= right * speed;
        if (input.keyD) pos += right * speed;
        if (input.keyQ) pos -= up    * speed;
        if (input.keyE) pos += up    * speed;

        cam.SetPosition(pos);
        cam.SetTarget(pos + dir);

        handleProjectionToggle(input, cam);
    }

    // Sync yaw/pitch from an existing camera position
    void SyncFromCamera(const Camera& cam) {
        float3 dir = glm::normalize(cam.GetTarget() - cam.GetPosition());
        m_pitch    = glm::degrees(glm::asin(dir.y));
        m_yaw      = glm::degrees(glm::atan2(dir.z, dir.x));
    }

private:
    float m_yaw   = -90.0f;
    float m_pitch =   0.0f;
    bool  m_prevKeyP = false;

    void handleProjectionToggle(const CameraInput& in, Camera& cam) {
        if (in.keyP && !m_prevKeyP) {
            ProjectionType next = (cam.GetProjType() == ProjectionType::Orthographic)
                                  ? ProjectionType::Perspective
                                  : ProjectionType::Orthographic;
            cam.SwitchProjection(next, 0.35f);
        }
        m_prevKeyP = in.keyP;
    }
};

// =============================================================================
// OrthoController — pan (middle-drag) + pixel-perfect zoom (scroll)
//   Designed explicitly for OrthographicCamera
// =============================================================================
class OrthoController : public ICameraController {
public:
    float panSensitivity = 1.0f;

    void OnInput(const CameraInput& input, Camera& cam) override {
        auto* ortho = dynamic_cast<OrthographicCamera*>(&cam);
        if (!ortho) return;

        // Pan — middle mouse drag
        if (input.mouseMiddleDown && (input.mouseDelta.x != 0.0f || input.mouseDelta.y != 0.0f)) {
            float2 worldDelta = ortho->ScreenToWorldDelta(input.mouseDelta, input.viewportSize);
            ortho->Pan(worldDelta * panSensitivity);
        }

        // Pixel-perfect zoom — scroll wheel
        if (input.scrollDelta != 0.0f) {
            ortho->ZoomBy(input.scrollDelta);
        }

        handlePresets(input, *ortho);
        handleProjectionToggle(input, *ortho);
    }

private:
    bool m_prevKeyP = false;

    void handlePresets(const CameraInput& in, Camera& cam) {
        if (in.key7) cam.ApplyPreset(ViewPreset::Top);
        if (in.key1) cam.ApplyPreset(ViewPreset::Front);
        if (in.key3) cam.ApplyPreset(ViewPreset::Side);
        if (in.key5) cam.ApplyPreset(ViewPreset::Isometric);
    }

    void handleProjectionToggle(const CameraInput& in, Camera& cam) {
        if (in.keyP && !m_prevKeyP) {
            ProjectionType next = (cam.GetProjType() == ProjectionType::Orthographic)
                                  ? ProjectionType::Perspective
                                  : ProjectionType::Orthographic;
            cam.SwitchProjection(next, 0.35f);
        }
        m_prevKeyP = in.keyP;
    }
};

} // namespace AG
