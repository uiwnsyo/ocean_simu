#pragma once
#include "AGMath.h"
#include "GraphicsHAL.h"
#include <memory>
#include <functional>

namespace AG {

// =============================================================================
// CameraUBO - std140 layout, binding = 0
// =============================================================================
struct alignas(16) CameraUBO {
    float4x4 view;          // offset  0
    float4x4 proj;          // offset 64
    float4x4 viewProj;      // offset 128
    float4   position;      // offset 192   (w = 1)
    float4   zParams;       // offset 208   (x=Near, y=Far, z=orthoZoom, w=isOrtho)
};

// =============================================================================
// Frustum - 6 plane frustum for culling
// =============================================================================
struct Frustum {
    // Planes ordered: Left, Right, Bottom, Top, Near, Far
    // Stored as float4(normal.xyz, d) where normal is inward
    float4 planes[6];

    // Extract from VP matrix (Gribb-Hartmann method)
    void ExtractFromVP(const float4x4& vp) {
        // Left: col3 + col0
        planes[0] = float4(vp[0][3]+vp[0][0], vp[1][3]+vp[1][0], vp[2][3]+vp[2][0], vp[3][3]+vp[3][0]);
        // Right: col3 - col0
        planes[1] = float4(vp[0][3]-vp[0][0], vp[1][3]-vp[1][0], vp[2][3]-vp[2][0], vp[3][3]-vp[3][0]);
        // Bottom: col3 + col1
        planes[2] = float4(vp[0][3]+vp[0][1], vp[1][3]+vp[1][1], vp[2][3]+vp[2][1], vp[3][3]+vp[3][1]);
        // Top: col3 - col1
        planes[3] = float4(vp[0][3]-vp[0][1], vp[1][3]-vp[1][1], vp[2][3]-vp[2][1], vp[3][3]-vp[3][1]);
        // Near: col3 + col2  (Vulkan NDC: 0..1 depth → use col2 directly if GLM_FORCE_DEPTH_ZERO_TO_ONE)
        planes[4] = float4(vp[0][2], vp[1][2], vp[2][2], vp[3][2]);
        // Far: col3 - col2
        planes[5] = float4(vp[0][3]-vp[0][2], vp[1][3]-vp[1][2], vp[2][3]-vp[2][2], vp[3][3]-vp[3][2]);

        // Normalize planes
        for (auto& p : planes) {
            float len = glm::length(float3(p));
            if (len > 1e-6f) p /= len;
        }
    }

    // AABB vs Frustum test — true if visible
    bool TestAABB(const float3& center, const float3& halfExtents) const {
        for (const auto& plane : planes) {
            float d = glm::dot(float3(plane), center) + plane.w;
            float r = glm::dot(halfExtents, glm::abs(float3(plane)));
            if (d + r < 0.0f) return false; // Fully outside
        }
        return true;
    }
};

// =============================================================================
// ProjectionType & ViewPreset
// =============================================================================
enum class ProjectionType { Orthographic, Perspective };

enum class ViewPreset {
    Custom,
    Top,        // Key 7
    Front,      // Key 1
    Side,       // Key 3
    Isometric   // Key 5
};

// =============================================================================
// Camera — Abstract base
// =============================================================================
class Camera {
public:
    explicit Camera(ProjectionType type = ProjectionType::Orthographic)
        : m_projType(type) {}
    virtual ~Camera() = default;

    // ---- Setters ----
    void SetPosition(const float3& pos)   { m_position = pos;  m_dirty = true; }
    void SetTarget(const float3& target)  { m_target   = target; m_dirty = true; }
    void SetUp(const float3& up)          { m_worldUp  = up;   m_dirty = true; }
    void SetAspect(float aspect)          { m_aspect   = aspect; m_dirty = true; }
    void SetNearFar(float n, float f)     { m_near = n; m_far = f; m_dirty = true; }

    // ---- Getters ----
    const float3&      GetPosition()  const { return m_position; }
    const float3&      GetTarget()    const { return m_target; }
    const float4x4&    GetView()      const { return m_view; }
    const float4x4&    GetProj()      const { return m_proj; }
    const float4x4&    GetViewProj()  const { return m_viewProj; }
    const Frustum&     GetFrustum()   const { return m_frustum; }
    ProjectionType     GetProjType()  const { return m_projType; }

    // ---- Apply View Preset ----
    void ApplyPreset(ViewPreset preset) {
        m_preset = preset;
        switch (preset) {
        case ViewPreset::Top:
            m_position = m_target + float3(0.0f, 10.0f, 0.0f);
            m_worldUp  = float3(0.0f, 0.0f, -1.0f);
            break;
        case ViewPreset::Front:
            m_position = m_target + float3(0.0f, 0.0f, 10.0f);
            m_worldUp  = float3(0.0f, 1.0f, 0.0f);
            break;
        case ViewPreset::Side:
            m_position = m_target + float3(10.0f, 0.0f, 0.0f);
            m_worldUp  = float3(0.0f, 1.0f, 0.0f);
            break;
        case ViewPreset::Isometric:
            m_position = m_target + glm::normalize(float3(1.0f, 1.0f, 1.0f)) * 10.0f;
            m_worldUp  = float3(0.0f, 1.0f, 0.0f);
            break;
        default: break;
        }
        m_dirty = true;
    }

    // ---- Runtime Projection Switch with lerp ----
    void SwitchProjection(ProjectionType newType, float blendDuration = 0.3f) {
        if (m_projType == newType) return;
        m_blendFrom     = m_projType;
        m_projType      = newType;
        m_blendDuration = blendDuration;
        m_blendTimer    = 0.0f;
        m_blending      = true;
    }

    // ---- Update (call once per frame) ----
    void Update(float dt) {
        // Advance blend
        if (m_blending) {
            m_blendTimer += dt;
            m_blendT = glm::clamp(m_blendTimer / m_blendDuration, 0.0f, 1.0f);
            if (m_blendT >= 1.0f) m_blending = false;
            m_dirty = true;
        }

        if (!m_dirty) return;
        m_dirty = false;

        // View matrix
        m_view = glm::lookAt(m_position, m_target, m_worldUp);

        // Build both projections for interpolation
        float4x4 projA = BuildProjection(m_blending ? m_blendFrom : m_projType);
        float4x4 projB = BuildProjection(m_projType);

        if (m_blending) {
            // Lerp each column
            for (int c = 0; c < 4; ++c)
                m_proj[c] = glm::mix(projA[c], projB[c], m_blendT);
        } else {
            m_proj = projB;
        }

        m_viewProj = m_proj * m_view;
        m_frustum.ExtractFromVP(m_viewProj);

        // Upload UBO data
        CameraUBO data;
        data.view       = m_view;
        data.proj       = m_proj;
        data.viewProj   = m_viewProj;
        data.position   = float4(m_position, 1.0f);
        data.zParams    = float4(m_near, m_far, GetOrthoZoom(), (m_projType == ProjectionType::Orthographic ? 1.0f : 0.0f));

        if (m_ubo) {
            m_ubo->Update(&data, sizeof(CameraUBO));
        }
    }

    // ---- UBO Binding ----
    void CreateUBO(IGraphicsDevice* device) {
        BufferDesc desc;
        desc.size  = sizeof(CameraUBO);
        desc.usage = BufferUsage::Uniform;
        desc.isPersistent = true;
        m_ubo = device->CreateBuffer(desc);
    }
    IGpuBuffer* GetUBO() const { return m_ubo.get(); }

    // ---- Frustum culling convenience ----
    bool IsVisible(const float3& center, const float3& halfExtents) const {
        return m_frustum.TestAABB(center, halfExtents);
    }

protected:
    // Subclasses override these for their projection-specific parameters
    virtual float4x4 BuildProjection(ProjectionType type) = 0;
    virtual float    GetOrthoZoom()  const { return 1.0f; }

    // State
    float3 m_position = {0.0f, 0.0f, 5.0f};
    float3 m_target   = {0.0f, 0.0f, 0.0f};
    float3 m_worldUp  = {0.0f, 1.0f, 0.0f};
    float  m_aspect   = 16.0f / 9.0f;
    float  m_near     = 0.1f;
    float  m_far      = 1000.0f;

    // Matrices
    float4x4 m_view     = float4x4(1.0f);
    float4x4 m_proj     = float4x4(1.0f);
    float4x4 m_viewProj = float4x4(1.0f);
    Frustum  m_frustum;

    // Projection type & blending
    ProjectionType m_projType   = ProjectionType::Orthographic;
    ProjectionType m_blendFrom  = ProjectionType::Orthographic;
    bool  m_blending      = false;
    float m_blendDuration = 0.3f;
    float m_blendTimer    = 0.0f;
    float m_blendT        = 0.0f;

    ViewPreset m_preset = ViewPreset::Custom;
    bool       m_dirty  = true;

    // GPU resource
    std::shared_ptr<IGpuBuffer> m_ubo;
};

} // namespace AG
