#pragma once
/**
 * @file Renderer.h
 * @brief 메인 렌더러 — 7단계 렌더 패스 파이프라인.
 *
 * 패스 순서:
 *   Shadow → G-Buffer → Lighting → Ocean → Transparent → PostProcess → UI
 *
 * 기본 카메라: OrthographicCamera (렌더러 초기화 시 자동 생성)
 * HDR Framebuffer: RGBA16F → LDR (tone mapping)
 * TAA 지터: Orthographic 공간 기준 Halton 시퀀스 적용
 */

#include "AGMath.h"
#include "GraphicsHAL.h"
#include "RenderGraph.h"
#include "Camera.h"
#include "OrthographicCamera.h"
#include "GpuMesh.h"
#include "SharedSimulationBuffer.h"
#include <array>
#include <memory>
#include <vector>
#include <string>

namespace AG {

// ============================================================================
// Frame Constants UBO (binding = 1)
// ============================================================================

/** @brief 포스트 프로세스 파라미터 등 프레임 상수. std140 layout. */
struct alignas(16) FrameConstantsUBO {
    float4x4 prevViewProj;        ///< 이전 프레임 VP (TAA reprojection)
    float4   jitter;              ///< xy = 현재 지터, zw = 이전 지터 (ortho 공간)
    float4   viewportSize;        ///< xy = 뷰포트, zw = 1/뷰포트
    float4   bloomParams;         ///< x = threshold, y = intensity, z = scatter, w = unused
    float4   ssaoParams;          ///< x = radius, y = bias, z = power, w = sampleCount
    float4   chromaticAberration; ///< x = strength, y = enabled, zw = unused
    float4   timeVec;             ///< x = time, y = deltaTime, zw = unused
    float4   toneMapParams;       ///< x = exposure, yzw = unused
};

// ============================================================================
// G-Buffer Resource Handles (렌더러가 소유)
// ============================================================================

struct GBufferTargets {
    ResourceHandle albedoRT;    ///< RGBA8_UNORM  (albedo + alpha)
    ResourceHandle normalRT;    ///< RGBA16F      (view-space normal + roughness)
    ResourceHandle materialRT;  ///< RGBA8_UNORM  (metallic, roughness, AO, emission mask)
    ResourceHandle velocityRT;  ///< RG16F        (screen-space velocity for TAA)
    ResourceHandle depthRT;     ///< D32_FLOAT
};

struct PostProcessTargets {
    ResourceHandle hdrRT;       ///< RGBA16F — Lighting 결과 + Ocean  
    ResourceHandle bloomBright; ///< RGBA16F — Bloom threshold 후 밝은 영역
    ResourceHandle bloomBlurA;  ///< RGBA16F — Bloom 블러 ping
    ResourceHandle bloomBlurB;  ///< RGBA16F — Bloom 블러 pong
    ResourceHandle ssaoRT;      ///< R8_UNORM — SSAO 결과
    ResourceHandle taaHistoryA; ///< RGBA16F — TAA history ping
    ResourceHandle taaHistoryB; ///< RGBA16F — TAA history pong
    ResourceHandle ldrRT;       ///< RGBA8_UNORM — 최종 LDR 출력
};

// ============================================================================
// ShadowMap
// ============================================================================

struct ShadowMapConfig {
    uint32_t resolution      = 2048;   ///< 그림자 맵 해상도
    float    bias            = 0.005f; ///< Shadow bias
    float    nearPlane       = 0.5f;
    float    farPlane        = 200.0f;
    float    orthoSize       = 50.0f;  ///< 직교 투영 그림자 카메라 크기
};

// ============================================================================
// RenderSettings
// ============================================================================

struct RenderSettings {
    uint32_t viewportWidth   = 1920;
    uint32_t viewportHeight  = 1080;

    // TAA
    bool     taaEnabled      = true;
    uint32_t taaJitterCount  = 8;      ///< Halton 시퀀스 샘플 수

    // Bloom
    bool     bloomEnabled    = true;
    float    bloomThreshold  = 1.0f;
    float    bloomIntensity  = 0.3f;

    // SSAO
    bool     ssaoEnabled     = true;
    float    ssaoRadius      = 0.5f;
    float    ssaoBias        = 0.025f;
    float    ssaoPower       = 2.2f;

    // Chromatic Aberration
    bool     chromaticEnabled   = false;
    float    chromaticStrength  = 0.003f;

    // Tone Mapping
    float    exposure        = 1.0f;

    // Ocean
    bool     oceanEnabled    = true;
    bool     oceanSSREnabled = true;

    // Shadow
    ShadowMapConfig shadow;

    // Camera (Orthographic default)
    float    orthoZoom       = 10.0f;
    float    cameraFovDeg    = 60.0f;
};

// ============================================================================
// Renderer
// ============================================================================

/**
 * @brief 메인 렌더러 클래스.
 *
 * 초기화 시 OrthographicCamera를 기본 카메라로 생성하며,
 * RenderGraph를 통해 7개 패스를 순서대로 실행한다.
 *
 * @note UI Pass는 항상 마지막에 실행되고 depth 테스트 없이 그린다.
 */
class Renderer {
public:
    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /**
     * @brief 렌더러를 초기화한다.
     * @param device   GPU 디바이스
     * @param settings 렌더 설정
     */
    explicit Renderer(IGraphicsDevice* device, const RenderSettings& settings = {});
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;

    // ------------------------------------------------------------------
    // Frame
    // ------------------------------------------------------------------

    /**
     * @brief 프레임을 렌더링한다.
     *
     * 내부적으로 RenderGraph::Compile() + Execute()를 순서대로 수행한다.
     *
     * @param dt 이전 프레임과의 시간 간격 (초)
     */
    void RenderFrame(float dt);

    /**
     * @brief 뷰포트 크기가 변경될 때 (창 리사이즈 시) 호출한다.
     * @param width  새 뷰포트 너비
     * @param height 새 뷰포트 높이
     */
    void OnResize(uint32_t width, uint32_t height);

    // ------------------------------------------------------------------
    // Camera
    // ------------------------------------------------------------------

    /**
     * @brief 현재 기본 카메라(OrthographicCamera)를 반환한다.
     */
    OrthographicCamera* GetCamera() { return m_camera.get(); }

    /**
     * @brief 기본 카메라를 외부 교체한다 (소유권 이전).
     */
    void SetCamera(std::shared_ptr<Camera> cam);

    // ------------------------------------------------------------------
    // Settings
    // ------------------------------------------------------------------

    RenderSettings& GetSettings()             { return m_settings; }
    const RenderSettings& GetSettings() const { return m_settings; }

    /**
     * @brief 설정 변경 후 렌더 타겟 재생성이 필요한 경우 호출.
     */
    void ApplySettings();

    // ------------------------------------------------------------------
    // Scene Submission
    // ------------------------------------------------------------------

    /**
     * @brief 렌더링할 불투명 오브젝트를 등록한다.
     * @param gpuMesh  GPU 메시 핸들
     * @param transform 오브젝트 월드 변환 행렬
     * @param lod       LOD 레벨 (0 = 최고 품질)
     */
    void SubmitOpaque(GpuMesh* gpuMesh, const float4x4& transform, uint32_t lod = 0);

    /**
     * @brief 렌더링할 반투명 오브젝트를 등록한다.
     */
    void SubmitTransparent(GpuMesh* gpuMesh, const float4x4& transform, float alpha);

    /**
     * @brief 렌더링할 UI 쿼드를 등록한다 (스크린 좌표).
     */
    void SubmitUI(GpuMesh* gpuMesh, const float4x4& uiTransform, uint32_t textureId);

    // ------------------------------------------------------------------
    // Debug / Profiling
    // ------------------------------------------------------------------

    /** @brief 각 패스의 GPU 실행 시간(ms)을 반환한다. */
    std::vector<std::pair<std::string, float>> GetPassTimings() const;

private:
    // ------------------------------------------------------------------
    // RenderGraph Setup
    // ------------------------------------------------------------------

    /** @brief RenderGraph 리소스 및 패스 전체를 등록한다. */
    void BuildRenderGraph();

    void AddShadowPass();
    void AddGBufferPass();
    void AddLightingPass();
    void AddOceanPass();
    void AddTransparentPass();
    void AddPostProcessPass();
    void AddUIPass();

    // ------------------------------------------------------------------
    // TAA Jitter (Orthographic Space)
    // ------------------------------------------------------------------

    /**
     * @brief Halton 시퀀스 기반 TAA 지터를 계산한다.
     *
     * Orthographic 카메라에서는 NDC 지터가 아닌
     * ortho 공간의 픽셀 크기 단위로 오프셋을 적용한다.
     *
     * @param frameIndex 현재 프레임 번호 (mod m_taaJitterCount)
     * @return float2 (orthoSpaceX, orthoSpaceY) 지터 오프셋
     */
    float2 ComputeOrthoJitter(uint32_t frameIndex) const;

    /**
     * @brief Halton 수열 단일 값.
     * @param index  수열 인덱스
     * @param base   수열 밑수 (2 또는 3)
     */
    static float Halton(uint32_t index, uint32_t base);

    // ------------------------------------------------------------------
    // UBO Upload
    // ------------------------------------------------------------------

    void UploadFrameConstants(float dt);

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    IGraphicsDevice*  m_device   = nullptr;
    RenderSettings    m_settings;

    // Camera — OrthographicCamera by default
    std::shared_ptr<OrthographicCamera> m_camera;

    // RenderGraph
    RenderGraph m_renderGraph;
    bool        m_graphDirty = true; ///< true → 다음 프레임에 RenderGraph 재빌드

    // G-Buffer & Post-Process targets
    GBufferTargets     m_gbuffer;
    PostProcessTargets m_pp;
    ResourceHandle     m_shadowMap;

    // Frame state
    uint32_t m_frameIndex    = 0;
    float4x4 m_prevViewProj  = float4x4(1.0f);
    float2   m_prevJitter    = float2(0.0f);
    float2   m_currJitter    = float2(0.0f);
    bool     m_taaHistoryPing = true; ///< ping-pong flip

    // GPU Buffers
    std::shared_ptr<IGpuBuffer> m_frameConstantsUBO; ///< binding = 1
    std::shared_ptr<IGpuBuffer> m_ssaoKernelUBO;     ///< SSAO 반구 샘플 커널
    std::shared_ptr<IGpuBuffer> m_oceanDataSSBO;     ///< 오션 변위 데이터 (SSBO)

    // Command list
    std::shared_ptr<IGpuCommandList> m_cmd;

    // Per-frame submission lists
    struct OpaqueDrawCall  { GpuMesh* mesh; float4x4 transform; uint32_t lod; };
    struct TranspDrawCall  { GpuMesh* mesh; float4x4 transform; float alpha; };
    struct UIDrawCall      { GpuMesh* mesh; float4x4 transform; uint32_t texId; };

    std::vector<OpaqueDrawCall> m_opaqueQueue;
    std::vector<TranspDrawCall> m_transpQueue;
    std::vector<UIDrawCall>     m_uiQueue;

    // Timing
    std::vector<std::pair<std::string, float>> m_passTimings;
};

} // namespace AG
