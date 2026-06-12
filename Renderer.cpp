/**
 * @file Renderer.cpp
 * @brief 7단계 렌더 파이프라인 구현.
 */

#include "Renderer.h"
#include "OrthographicCamera.h"
#include "MeshRegistry.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace AG {

// ============================================================================
// Helper — TextureDesc shortcuts
// ============================================================================

static TextureDesc MakeRT(uint32_t w, uint32_t h, TextureFormat fmt) {
    return { w, h, fmt, 1, true };
}
static TextureDesc MakeDepth(uint32_t w, uint32_t h) {
    return { w, h, TextureFormat::D32_FLOAT, 1, true };
}
static TextureDesc MakeShadow(uint32_t res) {
    return { res, res, TextureFormat::D32_FLOAT, 1, true };
}

// ============================================================================
// Renderer — ctor / dtor
// ============================================================================

Renderer::Renderer(IGraphicsDevice* device, const RenderSettings& settings)
    : m_device(device), m_settings(settings)
{
    // ---- Default Camera: OrthographicCamera ----
    m_camera = std::make_shared<OrthographicCamera>(
        settings.orthoZoom,
        (float)settings.viewportWidth / (float)settings.viewportHeight
    );
    m_camera->SetNearFar(-100.0f, 100.0f);
    m_camera->CreateUBO(device);
    m_camera->ApplyPreset(ViewPreset::Front);

    // ---- Frame Constants UBO ----
    {
        BufferDesc desc;
        desc.size  = sizeof(FrameConstantsUBO);
        desc.usage = BufferUsage::Uniform;
        desc.isPersistent = true;
        m_frameConstantsUBO = device->CreateBuffer(desc);
    }

    // ---- SSAO Kernel UBO: 64 hemisphere samples ----
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        struct KernelSample { float4 v; };
        std::vector<KernelSample> kernel(64);
        for (uint32_t i = 0; i < 64; ++i) {
            float3 s = glm::normalize(float3(
                dist(rng) * 2.0f - 1.0f,
                dist(rng) * 2.0f - 1.0f,
                dist(rng)));
            float scale = (float)i / 64.0f;
            scale = glm::mix(0.1f, 1.0f, scale * scale);
            kernel[i].v = float4(s * scale * dist(rng), 0.0f);
        }
        BufferDesc desc;
        desc.size  = 64 * sizeof(KernelSample);
        desc.usage = BufferUsage::Uniform;
        m_ssaoKernelUBO = device->CreateBuffer(desc);
        m_ssaoKernelUBO->Update(kernel.data(), desc.size);
    }

    // ---- Ocean SSBO ----
    {
        auto& sim = SharedSimulationBuffer::Get();
        size_t bufSize = sizeof(OceanDisplacementSample) *
                         std::max(1u, sim.GetResX()) *
                         std::max(1u, sim.GetResZ());
        BufferDesc desc;
        desc.size  = bufSize;
        desc.usage = BufferUsage::Storage;
        m_oceanDataSSBO = device->CreateBuffer(desc);
    }

    // ---- Command List ----
    m_cmd = device->CreateCommandList();

    std::cout << "[Renderer] Initialized. Default: OrthographicCamera zoom="
              << settings.orthoZoom << "\n";
    std::cout << "[Renderer] Viewport: " << settings.viewportWidth
              << "x" << settings.viewportHeight << "\n";
}

Renderer::~Renderer() = default;

// ============================================================================
// RenderFrame
// ============================================================================

void Renderer::RenderFrame(float dt)
{
    // ---- Camera Update (TAA jitter 적용) ----
    m_prevJitter  = m_currJitter;
    m_prevViewProj = m_camera->GetViewProj();

    if (m_settings.taaEnabled) {
        m_currJitter = ComputeOrthoJitter(m_frameIndex % m_settings.taaJitterCount);
    } else {
        m_currJitter = float2(0.0f);
    }
    m_camera->Update(dt);

    // ---- 오션 변위 데이터를 GPU에 업로드 (읽기 전용 접근) ----
    if (m_settings.oceanEnabled) {
        auto& sim = SharedSimulationBuffer::Get();
        const OceanDisplacementSample* data = sim.AcquireRead();
        size_t byteSize = sizeof(OceanDisplacementSample)
                          * sim.GetResX() * sim.GetResZ();
        if (byteSize > 0 && data)
            m_oceanDataSSBO->Update(data, byteSize);
        sim.ReleaseRead();
    }

    // ---- Frame Constants ----
    UploadFrameConstants(dt);

    // ---- RenderGraph 재빌드 (설정 변경 또는 첫 프레임) ----
    if (m_graphDirty) {
        BuildRenderGraph();
        m_renderGraph.Compile();
        m_graphDirty = false;
    }

    // ---- 제출 큐에서 Frustum Culling ----
    m_opaqueQueue.erase(
        std::remove_if(m_opaqueQueue.begin(), m_opaqueQueue.end(),
            [&](const OpaqueDrawCall& dc) {
                // AABB를 추출하지 않으므로 여기서는 보수적으로 통과
                return false;
            }),
        m_opaqueQueue.end());

    // ---- Execute ----
    m_cmd->Begin();
    m_renderGraph.Execute(m_device, m_cmd.get());
    m_cmd->End();
    m_device->Submit(m_cmd.get());

    // ---- Flip TAA history ----
    m_taaHistoryPing = !m_taaHistoryPing;

    // ---- Reset submission queues ----
    m_opaqueQueue.clear();
    m_transpQueue.clear();
    m_uiQueue.clear();

    ++m_frameIndex;
}

// ============================================================================
// OnResize
// ============================================================================

void Renderer::OnResize(uint32_t width, uint32_t height)
{
    m_settings.viewportWidth  = width;
    m_settings.viewportHeight = height;
    m_camera->SetAspect((float)width / (float)height);
    m_graphDirty = true; // 렌더 타겟 재생성
    std::cout << "[Renderer] Resize → " << width << "x" << height << "\n";
}

// ============================================================================
// Scene Submission
// ============================================================================

void Renderer::SubmitOpaque(GpuMesh* mesh, const float4x4& transform, uint32_t lod) {
    m_opaqueQueue.push_back({ mesh, transform, lod });
}
void Renderer::SubmitTransparent(GpuMesh* mesh, const float4x4& transform, float alpha) {
    m_transpQueue.push_back({ mesh, transform, alpha });
}
void Renderer::SubmitUI(GpuMesh* mesh, const float4x4& transform, uint32_t texId) {
    m_uiQueue.push_back({ mesh, transform, texId });
}

// ============================================================================
// SetCamera / ApplySettings
// ============================================================================

void Renderer::SetCamera(std::shared_ptr<Camera> cam) {
    // OrthographicCamera 특수화 유지
    m_camera = std::dynamic_pointer_cast<OrthographicCamera>(cam);
    if (!m_camera) {
        // Fallback: 기본 카메라 유지 (제약 조건: 기본은 반드시 Ortho)
        std::cerr << "[Renderer] Warning: Camera must be OrthographicCamera. Keeping default.\n";
    }
}

void Renderer::ApplySettings() {
    m_graphDirty = true;
}

// ============================================================================
// BuildRenderGraph
// ============================================================================

void Renderer::BuildRenderGraph()
{
    m_renderGraph = RenderGraph{}; // 초기화

    const uint32_t W  = m_settings.viewportWidth;
    const uint32_t H  = m_settings.viewportHeight;
    const uint32_t SR = m_settings.shadow.resolution;

    // ---- Shadow Map ----
    m_shadowMap = m_renderGraph.CreateTexture("ShadowMap", MakeShadow(SR));

    // ---- G-Buffer RTs ----
    m_gbuffer.albedoRT   = m_renderGraph.CreateTexture("GBuf_Albedo",   MakeRT(W, H, TextureFormat::RGBA8_UNORM));
    m_gbuffer.normalRT   = m_renderGraph.CreateTexture("GBuf_Normal",   MakeRT(W, H, TextureFormat::RGBA16_FLOAT));
    m_gbuffer.materialRT = m_renderGraph.CreateTexture("GBuf_Material", MakeRT(W, H, TextureFormat::RGBA8_UNORM));
    m_gbuffer.velocityRT = m_renderGraph.CreateTexture("GBuf_Velocity", MakeRT(W, H, TextureFormat::RGBA16_FLOAT));
    m_gbuffer.depthRT    = m_renderGraph.CreateTexture("GBuf_Depth",    MakeDepth(W, H));

    // ---- Post-Process RTs ----
    m_pp.hdrRT       = m_renderGraph.CreateTexture("HDR",          MakeRT(W, H, TextureFormat::RGBA16_FLOAT));
    m_pp.bloomBright = m_renderGraph.CreateTexture("BloomBright",  MakeRT(W/2, H/2, TextureFormat::RGBA16_FLOAT));
    m_pp.bloomBlurA  = m_renderGraph.CreateTexture("BloomBlurA",   MakeRT(W/2, H/2, TextureFormat::RGBA16_FLOAT));
    m_pp.bloomBlurB  = m_renderGraph.CreateTexture("BloomBlurB",   MakeRT(W/2, H/2, TextureFormat::RGBA16_FLOAT));
    m_pp.ssaoRT      = m_renderGraph.CreateTexture("SSAO",         MakeRT(W/2, H/2, TextureFormat::RGBA8_UNORM));
    m_pp.taaHistoryA = m_renderGraph.CreateTexture("TAA_HistA",    MakeRT(W, H, TextureFormat::RGBA16_FLOAT));
    m_pp.taaHistoryB = m_renderGraph.CreateTexture("TAA_HistB",    MakeRT(W, H, TextureFormat::RGBA16_FLOAT));
    m_pp.ldrRT       = m_renderGraph.CreateTexture("LDR_Output",   MakeRT(W, H, TextureFormat::RGBA8_UNORM));

    // ---- 패스 등록 ----
    AddShadowPass();
    AddGBufferPass();
    AddLightingPass();
    AddOceanPass();
    AddTransparentPass();
    AddPostProcessPass();
    AddUIPass();
}

// ============================================================================
// Shadow Pass
// ============================================================================

void Renderer::AddShadowPass()
{
    m_renderGraph.AddPass("ShadowPass",
        {},                    // Reads
        { m_shadowMap },       // Writes
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("Shadow");
            // glBindFramebuffer → depth only attachment
            // glUse(shadowProgram)
            // CameraUBO = light-space ortho camera (binding=0)
            for (auto& dc : m_opaqueQueue) {
                // glUniformMatrix4fv(model, dc.transform)
                dc.mesh->Draw(dc.lod);
                (void)dc;
            }
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// G-Buffer Pass
// ============================================================================

void Renderer::AddGBufferPass()
{
    m_renderGraph.AddPass("GBufferPass",
        { m_shadowMap },
        { m_gbuffer.albedoRT, m_gbuffer.normalRT,
          m_gbuffer.materialRT, m_gbuffer.velocityRT, m_gbuffer.depthRT },
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("GBuffer");
            // MRT 설정: GL_COLOR_ATTACHMENT0 ~ 3
            // glUse(gbufferProgram)
            // Bind CameraUBO (binding=0), FrameConstantsUBO (binding=1)
            for (auto& dc : m_opaqueQueue) {
                // glUniformMatrix4fv(model/prevModel, ...)
                dc.mesh->Draw(dc.lod);
                (void)dc;
            }
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// Lighting Pass (Deferred)
// ============================================================================

void Renderer::AddLightingPass()
{
    m_renderGraph.AddPass("LightingPass",
        { m_gbuffer.albedoRT, m_gbuffer.normalRT,
          m_gbuffer.materialRT, m_gbuffer.depthRT,
          m_shadowMap, m_pp.ssaoRT },
        { m_pp.hdrRT },
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("Lighting");
            // Full-screen quad
            // glUse(lightingProgram)
            // Bind all G-Buffer textures (binding 0..5)
            // PBR + shadow lookup + SSAO occlusion
            ctx.cmd->Draw(3); // full-screen triangle
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// Ocean Pass
// ============================================================================

void Renderer::AddOceanPass()
{
    if (!m_settings.oceanEnabled) return;

    m_renderGraph.AddPass("OceanPass",
        { m_gbuffer.depthRT, m_pp.hdrRT },  // SSR reads hdrRT
        { m_pp.hdrRT },
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("Ocean");
            // glUse(oceanProgram)
            // Bind displacement SSBO (binding=2)
            // Ocean mesh = MeshRegistry::Get().Find("Ocean") or plane LOD
            auto oceanMesh = MeshRegistry::Get().Find("Plane");
            if (oceanMesh) {
                // 변위 맵 적용은 vertex shader 내에서 SSBO 참조
                // SSR은 fragment shader의 march loop
                (void)ctx;
            }
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// Transparent Pass
// ============================================================================

void Renderer::AddTransparentPass()
{
    // 투명 오브젝트는 카메라 거리 기준 back-to-front 정렬 후 블렌딩
    m_renderGraph.AddPass("TransparentPass",
        { m_gbuffer.depthRT },
        { m_pp.hdrRT },
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("Transparent");

            // 거리 정렬 (직교 카메라: Y축 거리)
            float3 camPos = m_camera->GetPosition();
            std::sort(m_transpQueue.begin(), m_transpQueue.end(),
                [&](const TranspDrawCall& a, const TranspDrawCall& b) {
                    float3 pa = float3(a.transform[3]);
                    float3 pb = float3(b.transform[3]);
                    return glm::length2(pa - camPos) > glm::length2(pb - camPos);
                });

            for (auto& dc : m_transpQueue) {
                // glEnable(GL_BLEND)
                // glUniform1f(alpha, dc.alpha)
                dc.mesh->Draw(0);
                (void)dc;
            }
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// Post-Process Pass (SSAO → TAA → Bloom → Tone Map)
// ============================================================================

void Renderer::AddPostProcessPass()
{
    const ResourceHandle taaSrc  = m_taaHistoryPing ? m_pp.taaHistoryA : m_pp.taaHistoryB;
    const ResourceHandle taaDst  = m_taaHistoryPing ? m_pp.taaHistoryB : m_pp.taaHistoryA;

    // ---- SSAO ----
    if (m_settings.ssaoEnabled) {
        m_renderGraph.AddPass("SSAO",
            { m_gbuffer.normalRT, m_gbuffer.depthRT },
            { m_pp.ssaoRT },
            [this](RenderPassContext& ctx) {
                ctx.cmd->PushMarker("SSAO");
                // glUse(ssaoProgram)
                // Bind normal RT, depth RT, random noise texture
                // Bind SSAO kernel UBO (binding=3)
                ctx.cmd->Draw(3);
                ctx.cmd->PopMarker();
            }
        );
    }

    // ---- TAA ----
    if (m_settings.taaEnabled) {
        m_renderGraph.AddPass("TAA",
            { m_pp.hdrRT, taaSrc, m_gbuffer.velocityRT, m_gbuffer.depthRT },
            { taaDst },
            [this, taaDst](RenderPassContext& ctx) {
                ctx.cmd->PushMarker("TAA");
                // glUse(taaProgram)
                // Uniform: jitter (ortho space), blendFactor ~ 0.1
                ctx.cmd->Draw(3);
                ctx.cmd->PopMarker();
            }
        );
    }

    const ResourceHandle taaOutput = m_settings.taaEnabled ? taaDst : m_pp.hdrRT;

    // ---- Bloom: Threshold ----
    if (m_settings.bloomEnabled) {
        m_renderGraph.AddPass("Bloom_Threshold",
            { taaOutput },
            { m_pp.bloomBright },
            [this](RenderPassContext& ctx) {
                ctx.cmd->PushMarker("BloomThreshold");
                ctx.cmd->Draw(3);
                ctx.cmd->PopMarker();
            }
        );

        // Bloom: Dual-Kawase Blur (2 iterations)
        m_renderGraph.AddPass("Bloom_Blur1",
            { m_pp.bloomBright },
            { m_pp.bloomBlurA },
            [](RenderPassContext& ctx) { ctx.cmd->Draw(3); }
        );
        m_renderGraph.AddPass("Bloom_Blur2",
            { m_pp.bloomBlurA },
            { m_pp.bloomBlurB },
            [](RenderPassContext& ctx) { ctx.cmd->Draw(3); }
        );
    }

    // ---- Tone Mapping + Chromatic Aberration + Bloom Composite ----
    std::vector<ResourceHandle> toneReads = { taaOutput };
    if (m_settings.bloomEnabled) {
        toneReads.push_back(m_pp.bloomBlurB);
    }

    m_renderGraph.AddPass("ToneMapping",
        toneReads,
        { m_pp.ldrRT },
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("ToneMapping");
            // glUse(tonemapProgram)
            // ACES Filmic + chromatic aberration if enabled
            ctx.cmd->Draw(3);
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// UI Pass
// ============================================================================

void Renderer::AddUIPass()
{
    m_renderGraph.AddPass("UIPass",
        {},
        { m_pp.ldrRT },
        [this](RenderPassContext& ctx) {
            ctx.cmd->PushMarker("UI");
            // glDisable(GL_DEPTH_TEST)
            // glEnable(GL_BLEND), glBlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA)
            for (auto& dc : m_uiQueue) {
                // glBindTexture(dc.texId)
                dc.mesh->Draw(0);
                (void)dc;
            }
            ctx.cmd->PopMarker();
        }
    );
}

// ============================================================================
// TAA Jitter (Orthographic Space)
// ============================================================================

float Renderer::Halton(uint32_t index, uint32_t base)
{
    float result = 0.0f;
    float f      = 1.0f;
    uint32_t i   = index;
    while (i > 0) {
        f      /= (float)base;
        result += f * (float)(i % base);
        i      /= base;
    }
    return result;
}

float2 Renderer::ComputeOrthoJitter(uint32_t frameIndex) const
{
    // Halton(base=2, base=3) → [0,1)에서 픽셀 크기로 스케일
    float hx = Halton(frameIndex + 1, 2) - 0.5f;
    float hy = Halton(frameIndex + 1, 3) - 0.5f;

    // Orthographic 공간 픽셀 크기 = orthoZoom * aspect / viewportWidth
    float zoom  = m_camera->GetOrthoZoom();
    float asp   = (float)m_settings.viewportWidth / (float)m_settings.viewportHeight;
    float pixW  = zoom * asp / (float)m_settings.viewportWidth;
    float pixH  = zoom       / (float)m_settings.viewportHeight;

    return float2(hx * pixW, hy * pixH);
}

// ============================================================================
// Upload Frame Constants
// ============================================================================

void Renderer::UploadFrameConstants(float dt)
{
    FrameConstantsUBO ubo{};
    ubo.prevViewProj     = m_prevViewProj;
    ubo.jitter           = float4(m_currJitter, m_prevJitter);
    ubo.viewportSize     = float4(
        (float)m_settings.viewportWidth,
        (float)m_settings.viewportHeight,
        1.0f / (float)m_settings.viewportWidth,
        1.0f / (float)m_settings.viewportHeight);
    ubo.bloomParams      = float4(m_settings.bloomThreshold,
                                   m_settings.bloomIntensity, 0.7f, 0.0f);
    ubo.ssaoParams       = float4(m_settings.ssaoRadius, m_settings.ssaoBias,
                                   m_settings.ssaoPower, 64.0f);
    ubo.chromaticAberration = float4(m_settings.chromaticStrength,
                                      m_settings.chromaticEnabled ? 1.0f : 0.0f,
                                      0.0f, 0.0f);
    ubo.timeVec          = float4(
        (float)m_frameIndex / 60.0f, dt, 0.0f, 0.0f);
    ubo.toneMapParams    = float4(m_settings.exposure, 0.0f, 0.0f, 0.0f);

    m_frameConstantsUBO->Update(&ubo, sizeof(ubo));
}

// ============================================================================
// Profiling
// ============================================================================

std::vector<std::pair<std::string, float>> Renderer::GetPassTimings() const {
    return m_passTimings;
}

} // namespace AG
