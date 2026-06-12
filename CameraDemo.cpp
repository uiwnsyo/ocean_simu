#include <iostream>
#include <cassert>

// -- Minimal stub for IGraphicsDevice (no GL/Vulkan needed for this test) --
#include "OpenGLDevice.h"
#include "OrthographicCamera.h"
#include "PerspectiveCamera.h"
#include "CameraController.h"

using namespace AG;

// Stub device stub that simulates UBO upload
struct StubBuffer : IGpuBuffer {
    std::vector<uint8_t> data;
    void Update(const void* src, size_t size, size_t offset = 0) override {
        if (data.size() < offset + size) data.resize(offset + size);
        std::memcpy(data.data() + offset, src, size);
        std::cout << "[UBO] Camera UBO updated (" << size << " bytes)\n";
    }
};

struct StubDevice : IGraphicsDevice {
    std::shared_ptr<IGpuBuffer> CreateBuffer(const BufferDesc&) override {
        return std::make_shared<StubBuffer>();
    }
    std::shared_ptr<IGpuTexture> CreateTexture(const TextureDesc&) override { return nullptr; }
    std::shared_ptr<IGpuCommandList> CreateCommandList() override { return nullptr; }
    void Submit(IGpuCommandList*) override {}
    void WaitIdle() override {}
    uint64_t GetTimestampFrequency() override { return 1000000; }
};

int main() {
    std::cout << "=== Camera System Demo ===\n\n";

    StubDevice device;

    // ----------------------------------------------------------------
    // 1. Orthographic Camera (default)
    // ----------------------------------------------------------------
    OrthographicCamera ortho(10.0f, 16.0f / 9.0f);
    ortho.SetNearFar(-100.0f, 100.0f);
    ortho.CreateUBO(&device);

    std::cout << "-- Ortho: Initial update --\n";
    ortho.Update(0.016f);

    // Frustum culling test
    bool vis = ortho.IsVisible(float3(0,0,0), float3(1,1,1));
    std::cout << "AABB(0,0,0) r=1 visible: " << (vis ? "YES" : "NO") << "\n\n";

    // ----------------------------------------------------------------
    // 2. View Presets
    // ----------------------------------------------------------------
    std::cout << "-- Applying TOP preset --\n";
    ortho.ApplyPreset(ViewPreset::Top);
    ortho.Update(0.016f);

    std::cout << "-- Applying FRONT preset --\n";
    ortho.ApplyPreset(ViewPreset::Front);
    ortho.Update(0.016f);

    // ----------------------------------------------------------------
    // 3. OrthoController: pan + zoom
    // ----------------------------------------------------------------
    OrthoController orthoCtrl;
    CameraInput input;
    input.viewportSize   = {1920, 1080};
    input.dt             = 0.016f;

    // Scroll zoom in
    input.scrollDelta    = 3.0f;
    orthoCtrl.OnInput(input, ortho);
    ortho.Update(0.016f);
    std::cout << "-- Ortho zoom in (scroll+3) → zoom=" << ortho.GetOrthoZoom() << " --\n\n";

    // Panning
    input.scrollDelta    = 0.0f;
    input.mouseMiddleDown= true;
    input.mouseDelta     = {50, 30};
    orthoCtrl.OnInput(input, ortho);
    ortho.Update(0.016f);
    std::cout << "-- Ortho pan (50,30 pixels) --\n\n";

    // ----------------------------------------------------------------
    // 4. Projection switch with blend animation
    // ----------------------------------------------------------------
    PerspectiveCamera persp(60.0f, 16.0f/9.0f);
    persp.CreateUBO(&device);
    persp.Update(0.016f);

    std::cout << "-- Switching Perspective → Orthographic with 0.3s blend --\n";
    persp.SwitchProjection(ProjectionType::Orthographic, 0.3f);

    // Simulate 20 frames at 60fps
    for (int i = 0; i < 20; ++i) {
        persp.Update(0.016f);
    }
    std::cout << "Blend complete.\n\n";

    // ----------------------------------------------------------------
    // 5. Orbit Controller
    // ----------------------------------------------------------------
    PerspectiveCamera orbit(60.0f, 16.0f/9.0f);
    orbit.CreateUBO(&device);
    OrbitController orbitCtrl;

    input = {};
    input.mouseRightDown = true;
    input.mouseDelta     = {30, -15};
    input.dt             = 0.016f;
    orbitCtrl.OnInput(input, orbit);
    orbit.Update(0.016f);
    std::cout << "-- Orbit: dragged right 30px, up 15px --\n";
    auto pos = orbit.GetPosition();
    std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n\n";

    // ----------------------------------------------------------------
    // 6. Fly Controller
    // ----------------------------------------------------------------
    PerspectiveCamera fly(60.0f, 16.0f/9.0f);
    fly.CreateUBO(&device);
    FlyController flyCtrl;

    input = {};
    input.keyW = true;
    input.dt   = 0.016f;
    flyCtrl.OnInput(input, fly);
    fly.Update(0.016f);
    std::cout << "-- Fly: moved forward (W key) --\n";
    auto flyPos = fly.GetPosition();
    std::cout << "Position: (" << flyPos.x << ", " << flyPos.y << ", " << flyPos.z << ")\n\n";

    std::cout << "=== Camera System Demo Complete ===\n";
    return 0;
}
