#pragma once
#include "GraphicsHAL.h"
#include <map>

namespace AG {

class OpenGLBuffer : public IGpuBuffer {
public:
    uint32_t handle;
    size_t size;
    OpenGLBuffer(const BufferDesc& desc);
    ~OpenGLBuffer();
    void Update(const void* data, size_t size, size_t offset) override;
};

class OpenGLTexture : public IGpuTexture {
public:
    uint32_t handle;
    TextureDesc desc;
    OpenGLTexture(const TextureDesc& desc);
    ~OpenGLTexture();
};

class OpenGLCommandList : public IGpuCommandList {
public:
    void Begin() override {}
    void End() override {}
    void SetPipeline(IGpuPipeline* pipeline) override {}
    void SetBuffer(IGpuBuffer* buffer, uint slot) override {}
    void SetTexture(IGpuTexture* texture, uint slot) override {}
    void Draw(uint vertexCount, uint instanceCount) override;
    void Dispatch(uint groupX, uint groupY, uint groupZ) override;
    void PushMarker(const std::string& name) override;
    void PopMarker() override;
};

class OpenGLDevice : public IGraphicsDevice {
public:
    std::shared_ptr<IGpuBuffer> CreateBuffer(const BufferDesc& desc) override {
        return std::make_shared<OpenGLBuffer>(desc);
    }
    std::shared_ptr<IGpuTexture> CreateTexture(const TextureDesc& desc) override {
        return std::make_shared<OpenGLTexture>(desc);
    }
    std::shared_ptr<IGpuCommandList> CreateCommandList() override {
        return std::make_shared<OpenGLCommandList>();
    }
    
    void Submit(IGpuCommandList* cmd) override {}
    void WaitIdle() override {}
    uint64_t GetTimestampFrequency() override { return 1000000; } // NS
};

} // namespace AG


