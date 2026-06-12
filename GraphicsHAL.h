#pragma once
#include <string>
#include <vector>
#include <memory>
#include "AGMath.h"

namespace AG {

enum class BufferUsage {
    Vertex, Index, Uniform, Storage, Indirect
};

enum class TextureFormat {
    RGBA8_UNORM, RGBA16_FLOAT, D24_S8, D32_FLOAT
};

struct BufferDesc {
    size_t size;
    BufferUsage usage;
    bool isPersistent = false;
};

struct TextureDesc {
    uint width, height;
    TextureFormat format;
    uint mips = 1;
    bool isRenderTarget = false;
};

class IGpuBuffer {
public:
    virtual ~IGpuBuffer() = default;
    virtual void Update(const void* data, size_t size, size_t offset = 0) = 0;
};

class IGpuTexture {
public:
    virtual ~IGpuTexture() = default;
};

class IGpuPipeline {
public:
    virtual ~IGpuPipeline() = default;
};

class IGpuCommandList {
public:
    virtual ~IGpuCommandList() = default;
    virtual void Begin() = 0;
    virtual void End() = 0;
    
    virtual void SetPipeline(IGpuPipeline* pipeline) = 0;
    virtual void SetBuffer(IGpuBuffer* buffer, uint slot) = 0;
    virtual void SetTexture(IGpuTexture* texture, uint slot) = 0;
    
    virtual void Draw(uint vertexCount, uint instanceCount = 1) = 0;
    virtual void Dispatch(uint groupX, uint groupY, uint groupZ) = 0;
    
    // RenderDoc / Debug markers
    virtual void PushMarker(const std::string& name) = 0;
    virtual void PopMarker() = 0;
};

class IGraphicsDevice {
public:
    virtual ~IGraphicsDevice() = default;
    
    virtual std::shared_ptr<IGpuBuffer> CreateBuffer(const BufferDesc& desc) = 0;
    virtual std::shared_ptr<IGpuTexture> CreateTexture(const TextureDesc& desc) = 0;
    virtual std::shared_ptr<IGpuCommandList> CreateCommandList() = 0;
    
    virtual void Submit(IGpuCommandList* cmd) = 0;
    virtual void WaitIdle() = 0;

    // Timer Query
    virtual uint64_t GetTimestampFrequency() = 0;
};

} // namespace AG
