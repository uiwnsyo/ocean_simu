#include "OpenGLDevice.h"
#include <iostream>

namespace AG {

// --- OpenGLBuffer Implementation ---
OpenGLBuffer::OpenGLBuffer(const BufferDesc& desc) : size(desc.size) {
    // glCreateBuffers(1, &handle);
    // glNamedBufferStorage(handle, size, nullptr, flags);
    std::cout << "[HAL] OpenGL Buffer Created: " << size << " bytes\n";
}

OpenGLBuffer::~OpenGLBuffer() {
    // glDeleteBuffers(1, &handle);
}

void OpenGLBuffer::Update(const void* data, size_t size, size_t offset) {
    // glNamedBufferSubData(handle, offset, size, data);
}

// --- OpenGLTexture Implementation ---
OpenGLTexture::OpenGLTexture(const TextureDesc& desc) : desc(desc) {
    // glCreateTextures(GL_TEXTURE_2D, 1, &handle);
    // glTextureStorage2D(handle, ...);
    std::cout << "[HAL] OpenGL Texture Created: " << desc.width << "x" << desc.height << "\n";
}

OpenGLTexture::~OpenGLTexture() {
    // glDeleteTextures(1, &handle);
}

// --- OpenGLCommandList Implementation ---
void OpenGLCommandList::Draw(uint vertexCount, uint instanceCount) {
    // glDrawArraysInstanced(...)
    std::cout << "[HAL] Draw Call: " << vertexCount << " vertices\n";
}

void OpenGLCommandList::Dispatch(uint groupX, uint groupY, uint groupZ) {
    // glDispatchCompute(...)
    std::cout << "[HAL] Dispatch Call: " << groupX << "x" << groupY << "x" << groupZ << "\n";
}

void OpenGLCommandList::PushMarker(const std::string& name) {
    // glPushDebugGroup(GL_DEBUG_SOURCE_APPLICATION, 0, -1, name.c_str());
    std::cout << "[Debug] >>> Pass: " << name << "\n";
}

void OpenGLCommandList::PopMarker() {
    // glPopDebugGroup();
    std::cout << "[Debug] <<< End Pass\n";
}

} // namespace AG
