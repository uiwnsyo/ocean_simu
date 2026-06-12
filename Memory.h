#pragma once
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace AG {

class LinearAllocator {
public:
  LinearAllocator(size_t size) : totalSize(size) {
    memory = (uint8_t *)std::malloc(size);
    offset = 0;
  }

  ~LinearAllocator() { std::free(memory); }

  void *Allocate(size_t size, size_t alignment = 8) {
    size_t padding = (alignment - (offset % alignment)) % alignment;
    if (offset + size + padding > totalSize)
      return nullptr;

    void *ptr = memory + offset + padding;
    offset += size + padding;
    return ptr;
  }

  void Reset() { offset = 0; }

private:
  uint8_t *memory;
  size_t totalSize;
  size_t offset;
};

template <typename T> class PoolAllocator {
public:
  PoolAllocator(size_t count) {
    storage.resize(count);
    for (size_t i = 0; i < count; ++i) {
      freeList.push_back(&storage[i]);
    }
  }

  T *Allocate() {
    if (freeList.empty())
      return nullptr;
    T *ptr = freeList.back();
    freeList.pop_back();
    return ptr;
  }

  void Free(T *ptr) { freeList.push_back(ptr); }

private:
  std::vector<T> storage;
  std::vector<T *> freeList;
};

// Note: For GPU Memory, VMA (Vulkan Memory Allocator) is typically used for
// Vulkan. For OpenGL, we rely on glBufferData or glBufferStorage.

} // namespace AG
