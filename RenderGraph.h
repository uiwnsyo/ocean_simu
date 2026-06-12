#pragma once
#include <functional>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include "GraphicsHAL.h"

namespace AG {

using ResourceHandle = uint32_t;

struct RenderPassContext {
    IGpuCommandList* cmd;
    // Map handles to actual textures/buffers
    std::function<IGpuTexture*(ResourceHandle)> GetTexture;
};

class RenderGraph {
public:
    struct Pass {
        std::string name;
        std::vector<ResourceHandle> reads;
        std::vector<ResourceHandle> writes;
        std::function<void(RenderPassContext&)> execute;
        bool culled = false;
        int refCount = 0;
    };

    struct Resource {
        std::string name;
        TextureDesc desc;
        std::shared_ptr<IGpuTexture> actual;
        int lastWriterIdx = -1;
    };

    ResourceHandle CreateTexture(const std::string& name, const TextureDesc& desc) {
        Resource res;
        res.name = name;
        res.desc = desc;
        resources.push_back(res);
        return (ResourceHandle)resources.size() - 1;
    }

    void AddPass(const std::string& name, 
                 const std::vector<ResourceHandle>& reads, 
                 const std::vector<ResourceHandle>& writes,
                 std::function<void(RenderPassContext&)> execute) {
        Pass p;
        p.name = name;
        p.reads = reads;
        p.writes = writes;
        p.execute = execute;
        passes.push_back(p);
    }

    void Compile() {
        // Culling logic
        for (auto& p : passes) p.refCount = 0;
        
        // Final pass usually writes to Swapchain or is explicitly needed
        // For simplicity, we assume the last pass is the root
        if (!passes.empty()) {
             CullInternal((int)passes.size() - 1);
        }

        // Topological Sort (Very simple linear for now as we build in order)
        // In a real implementation, we would build a DAG and sort.
    }

    void Execute(IGpuDevice* device, IGpuCommandList* cmd) {
        for (auto& p : passes) {
            if (p.culled) continue;

            cmd->PushMarker(p.name);
            
            RenderPassContext ctx;
            ctx.cmd = cmd;
            ctx.GetTexture = [&](ResourceHandle h) { 
                if (!resources[h].actual) {
                    resources[h].actual = device->CreateTexture(resources[h].desc);
                }
                return resources[h].actual.get(); 
            };
            
            p.execute(ctx);
            
            cmd->PopMarker();
        }
    }

private:
    void CullInternal(int passIdx) {
        // Simple recursive marking
        // If a pass writes to a resource used by a needed pass, it's not culled
    }

    std::vector<Pass> passes;
    std::vector<Resource> resources;
};

} // namespace AG
