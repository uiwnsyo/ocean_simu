#include <iostream>
#include "GraphicsHAL.h"
#include "OpenGLDevice.h"
#include "RenderGraph.h"
#include "ShaderCompiler.h"
#include "Memory.h"

using namespace AG;

int main() {
    // 1. Device Initialization
    auto device = std::make_unique<OpenGLDevice>();
    auto cmd = device->CreateCommandList();

    // 2. Render Graph Setup
    RenderGraph graph;

    // Declare Resources
    TextureDesc colorDesc = { 1920, 1080, TextureFormat::RGBA8_UNORM };
    ResourceHandle sceneColor = graph.CreateTexture("SceneColor", colorDesc);
    ResourceHandle finalOutput = graph.CreateTexture("FinalOutput", colorDesc);

    // Geometry Pass
    graph.AddPass("GeometryPass", 
        {},            // Reads
        {sceneColor},  // Writes
        [](RenderPassContext& ctx) {
            auto tex = ctx.GetTexture(0); // sceneColor
            ctx.cmd->Draw(3); // Triangle
        }
    );

    // Post Process Pass
    graph.AddPass("PostProcess",
        {sceneColor},  // Reads
        {finalOutput}, // Writes
        [](RenderPassContext& ctx) {
            auto input = ctx.GetTexture(0); // sceneColor
            auto output = ctx.GetTexture(1); // finalOutput
            ctx.cmd->Dispatch(1, 1, 1);
        }
    );

    // 3. Compile and Run
    std::cout << "Compiling Render Graph...\n";
    graph.Compile();

    std::cout << "Executing Render Graph...\n";
    cmd->Begin();
    graph.Execute(device.get(), cmd.get());
    cmd->End();

    device->Submit(cmd.get());
    
    std::cout << "\nDemo Completed Successfully.\n";
    return 0;
}
