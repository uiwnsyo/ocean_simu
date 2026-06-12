#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

namespace AG {

struct ShaderSource {
    std::string path;
    std::string code;
    std::vector<uint32_t> spirv;
};

class ShaderCompiler {
public:
    static bool CompileGLSLToSPIRV(ShaderSource& source) {
        // Wrapper for glslang or shaderc
        std::cout << "Compiling " << source.path << " to SPIR-V...\n";
        // Logic to call glslangValidator -V
        return true;
    }

    static bool CrossCompileToGLSL(const std::vector<uint32_t>& spirv, std::string& outGlsl) {
        // Wrapper for SPIRV-Cross
        std::cout << "Cross-compiling SPIR-V to OpenGL GLSL 4.6...\n";
        return true;
    }
};

class ShaderHotReloader {
public:
    void AddShader(const std::string& path) {
        lastModified[path] = std::filesystem::last_write_time(path);
    }

    void Update() {
        for (auto& [path, time] : lastModified) {
            auto current = std::filesystem::last_write_time(path);
            if (current > time) {
                std::cout << "Shader changed: " << path << ". Reloading...\n";
                time = current;
                // Re-compile logic
            }
        }
    }

private:
    std::unordered_map<std::string, std::filesystem::file_time_type> lastModified;
};

} // namespace AG
