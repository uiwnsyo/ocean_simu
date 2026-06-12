#pragma once
#include <string>
#include <map>
#include "GraphicsHAL.h"

namespace AG {

class GpuProfiler {
public:
    struct Timestamp {
        uint32_t startQuery;
        uint32_t endQuery;
        float elapsedMs;
    };

    void BeginPass(const std::string& name) {
        currentPass = name;
        // In GL: glGenQueries, glQueryCounter(startQuery, GL_TIMESTAMP)
    }

    void EndPass() {
        // In GL: glQueryCounter(endQuery, GL_TIMESTAMP)
        // Resolve results at end of frame or N frames later
    }

    void Resolve() {
        // Fetch results and update elapsedMs
    }

    const std::map<std::string, Timestamp>& GetResults() const { return results; }

private:
    std::string currentPass;
    std::map<std::string, Timestamp> results;
};

} // namespace AG
