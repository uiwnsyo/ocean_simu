#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

namespace AG {
    using float2 = glm::vec2;
    using float3 = glm::vec3;
    using float4 = glm::vec4;
    using int2   = glm::ivec2;
    using int3   = glm::ivec3;
    using int4   = glm::ivec4;
    using uint   = uint32_t;
    
    using float4x4 = glm::mat4;
    using quat     = glm::quat;

    // Helper functions
    inline float4x4 Perspective(float fov, float aspect, float near, float far) {
        return glm::perspective(fov, aspect, near, far);
    }

    inline float4x4 LookAt(const float3& eye, const float3& center, const float3& up) {
        return glm::lookAt(eye, center, up);
    }
}
