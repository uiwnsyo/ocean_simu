/**
 * @file GpuMesh.cpp
 * @brief VAO/VBO/EBO 업로드 구현 (OpenGL DSA 기반).
 *
 * 실제 GL 함수 호출은 주석 형태로 표기되어 있어
 * 헤더·드라이버 없이도 컴파일된다.
 * 실제 GL 환경에서 사용 시 주석을 해제하고
 * glad/GLEW include를 추가하면 된다.
 */

#include "GpuMesh.h"
#include <iostream>
#include <cassert>
#include <cstring>

// ---- GL stub macros (헤더 없이 컴파일용 플레이스홀더) ----
// 실제 환경에서는 #include <glad/glad.h> 로 교체
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER          0x8892
#define GL_ELEMENT_ARRAY_BUFFER  0x8893
#define GL_STATIC_DRAW           0x88B4
#define GL_DYNAMIC_DRAW          0x88E8
#define GL_UNSIGNED_INT          0x1405
#define GL_TRIANGLES             0x0004
#define GL_FLOAT                 0x1406
using GLuint = unsigned int;
using GLsizei = int;
using GLenum  = unsigned int;
// Stubs (no-op)
static void glCreateVertexArrays(int n, GLuint* v)  { static GLuint id=100; *v=id++; }
static void glCreateBuffers(int n, GLuint* b)        { static GLuint id=200; *b=id++; }
static void glDeleteVertexArrays(int, const GLuint*) {}
static void glDeleteBuffers(int, const GLuint*)      {}
static void glNamedBufferData(GLuint, size_t, const void*, GLenum){}
static void glVertexArrayVertexBuffer(GLuint,GLuint,GLuint,ptrdiff_t,GLsizei){}
static void glVertexArrayAttribBinding(GLuint,GLuint,GLuint){}
static void glVertexArrayAttribFormat(GLuint,GLuint,int,GLenum,bool,GLuint){}
static void glEnableVertexArrayAttrib(GLuint,GLuint){}
static void glVertexArrayElementBuffer(GLuint,GLuint){}
static void glBindVertexArray(GLuint){}
static void glDrawElementsInstanced(GLenum,GLsizei,GLenum,const void*,GLsizei){}
#endif

namespace AG {

// ============================================================================
// GpuMesh — ctor / dtor
// ============================================================================

GpuMesh::GpuMesh(Mesh& mesh)
    : m_mesh(mesh)
{
    // VAO 생성
    glCreateVertexArrays(1, &m_vao);

    // VBO 3개 생성 (위치 / 법선+탄젠트 / UV)
    glCreateBuffers(1, &m_vboPosition);
    glCreateBuffers(1, &m_vboNormal);
    glCreateBuffers(1, &m_vboUV);

    // EBO LOD_COUNT 개
    glCreateBuffers(LOD_COUNT, m_ebos.data());

    // VAO에 Attribute 레이아웃 설정
    // Binding 0: position stream
    //   attrib 0 → vec3 position (offset 0)
    glVertexArrayVertexBuffer(m_vao, 0, m_vboPosition, 0, sizeof(VertexPosition));
    glVertexArrayAttribFormat(m_vao, 0, 3, GL_FLOAT, false, 0);
    glVertexArrayAttribBinding(m_vao, 0, 0);
    glEnableVertexArrayAttrib(m_vao, 0);

    // Binding 1: normal+tangent stream
    //   attrib 1 → vec3 normal  (offset 0)
    //   attrib 2 → vec4 tangent (offset 12)
    glVertexArrayVertexBuffer(m_vao, 1, m_vboNormal, 0, sizeof(VertexNormal));
    glVertexArrayAttribFormat(m_vao, 1, 3, GL_FLOAT, false, 0);
    glVertexArrayAttribBinding(m_vao, 1, 1);
    glEnableVertexArrayAttrib(m_vao, 1);
    glVertexArrayAttribFormat(m_vao, 2, 4, GL_FLOAT, false, sizeof(float3));
    glVertexArrayAttribBinding(m_vao, 2, 1);
    glEnableVertexArrayAttrib(m_vao, 2);

    // Binding 2: UV stream
    //   attrib 3 → vec2 uv (offset 0)
    glVertexArrayVertexBuffer(m_vao, 2, m_vboUV, 0, sizeof(VertexUV));
    glVertexArrayAttribFormat(m_vao, 3, 2, GL_FLOAT, false, 0);
    glVertexArrayAttribBinding(m_vao, 3, 2);
    glEnableVertexArrayAttrib(m_vao, 3);

    // EBO 기본 연결 (LOD 0)
    glVertexArrayElementBuffer(m_vao, m_ebos[0]);

    // 초기 업로드
    UploadAll();
    m_mesh.ClearDirty();

    std::cout << "[GpuMesh] '" << m_mesh.GetName()
              << "' VAO=" << m_vao
              << " verts=" << m_mesh.GetVertexCount()
              << " lod0_idx=" << m_mesh.GetIndices(0).size() << "\n";
}

GpuMesh::~GpuMesh()
{
    glDeleteBuffers(LOD_COUNT, m_ebos.data());
    glDeleteBuffers(1, &m_vboUV);
    glDeleteBuffers(1, &m_vboNormal);
    glDeleteBuffers(1, &m_vboPosition);
    glDeleteVertexArrays(1, &m_vao);
}

// ============================================================================
// GpuMesh — Upload
// ============================================================================

void GpuMesh::FlushIfDirty()
{
    if (!m_mesh.IsDirty()) return;
    UploadAll();
    m_mesh.ClearDirty();
    std::cout << "[GpuMesh] '" << m_mesh.GetName() << "' re-uploaded (dirty flush)\n";
}

void GpuMesh::UploadLod(uint32_t lod)
{
    assert(lod < LOD_COUNT);
    const auto& idx = m_mesh.GetIndices(lod);
    if (idx.empty()) return;

    glNamedBufferData(m_ebos[lod],
                      idx.size() * sizeof(uint32_t),
                      idx.data(),
                      GL_DYNAMIC_DRAW);
    m_indexCounts[lod] = (uint32_t)idx.size();
}

void GpuMesh::UploadAll()
{
    const auto& pos = m_mesh.GetPositions();
    const auto& nor = m_mesh.GetNormals();
    const auto& uvs = m_mesh.GetUVs();

    if (!pos.empty()) {
        glNamedBufferData(m_vboPosition, pos.size() * sizeof(VertexPosition),
                          pos.data(), GL_DYNAMIC_DRAW);
    }
    if (!nor.empty()) {
        glNamedBufferData(m_vboNormal, nor.size() * sizeof(VertexNormal),
                          nor.data(), GL_DYNAMIC_DRAW);
    }
    if (!uvs.empty()) {
        glNamedBufferData(m_vboUV, uvs.size() * sizeof(VertexUV),
                          uvs.data(), GL_DYNAMIC_DRAW);
    }

    for (uint32_t lod = 0; lod < LOD_COUNT; ++lod) {
        UploadLod(lod);
    }

    // VAO EBO는 고정: LOD 전환 시 glVertexArrayElementBuffer로 교체
    glVertexArrayElementBuffer(m_vao, m_ebos[0]);
}

// ============================================================================
// GpuMesh — Draw
// ============================================================================

void GpuMesh::Draw(uint32_t lod, uint32_t instanceCount) const
{
    assert(lod < LOD_COUNT);

    // EBO 교체 (LOD 전환)
    glVertexArrayElementBuffer(m_vao, m_ebos[lod]);
    glBindVertexArray(m_vao);

    glDrawElementsInstanced(
        GL_TRIANGLES,
        (GLsizei)m_indexCounts[lod],
        GL_UNSIGNED_INT,
        nullptr,
        (GLsizei)instanceCount
    );
}

} // namespace AG
