/**
 * @file OceanSimulator.cpp
 * @brief Phillips 스펙트럼 FFT 오션 시뮬레이션 구현.
 */

#include "OceanSimulator.h"
#include <random>
#include <cmath>
#include <cassert>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace AG {

// ============================================================================
// Constructor
// ============================================================================

OceanSimulator::OceanSimulator(const OceanParameters& params)
    : m_params(params)
{
    SharedSimulationBuffer::Get().Initialize(params.resolution, params.resolution);
    InitSpectrum();
}

// ============================================================================
// SetParameters
// ============================================================================

void OceanSimulator::SetParameters(const OceanParameters& params)
{
    m_params = params;
    SharedSimulationBuffer::Get().Initialize(params.resolution, params.resolution);
    InitSpectrum();
}

// ============================================================================
// InitSpectrum — Phillips H₀(k)
// ============================================================================

void OceanSimulator::InitSpectrum()
{
    const uint32_t n = N();
    m_h0.resize(n * n);
    m_h0conj.resize(n * n);
    m_omega.resize(n * n);
    m_hktX.resize(n * n);
    m_hktY.resize(n * n);
    m_hktZ.resize(n * n);
    m_displX.resize(n * n, 0.0f);
    m_displY.resize(n * n, 0.0f);
    m_displZ.resize(n * n, 0.0f);
    m_foam.resize(n * n, 0.0f);

    std::mt19937 rng(42);
    std::normal_distribution<float> gauss(0.0f, 1.0f);

    const float L     = m_params.patchSize;
    const float A     = m_params.amplitude;
    const float V     = m_params.windSpeed;
    const float windR = glm::radians(m_params.windDirDeg);
    float2 windDir    = float2(std::cos(windR), std::sin(windR));
    float  windL      = V * V / G(); // L = V²/g

    for (uint32_t iz = 0; iz < n; ++iz) {
        for (uint32_t ix = 0; ix < n; ++ix) {
            // 파수 벡터 k
            float kx = (2.0f * (float)M_PI * (float)((int)ix - (int)n/2)) / L;
            float kz = (2.0f * (float)M_PI * (float)((int)iz - (int)n/2)) / L;
            float kLen = std::sqrt(kx*kx + kz*kz);

            uint32_t idx = iz * n + ix;

            if (kLen < 1e-6f) {
                m_h0[idx]     = Complex(0.0f, 0.0f);
                m_h0conj[idx] = Complex(0.0f, 0.0f);
                m_omega[idx]  = 0.0f;
                continue;
            }

            float2 kHat  = float2(kx, kz) / kLen;
            float  kDotW = glm::dot(kHat, windDir);

            // Phillips 스펙트럼
            // P(k) = A * exp(-1/(kL)²) / k⁴ * |k̂·ŵ|²
            float k2     = kLen * kLen;
            float k4     = k2 * k2;
            float phillip = A * std::exp(-1.0f / (k2 * windL * windL)) / k4
                          * (kDotW * kDotW);

            // 역풍 제거 (약한 파도)
            if (kDotW < 0.0f) phillip *= 0.07f;

            // 최소 파장 억제
            float smallWave = std::exp(-k2 * (L / (float)n) * (L / (float)n));
            phillip *= smallWave;

            float amp = std::sqrt(phillip * 0.5f);

            // Gaussian 랜덤 진폭
            m_h0[idx]     = Complex(gauss(rng), gauss(rng)) * amp;
            // conj(H₀(-k)) — -k는 wrap-around 인덱스
            uint32_t mx = (n - ix) % n;
            uint32_t mz = (n - iz) % n;
            // 나중에 Update()에서 직접 사용하므로 현재는 복소 켤레만 저장
            m_h0conj[idx] = std::conj(m_h0[mz * n + mx]);

            // 분산 관계
            m_omega[idx] = Dispersion(kLen);
        }
    }
}

float OceanSimulator::Dispersion(float kLen) const {
    return std::sqrt(G() * kLen); // 심해 근사: ω = sqrt(g|k|)
}

// ============================================================================
// Update — 시간 진화 + IFFT
// ============================================================================

void OceanSimulator::Update(float t)
{
    const uint32_t n = N();
    const float    L = m_params.patchSize;

    for (uint32_t iz = 0; iz < n; ++iz) {
        for (uint32_t ix = 0; ix < n; ++ix) {
            uint32_t idx = iz * n + ix;
            float    w   = m_omega[idx];

            // H(k,t) = H₀(k)*exp(iωt) + conj(H₀(-k))*exp(-iωt)
            float cosWt = std::cos(w * t), sinWt = std::sin(w * t);
            Complex ewt (cosWt,  sinWt);
            Complex ewtn(cosWt, -sinWt);

            Complex hkt = m_h0[idx] * ewt + m_h0conj[idx] * ewtn;

            // 높이 스펙트럼
            m_hktY[idx] = hkt;

            // XZ 변위 (choppy waves): -i * (k/|k|) * H(k,t)
            float kx = (2.0f * (float)M_PI * (float)((int)ix - (int)n/2)) / L;
            float kz = (2.0f * (float)M_PI * (float)((int)iz - (int)n/2)) / L;
            float kLen = std::sqrt(kx*kx + kz*kz);

            if (kLen > 1e-6f) {
                Complex minusI(0.0f, -1.0f);
                m_hktX[idx] = minusI * Complex(kx / kLen, 0.0f) * hkt;
                m_hktZ[idx] = minusI * Complex(kz / kLen, 0.0f) * hkt;
            } else {
                m_hktX[idx] = Complex(0.0f);
                m_hktZ[idx] = Complex(0.0f);
            }
        }
    }

    // 2D IFFT 각 채널
    IFFT2D(m_hktY);
    IFFT2D(m_hktX);
    IFFT2D(m_hktZ);

    // 실수부 추출 + 부호 보정 (체커보드 역전)
    for (uint32_t iz = 0; iz < n; ++iz) {
        for (uint32_t ix = 0; ix < n; ++ix) {
            uint32_t idx  = iz * n + ix;
            float    sign = ((ix + iz) & 1) ? -1.0f : 1.0f;
            m_displY[idx] = m_hktY[idx].real() * sign;
            m_displX[idx] = m_hktX[idx].real() * sign * m_params.choppiness;
            m_displZ[idx] = m_hktZ[idx].real() * sign * m_params.choppiness;
        }
    }

    // Jacobian 기반 거품 계산 (∂dx/∂x + ∂dz/∂z 발산)
    for (uint32_t iz = 0; iz < n; ++iz) {
        for (uint32_t ix = 0; ix < n; ++ix) {
            uint32_t ixp = (ix+1)%n, ixm = (ix+n-1)%n;
            uint32_t izp = (iz+1)%n, izm = (iz+n-1)%n;
            float ddx = (m_displX[iz*n+ixp] - m_displX[iz*n+ixm]) * 0.5f;
            float ddz = (m_displZ[izp*n+ix] - m_displZ[izm*n+ix]) * 0.5f;
            float J   = (1.0f + ddx) * (1.0f + ddz);
            m_foam[iz*n+ix] = glm::clamp(
                m_params.foamThreshold - J, 0.0f, 1.0f);
        }
    }

    WriteToSharedBuffer(t);
}

// ============================================================================
// WriteToSharedBuffer
// ============================================================================

void OceanSimulator::WriteToSharedBuffer(float t)
{
    const uint32_t n = N();
    OceanDisplacementSample* buf = SharedSimulationBuffer::Get().AcquireWrite();

    for (uint32_t iz = 0; iz < n; ++iz) {
        for (uint32_t ix = 0; ix < n; ++ix) {
            uint32_t idx = iz * n + ix;

            // 법선 = 중앙 차분 (Y 변위 기준)
            uint32_t ixp = (ix+1)%n, ixm = (ix+n-1)%n;
            uint32_t izp = (iz+1)%n, izm = (iz+n-1)%n;
            float dydx = (m_displY[iz*n+ixp] - m_displY[iz*n+ixm]) * 0.5f;
            float dydz = (m_displY[izp*n+ix] - m_displY[izm*n+ix]) * 0.5f;
            float3 normal = glm::normalize(float3(-dydx, 1.0f, -dydz));

            buf[idx].displacement = float3(m_displX[idx], m_displY[idx], m_displZ[idx]);
            buf[idx].normal       = normal;
            buf[idx].foam         = m_foam[idx];
            buf[idx]._pad         = 0.0f;
        }
    }

    SharedSimulationBuffer::Get().CommitWrite(t);
}

// ============================================================================
// SampleHeightAt
// ============================================================================

float OceanSimulator::SampleHeightAt(float worldX, float worldZ) const
{
    return SampleAt(worldX, worldZ).displacement.y;
}

OceanDisplacementSample OceanSimulator::SampleAt(float worldX, float worldZ) const
{
    const uint32_t n  = N();
    const float    L  = m_params.patchSize;

    // 격자 좌표 (래핑)
    float fx = std::fmod(worldX / L * n + n * 100, (float)n);
    float fz = std::fmod(worldZ / L * n + n * 100, (float)n);
    uint32_t ix0 = (uint32_t)fx % n, iz0 = (uint32_t)fz % n;
    uint32_t ix1 = (ix0+1) % n,      iz1 = (iz0+1) % n;
    float tx = fx - std::floor(fx), tz = fz - std::floor(fz);

    auto Lerp = [&](float a, float b, float t) { return a + (b-a)*t; };
    auto Sample = [&](uint32_t x, uint32_t z) {
        uint32_t i = z * n + x;
        return OceanDisplacementSample{
            float3(m_displX[i], m_displY[i], m_displZ[i]),
            float3(0,1,0), m_foam[i], 0.0f
        };
    };

    // Bilinear interpolation of Y (height only)
    float h00 = m_displY[iz0*n+ix0], h10 = m_displY[iz0*n+ix1];
    float h01 = m_displY[iz1*n+ix0], h11 = m_displY[iz1*n+ix1];
    float h   = Lerp(Lerp(h00,h10,tx), Lerp(h01,h11,tx), tz);

    OceanDisplacementSample s;
    s.displacement = float3(0.0f, h, 0.0f);
    s.normal       = float3(0,1,0);
    s.foam         = m_foam[iz0*n+ix0];
    s._pad         = 0.0f;
    return s;
}

// ============================================================================
// FFT1D — Cooley-Tukey DIF
// ============================================================================

void OceanSimulator::FFT1D(std::vector<Complex>& data, bool inverse)
{
    const uint32_t n = (uint32_t)data.size();
    assert((n & (n-1)) == 0 && "FFT size must be power of 2");

    // Bit-reversal permutation
    for (uint32_t i = 1, j = 0; i < n; ++i) {
        uint32_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[i], data[j]);
    }

    // FFT butterfly
    for (uint32_t len = 2; len <= n; len <<= 1) {
        double ang = 2.0 * M_PI / (double)len * (inverse ? 1.0 : -1.0);
        Complex wlen(std::cos(ang), std::sin(ang));
        for (uint32_t i = 0; i < n; i += len) {
            Complex w(1.0f, 0.0f);
            for (uint32_t j = 0; j < len/2; ++j) {
                Complex u = data[i+j];
                Complex v = data[i+j+len/2] * w;
                data[i+j]        = u + v;
                data[i+j+len/2]  = u - v;
                w *= wlen;
            }
        }
    }

    if (inverse) {
        for (auto& c : data) c /= (float)n;
    }
}

void OceanSimulator::IFFT2D(std::vector<Complex>& data)
{
    const uint32_t n = N();
    std::vector<Complex> row(n);

    // 행 방향 IFFT
    for (uint32_t iz = 0; iz < n; ++iz) {
        for (uint32_t ix = 0; ix < n; ++ix) row[ix] = data[iz*n+ix];
        FFT1D(row, true);
        for (uint32_t ix = 0; ix < n; ++ix) data[iz*n+ix] = row[ix];
    }

    // 열 방향 IFFT
    std::vector<Complex> col(n);
    for (uint32_t ix = 0; ix < n; ++ix) {
        for (uint32_t iz = 0; iz < n; ++iz) col[iz] = data[iz*n+ix];
        FFT1D(col, true);
        for (uint32_t iz = 0; iz < n; ++iz) data[iz*n+ix] = col[iz];
    }
}

} // namespace AG
