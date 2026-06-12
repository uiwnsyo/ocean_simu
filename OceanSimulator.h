#pragma once
/**
 * @file OceanSimulator.h
 * @brief Phillips 스펙트럼 FFT 오션 시뮬레이션.
 *
 * GPU Compute Shader 기반. 256×256 그리드.
 * 출력: XYZ 변위 맵 + 법선 맵 → SharedSimulationBuffer에 기록.
 */

#include "AGMath.h"
#include "SharedSimulationBuffer.h"
#include <complex>
#include <vector>
#include <cstdint>

namespace AG {

// ============================================================================
// OceanParameters
// ============================================================================

/**
 * @brief 오션 시뮬레이션 파라미터.
 */
struct OceanParameters {
    float windSpeed     = 12.0f;     ///< 풍속 (m/s)
    float windDirDeg    = 0.0f;      ///< 풍향 (도, X축 기준)
    float amplitude     = 1.0f;      ///< Phillips 스펙트럼 진폭 스케일
    float patchSize     = 256.0f;    ///< 그리드 월드 크기 (m)
    float choppiness    = 1.0f;      ///< XZ 수평 변위 강도
    float foamThreshold = 1.2f;      ///< 거품 발생 Jacobian 임계값
    uint32_t resolution = 256;       ///< N×N 격자 크기 (2의 제곱수)
};

// ============================================================================
// OceanSimulator
// ============================================================================

/**
 * @brief FFT 오션 시뮬레이터.
 *
 * CPU 기반 시뮬레이션 루프 (GPU Compute Shader 연동 준비됨):
 *  1. InitSpectrum(): Phillips H₀(k) 초기화
 *  2. Update(t): H(k,t) 업데이트 → 역 FFT → 변위 추출
 *  3. WriteToSharedBuffer(): SharedSimulationBuffer에 기록
 *
 * GPU 연동: compute shader가 SSBO에 직접 쓰도록 SubmitGPUUpdate() 를 통해 확장 가능.
 */
class OceanSimulator {
public:
    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /**
     * @brief 파라미터를 설정하고 스펙트럼을 초기화한다.
     */
    explicit OceanSimulator(const OceanParameters& params = {});

    // ------------------------------------------------------------------
    // Simulation
    // ------------------------------------------------------------------

    /**
     * @brief 파라미터를 런타임에 변경하고 스펙트럼을 재초기화한다.
     */
    void SetParameters(const OceanParameters& params);

    /**
     * @brief 시뮬레이션을 시간 t 로 진행한다.
     *
     * 순서:
     *  1. H(k,t) = H₀(k)*exp(iωt) + conj(H₀(-k))*exp(-iωt)
     *  2. 2D IFFT X, Y, Z 변위 채널
     *  3. Jacobian으로 거품 계산
     *  4. 법선 = normalize(-∂Y/∂x, 1, -∂Y/∂z)
     *  5. WriteToSharedBuffer()
     *
     * @param t  시뮬레이션 절대 시간 (초)
     */
    void Update(float t);

    /**
     * @brief 결과를 SharedSimulationBuffer에 기록한다.
     */
    void WriteToSharedBuffer(float t);

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    const OceanParameters& GetParams()    const { return m_params; }
    uint32_t               GetResolution()const { return m_params.resolution; }

    /**
     * @brief 월드 좌표 (x,z)에서 Y 변위(파고)를 이중 선형 보간으로 반환한다.
     * @param worldX, worldZ  월드 공간 좌표
     */
    float SampleHeightAt(float worldX, float worldZ) const;

    /**
     * @brief 월드 좌표에서 XYZ 전체 변위 + 법선을 반환한다.
     */
    OceanDisplacementSample SampleAt(float worldX, float worldZ) const;

private:
    using Complex = std::complex<float>;

    // ------------------------------------------------------------------
    // Phillips Spectrum
    // ------------------------------------------------------------------

    /**
     * @brief Phillips 스펙트럼 초기 진폭 H₀(k)를 생성한다.
     *
     * H₀(k) = (A / k⁴) * exp(-1/(kL)²) * |k̂·ŵ|²
     * L = V²/g  (V=풍속, g=9.81)
     */
    void InitSpectrum();

    /**
     * @brief 딥-워터 분산 관계. ω(k) = sqrt(g * |k|)
     */
    float Dispersion(float kLen) const;

    // ------------------------------------------------------------------
    // FFT
    // ------------------------------------------------------------------

    /**
     * @brief Cooley-Tukey 제자리 FFT (1D).
     * @param data  복소수 배열 (크기 N, 2의 제곱수)
     * @param inverse  역변환 여부
     */
    static void FFT1D(std::vector<Complex>& data, bool inverse);

    /**
     * @brief 행→열 순으로 2D IFFT.
     */
    void IFFT2D(std::vector<Complex>& data);

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------

    OceanParameters m_params;

    uint32_t N()   const { return m_params.resolution; }
    float    G()   const { return 9.81f; }

    // k-도메인 데이터
    std::vector<Complex> m_h0;        ///< H₀(k)  N×N
    std::vector<Complex> m_h0conj;    ///< conj(H₀(-k))  N×N
    std::vector<float>   m_omega;     ///< ω(k)   N×N

    // 임시 작업 버퍼 (Update 당 재사용)
    std::vector<Complex> m_hktX;      ///< X 변위 스펙트럼
    std::vector<Complex> m_hktY;      ///< Y 변위 (높이) 스펙트럼
    std::vector<Complex> m_hktZ;      ///< Z 변위 스펙트럼

    // 결과 (공간 도메인, 실수)
    std::vector<float>   m_displX;    ///< X 수평 변위
    std::vector<float>   m_displY;    ///< Y 높이
    std::vector<float>   m_displZ;    ///< Z 수평 변위
    std::vector<float>   m_foam;      ///< 거품 강도
};

} // namespace AG
