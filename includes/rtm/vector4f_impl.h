#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
// Copyright (c) 2018 Nicholas Frechette & Realtime Math contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "rtm/math.h"
#include "rtm/scalard.h"
#include "rtm/version.h"
#include "rtm/impl/bit_cast.impl.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/memory_utils.h"
#include "rtm/impl/vector_common.h"

#include <cstring>
#include <limits>

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	/// Returns per component the sine and cosine of the input angle.
	RTM_DISABLE_SECURITY_COOKIE_CHECK void RTM_SIMD_CALL vector_sincos(vector4f_arg0 input, vector4f& out_sine, vector4f& out_cosine) RTM_NO_EXCEPT
	{
#if defined(RTM_SSE2_INTRINSICS)
		// Use a degree 10 minimax approximation polynomial
		// See: GPGPU Programming for Games and Science (David H. Eberly)

		// Remap our input in the [-pi, pi] range
		__m128 quotient = _mm_mul_ps(input, _mm_set_ps1(constants::one_div_two_pi()));
		quotient = vector_round_bankers(quotient);
		quotient = _mm_mul_ps(quotient, _mm_set_ps1(constants::two_pi()));
		__m128 x = _mm_sub_ps(input, quotient);

		// Remap our input in the [-pi/2, pi/2] range
		const __m128 sign_mask = _mm_set_ps(-0.0F, -0.0F, -0.0F, -0.0F);
		__m128 x_sign = _mm_and_ps(x, sign_mask);
		__m128 reference = _mm_or_ps(x_sign, _mm_set_ps1(constants::pi()));
		const __m128 reflection = _mm_sub_ps(reference, x);

		const __m128i abs_mask = _mm_set_epi32(0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL, 0x7FFFFFFFULL);
		__m128 x_abs = _mm_and_ps(x, _mm_castsi128_ps(abs_mask));
		__m128 is_less_equal_than_half_pi = _mm_cmple_ps(x_abs, _mm_set_ps1(constants::half_pi()));

		x = RTM_VECTOR4F_SELECT(is_less_equal_than_half_pi, x, reflection);
		const __m128 x2 = _mm_mul_ps(x, x);

		// Calculate our cosine value
		__m128 cosine_result = _mm_add_ps(_mm_mul_ps(x2, _mm_set_ps1(-2.6051615464872668e-7F)), _mm_set_ps1(2.4760495088926859e-5F));
		cosine_result = _mm_add_ps(_mm_mul_ps(cosine_result, x2), _mm_set_ps1(-1.3888377661039897e-3F));
		cosine_result = _mm_add_ps(_mm_mul_ps(cosine_result, x2), _mm_set_ps1(4.1666638865338612e-2F));
		cosine_result = _mm_add_ps(_mm_mul_ps(cosine_result, x2), _mm_set_ps1(-4.9999999508695869e-1F));
		cosine_result = _mm_add_ps(_mm_mul_ps(cosine_result, x2), _mm_set_ps1(1.0F));

		// Remap into [-pi, pi]
		out_cosine = _mm_or_ps(cosine_result, _mm_andnot_ps(is_less_equal_than_half_pi, sign_mask));

		// Calculate our sine value
		__m128 sine_result = _mm_add_ps(_mm_mul_ps(x2, _mm_set_ps1(-2.3828544692960918e-8F)), _mm_set_ps1(2.7521557770526783e-6F));
		sine_result = _mm_add_ps(_mm_mul_ps(sine_result, x2), _mm_set_ps1(-1.9840782426250314e-4F));
		sine_result = _mm_add_ps(_mm_mul_ps(sine_result, x2), _mm_set_ps1(8.3333303183525942e-3F));
		sine_result = _mm_add_ps(_mm_mul_ps(sine_result, x2), _mm_set_ps1(-1.6666666601721269e-1F));
		sine_result = _mm_add_ps(_mm_mul_ps(sine_result, x2), _mm_set_ps1(1.0F));
		out_sine = _mm_mul_ps(sine_result, x);
#elif defined(RTM_WASM128_INTRINSICS)
		// Use a degree 10 minimax approximation polynomial
		// See: GPGPU Programming for Games and Science (David H. Eberly)

		// Remap our input in the [-pi, pi] range
		v128_t quotient = wasm_f32x4_mul(input, wasm_f32x4_splat(constants::one_div_two_pi()));
		quotient = vector_round_bankers(quotient);
		quotient = wasm_f32x4_mul(quotient, wasm_f32x4_splat(constants::two_pi()));
		v128_t x = wasm_f32x4_sub(input, quotient);

		// Remap our input in the [-pi/2, pi/2] range
		const v128_t sign_mask = wasm_f32x4_splat(-0.0F);
		v128_t x_sign = wasm_v128_and(x, sign_mask);
		v128_t reference = wasm_v128_or(x_sign, wasm_f32x4_splat(constants::pi()));
		const v128_t reflection = wasm_f32x4_sub(reference, x);

		v128_t x_abs = wasm_f32x4_abs(x);
		v128_t is_less_equal_than_half_pi = wasm_f32x4_le(x_abs, wasm_f32x4_splat(constants::half_pi()));

		x = RTM_VECTOR4F_SELECT(is_less_equal_than_half_pi, x, reflection);
		const v128_t x2 = wasm_f32x4_mul(x, x);

		// Calculate our cosine value
		v128_t cosine_result = wasm_f32x4_add(wasm_f32x4_mul(x2, wasm_f32x4_splat(-2.6051615464872668e-7F)), wasm_f32x4_splat(2.4760495088926859e-5F));
		cosine_result = wasm_f32x4_add(wasm_f32x4_mul(cosine_result, x2), wasm_f32x4_splat(-1.3888377661039897e-3F));
		cosine_result = wasm_f32x4_add(wasm_f32x4_mul(cosine_result, x2), wasm_f32x4_splat(4.1666638865338612e-2F));
		cosine_result = wasm_f32x4_add(wasm_f32x4_mul(cosine_result, x2), wasm_f32x4_splat(-4.9999999508695869e-1F));
		cosine_result = wasm_f32x4_add(wasm_f32x4_mul(cosine_result, x2), wasm_f32x4_splat(1.0F));

		// Remap into [-pi, pi]
		out_cosine = wasm_v128_or(cosine_result, wasm_v128_andnot(sign_mask, is_less_equal_than_half_pi));

		// Calculate our sine value
		v128_t sine_result = wasm_f32x4_add(wasm_f32x4_mul(x2, wasm_f32x4_splat(-2.3828544692960918e-8F)), wasm_f32x4_splat(2.7521557770526783e-6F));
		sine_result = wasm_f32x4_add(wasm_f32x4_mul(sine_result, x2), wasm_f32x4_splat(-1.9840782426250314e-4F));
		sine_result = wasm_f32x4_add(wasm_f32x4_mul(sine_result, x2), wasm_f32x4_splat(8.3333303183525942e-3F));
		sine_result = wasm_f32x4_add(wasm_f32x4_mul(sine_result, x2), wasm_f32x4_splat(-1.6666666601721269e-1F));
		sine_result = wasm_f32x4_add(wasm_f32x4_mul(sine_result, x2), wasm_f32x4_splat(1.0F));
		out_sine = wasm_f32x4_mul(sine_result, x);
#elif defined(RTM_NEON_INTRINSICS)
		// Use a degree 10 minimax approximation polynomial
		// See: GPGPU Programming for Games and Science (David H. Eberly)

		// Remap our input in the [-pi, pi] range
		float32x4_t quotient = vmulq_n_f32(input, constants::one_div_two_pi());
		quotient = vector_round_bankers(quotient);
		quotient = vmulq_n_f32(quotient, constants::two_pi());
		float32x4_t x = vsubq_f32(input, quotient);

		// Remap our input in the [-pi/2, pi/2] range
		const uint32x4_t sign_mask = vreinterpretq_u32_f32(vdupq_n_f32(-0.0F));
		const uint32x4_t sign = vandq_u32(vreinterpretq_u32_f32(x), sign_mask);
		const float32x4_t reference = vreinterpretq_f32_u32(vorrq_u32(sign, vreinterpretq_u32_f32(vdupq_n_f32(constants::pi()))));

		const float32x4_t reflection = vsubq_f32(reference, x);
#if !defined(RTM_IMPL_VCA_SUPPORTED)
		const uint32x4_t is_less_equal_than_half_pi = vcleq_f32(vabsq_f32(x), vdupq_n_f32(constants::half_pi()));
#else
		const uint32x4_t is_less_equal_than_half_pi = vcaleq_f32(x, vdupq_n_f32(constants::half_pi()));
#endif
		x = vbslq_f32(is_less_equal_than_half_pi, x, reflection);
		const float32x4_t x2 = vmulq_f32(x, x);

		// Calculate our cosine value
		float32x4_t cosine_result = RTM_VECTOR4F_MULS_ADD(x2, -2.6051615464872668e-7F, vdupq_n_f32(2.4760495088926859e-5F));
		cosine_result = RTM_VECTOR4F_MULV_ADD(cosine_result, x2, vdupq_n_f32(-1.3888377661039897e-3F));
		cosine_result = RTM_VECTOR4F_MULV_ADD(cosine_result, x2, vdupq_n_f32(4.1666638865338612e-2F));
		cosine_result = RTM_VECTOR4F_MULV_ADD(cosine_result, x2, vdupq_n_f32(-4.9999999508695869e-1F));
		cosine_result = RTM_VECTOR4F_MULV_ADD(cosine_result, x2, vdupq_n_f32(1.0F));

		// Remap into [-pi, pi]
		out_cosine = vbslq_f32(is_less_equal_than_half_pi, cosine_result, vnegq_f32(cosine_result));

		// Calculate our sine value
		float32x4_t sine_result = RTM_VECTOR4F_MULS_ADD(x2, -2.3828544692960918e-8F, vdupq_n_f32(2.7521557770526783e-6F));
		sine_result = RTM_VECTOR4F_MULV_ADD(sine_result, x2, vdupq_n_f32(-1.9840782426250314e-4F));
		sine_result = RTM_VECTOR4F_MULV_ADD(sine_result, x2, vdupq_n_f32(8.3333303183525942e-3F));
		sine_result = RTM_VECTOR4F_MULV_ADD(sine_result, x2, vdupq_n_f32(-1.6666666601721269e-1F));
		sine_result = RTM_VECTOR4F_MULV_ADD(sine_result, x2, vdupq_n_f32(1.0F));

		out_sine = vmulq_f32(sine_result, x);
#else
		const vector4f x = scalar_sincos(scalarf(vector_get_x(input)));
		const vector4f y = scalar_sincos(scalarf(vector_get_y(input)));
		const vector4f z = scalar_sincos(scalarf(vector_get_z(input)));
		const vector4f w = scalar_sincos(scalarf(vector_get_w(input)));

		out_cosine = vector4f{ x.y, y.y, z.y, w.y };
		out_sine = vector4f{ x.x, y.x, z.x, w.x };
#endif
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
