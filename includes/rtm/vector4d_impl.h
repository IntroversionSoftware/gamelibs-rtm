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

	//////////////////////////////////////////////////////////////////////////
	// Returns per component the sine and cosine of the input angle.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK void RTM_SIMD_CALL vector_sincos(vector4d_arg0 input, vector4d& out_sine, vector4d& out_cosine) RTM_NO_EXCEPT
	{
		const vector4d x = scalar_sincos(vector_get_x_as_scalar(input));
		const vector4d y = scalar_sincos(vector_get_y_as_scalar(input));
		const vector4d z = scalar_sincos(vector_get_z_as_scalar(input));
		const vector4d w = scalar_sincos(vector_get_w_as_scalar(input));

		const vector4d cos_xy = vector_mix<mix4::y, mix4::b, mix4::y, mix4::b>(x, y);
		const vector4d cos_zw = vector_mix<mix4::y, mix4::b, mix4::y, mix4::b>(z, w);
		out_cosine = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(cos_xy, cos_zw);

		const vector4d sin_xy = vector_mix<mix4::x, mix4::a, mix4::x, mix4::a>(x, y);
		const vector4d sin_zw = vector_mix<mix4::x, mix4::a, mix4::x, mix4::a>(z, w);
		out_sine = vector_mix<mix4::x, mix4::y, mix4::a, mix4::b>(sin_xy, sin_zw);
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
