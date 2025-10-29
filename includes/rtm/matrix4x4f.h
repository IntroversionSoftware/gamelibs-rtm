#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
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

#include "rtm/macros.h"
#include "rtm/math.h"
#include "rtm/matrix3x3f.h"
#include "rtm/vector4f.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/matrix_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis pointing in the forward direction of the default coordinate system (Z+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL matrix_get_coord_forward(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		return input.z_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis pointing in the up direction of the default coordinate system (Y+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL matrix_get_coord_up(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		return input.y_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis pointing in the cross direction of the default coordinate system (X+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL matrix_get_coord_cross(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		return input.x_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis holding the position of the default coordinate system (W+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL matrix_get_coord_position(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		return input.w_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 4x4 matrix axis.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4f RTM_SIMD_CALL matrix_get_axis(matrix4x4f_arg0 input, axis4 axis) RTM_NO_EXCEPT
	{
		return axis == axis4::x ? input.x_axis : (axis == axis4::y ? input.y_axis : (axis == axis4::z ? input.z_axis : input.w_axis));
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a new 4x4 matrix where the specified axis has been replaced on the input matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix4x4f RTM_SIMD_CALL matrix_set_axis(matrix4x4f_arg0 input, vector4f_arg5 axis_value, axis4 axis) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis4::x:	return matrix4x4f{ axis_value, input.y_axis, input.z_axis, input.w_axis };
			case axis4::y:	return matrix4x4f{ input.x_axis, axis_value, input.z_axis, input.w_axis };
			case axis4::z:	return matrix4x4f{ input.x_axis, input.y_axis, axis_value, input.w_axis };
			case axis4::w:	return matrix4x4f{ input.x_axis, input.y_axis, input.z_axis, axis_value };
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 4x4 matrix component from the specified axis.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline rtm_impl::vector4f_vector_get_component RTM_SIMD_CALL matrix_get_component(matrix4x4f_arg0 input, axis4 axis, component4 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis4::x:	return vector_get_component(input.x_axis, component);
			case axis4::y:	return vector_get_component(input.y_axis, component);
			case axis4::z:	return vector_get_component(input.z_axis, component);
			case axis4::w:	return vector_get_component(input.w_axis, component);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 4x4 matrix component from the specified axis.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline scalarf RTM_SIMD_CALL matrix_get_component_as_scalar(matrix4x4f_arg0 input, axis4 axis, component4 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis4::x:	return vector_get_component_as_scalar(input.x_axis, component);
			case axis4::y:	return vector_get_component_as_scalar(input.y_axis, component);
			case axis4::z:	return vector_get_component_as_scalar(input.z_axis, component);
			case axis4::w:	return vector_get_component_as_scalar(input.w_axis, component);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a new 4x4 matrix where the specified axis/component has been replaced on the input matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix4x4f RTM_SIMD_CALL matrix_set_component(matrix4x4f_arg0 input, float component_value, axis4 axis, component4 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis4::x:	return matrix4x4f{ vector_set_component(input.x_axis, component_value, component), input.y_axis, input.z_axis, input.w_axis };
			case axis4::y:	return matrix4x4f{ input.x_axis, vector_set_component(input.y_axis, component_value, component), input.z_axis, input.w_axis };
			case axis4::z:	return matrix4x4f{ input.x_axis, input.y_axis, vector_set_component(input.z_axis, component_value, component), input.w_axis };
			case axis4::w:	return matrix4x4f{ input.x_axis, input.y_axis, input.z_axis, vector_set_component(input.w_axis, component_value, component) };
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns a new 4x4 matrix where the specified axis/component has been replaced on the input matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix4x4f RTM_SIMD_CALL matrix_set_component(matrix4x4f_arg0 input, scalarf_arg4 component_value, axis4 axis, component4 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis4::x:	return matrix4x4f{ vector_set_component(input.x_axis, component_value, component), input.y_axis, input.z_axis, input.w_axis };
			case axis4::y:	return matrix4x4f{ input.x_axis, vector_set_component(input.y_axis, component_value, component), input.z_axis, input.w_axis };
			case axis4::z:	return matrix4x4f{ input.x_axis, input.y_axis, vector_set_component(input.z_axis, component_value, component), input.w_axis };
			case axis4::w:	return matrix4x4f{ input.x_axis, input.y_axis, input.z_axis, vector_set_component(input.w_axis, component_value, component) };
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two 4x4 matrices.
	// Multiplication order is as follow: local_to_world = matrix_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix4x4f RTM_SIMD_CALL matrix_mul(matrix4x4f_arg0 lhs, matrix4x4f_arg1 rhs) RTM_NO_EXCEPT
	{
		vector4f tmp = vector_mul(vector_dup_x(lhs.x_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.x_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.x_axis), rhs.z_axis, tmp);
		tmp = vector_mul_add(vector_dup_w(lhs.x_axis), rhs.w_axis, tmp);
		vector4f x_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.y_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.y_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.y_axis), rhs.z_axis, tmp);
		tmp = vector_mul_add(vector_dup_w(lhs.y_axis), rhs.w_axis, tmp);
		vector4f y_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.z_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.z_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.z_axis), rhs.z_axis, tmp);
		tmp = vector_mul_add(vector_dup_w(lhs.z_axis), rhs.w_axis, tmp);
		vector4f z_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.w_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.w_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.w_axis), rhs.z_axis, tmp);
		tmp = vector_mul_add(vector_dup_w(lhs.w_axis), rhs.w_axis, tmp);
		vector4f w_axis = tmp;

		return matrix4x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a 4x4 matrix and a 4D vector.
	// Multiplication order is as follow: world_position = matrix_mul(local_position, local_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4f RTM_SIMD_CALL matrix_mul_vector(vector4f_arg0 vec4, matrix4x4f_arg0 mtx) RTM_NO_EXCEPT
	{
		vector4f tmp;

		tmp = vector_mul(vector_dup_x(vec4), mtx.x_axis);
		tmp = vector_mul_add(vector_dup_y(vec4), mtx.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(vec4), mtx.z_axis, tmp);
		tmp = vector_mul_add(vector_dup_w(vec4), mtx.w_axis, tmp);

		return tmp;
	}

	//////////////////////////////////////////////////////////////////////////
	// Transposes a 4x4 matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE matrix4x4f RTM_SIMD_CALL matrix_transpose(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		vector4f x_axis;
		vector4f y_axis;
		vector4f z_axis;
		vector4f w_axis;
		RTM_MATRIXF_TRANSPOSE_4X4(input.x_axis, input.y_axis, input.z_axis, input.w_axis, x_axis, y_axis, z_axis, w_axis);
		return matrix4x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 4x4 matrix.
	// If the input matrix is not invertible, the result is undefined.
	// For a safe alternative, supply a fallback value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK matrix4x4f RTM_SIMD_CALL matrix_inverse(matrix4x4f_arg0 input) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 4x4 matrix.
	// If the input matrix has a determinant whose absolute value is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK matrix4x4f RTM_SIMD_CALL matrix_inverse(matrix4x4f_arg0 input, matrix4x4f_arg1 fallback, float threshold = 1.0E-8F) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Returns the determinant of the input 4x4 matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK scalarf RTM_SIMD_CALL matrix_determinant(matrix4x4f_arg0 input) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Returns the minor of the input 4x4 matrix.
	// See: https://en.wikipedia.org/wiki/Minor_(linear_algebra)
	// The minor is the determinant of the sub-matrix input when the specified
	// row and column are removed.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK scalarf RTM_SIMD_CALL matrix_minor(matrix4x4f_arg0 input, axis4 row, axis4 column) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Returns the cofactor matrix of the input 4x4 matrix.
	// See: https://en.wikipedia.org/wiki/Minor_(linear_algebra)#Cofactor_expansion_of_the_determinant
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix4x4f RTM_SIMD_CALL matrix_cofactor(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		const scalarf minor_xx = matrix_minor(input, axis4::x, axis4::x);
		const scalarf minor_xy = matrix_minor(input, axis4::x, axis4::y);
		const scalarf minor_xz = matrix_minor(input, axis4::x, axis4::z);
		const scalarf minor_xw = matrix_minor(input, axis4::x, axis4::w);

		const scalarf minor_yx = matrix_minor(input, axis4::y, axis4::x);
		const scalarf minor_yy = matrix_minor(input, axis4::y, axis4::y);
		const scalarf minor_yz = matrix_minor(input, axis4::y, axis4::z);
		const scalarf minor_yw = matrix_minor(input, axis4::y, axis4::w);

		const scalarf minor_zx = matrix_minor(input, axis4::z, axis4::x);
		const scalarf minor_zy = matrix_minor(input, axis4::z, axis4::y);
		const scalarf minor_zz = matrix_minor(input, axis4::z, axis4::z);
		const scalarf minor_zw = matrix_minor(input, axis4::z, axis4::w);

		const scalarf minor_wx = matrix_minor(input, axis4::w, axis4::x);
		const scalarf minor_wy = matrix_minor(input, axis4::w, axis4::y);
		const scalarf minor_wz = matrix_minor(input, axis4::w, axis4::z);
		const scalarf minor_ww = matrix_minor(input, axis4::w, axis4::w);

		const vector4f xz_axis_signs = vector_set(1.0F, -1.0F, 1.0F, -1.0F);
		const vector4f yw_axis_signs = vector_set(-1.0F, 1.0F, -1.0F, 1.0F);

		const vector4f x_axis = vector_mul(vector_set(minor_xx, minor_xy, minor_xz, minor_xw), xz_axis_signs);
		const vector4f y_axis = vector_mul(vector_set(minor_yx, minor_yy, minor_yz, minor_yw), yw_axis_signs);
		const vector4f z_axis = vector_mul(vector_set(minor_zx, minor_zy, minor_zz, minor_zw), xz_axis_signs);
		const vector4f w_axis = vector_mul(vector_set(minor_wx, minor_wy, minor_wz, minor_ww), yw_axis_signs);
		return matrix4x4f{ x_axis, y_axis, z_axis, w_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the adjugate of the input matrix.
	// See: https://en.wikipedia.org/wiki/Adjugate_matrix
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix4x4f RTM_SIMD_CALL matrix_adjugate(matrix4x4f_arg0 input) RTM_NO_EXCEPT
	{
		return matrix_transpose(matrix_cofactor(input));
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
