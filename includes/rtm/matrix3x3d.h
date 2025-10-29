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

#include "rtm/math.h"
#include "rtm/vector4d.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/matrix_common.h"
#include "rtm/impl/matrix_affine_common.h"

RTM_IMPL_FILE_PRAGMA_PUSH

namespace rtm
{
	RTM_IMPL_VERSION_NAMESPACE_BEGIN

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis pointing in the forward direction of the default coordinate system (Z+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4d RTM_SIMD_CALL matrix_get_coord_forward(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return input.z_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis pointing in the up direction of the default coordinate system (Y+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4d RTM_SIMD_CALL matrix_get_coord_up(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return input.y_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the axis pointing in the cross direction of the default coordinate system (X+).
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4d RTM_SIMD_CALL matrix_get_coord_cross(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return input.x_axis;
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 3x3 matrix axis.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE constexpr vector4d RTM_SIMD_CALL matrix_get_axis(matrix3x3d_arg0 input, axis3 axis) RTM_NO_EXCEPT
	{
		return axis == axis3::x ? input.x_axis : (axis == axis3::y ? input.y_axis : input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a new 3x3 matrix where the specified axis has been replaced on the input matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_set_axis(matrix3x3d_arg0 input, vector4d_arg0 axis_value, axis3 axis) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis3::x:	return matrix3x3d{ axis_value, input.y_axis, input.z_axis };
			case axis3::y:	return matrix3x3d{ input.x_axis, axis_value, input.z_axis };
			case axis3::z:	return matrix3x3d{ input.x_axis, input.y_axis, axis_value };
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 3x3 matrix component from the specified axis.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline rtm_impl::vector4d_vector_get_component RTM_SIMD_CALL matrix_get_component(matrix3x3d_arg0 input, axis3 axis, component3 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis3::x:	return vector_get_component3(input.x_axis, component);
			case axis3::y:	return vector_get_component3(input.y_axis, component);
			case axis3::z:	return vector_get_component3(input.z_axis, component);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the desired 3x3 matrix component from the specified axis.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline scalard RTM_SIMD_CALL matrix_get_component_as_scalar(matrix3x3d_arg0 input, axis3 axis, component3 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis3::x:	return vector_get_component3_as_scalar(input.x_axis, component);
			case axis3::y:	return vector_get_component3_as_scalar(input.y_axis, component);
			case axis3::z:	return vector_get_component3_as_scalar(input.z_axis, component);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns a new 3x3 matrix where the specified axis/component has been replaced on the input matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_set_component(matrix3x3d_arg0 input, double component_value, axis3 axis, component3 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis3::x:	return matrix3x3d{ vector_set_component3(input.x_axis, component_value, component), input.y_axis, input.z_axis };
			case axis3::y:	return matrix3x3d{ input.x_axis, vector_set_component3(input.y_axis, component_value, component), input.z_axis };
			case axis3::z:	return matrix3x3d{ input.x_axis, input.y_axis, vector_set_component3(input.z_axis, component_value, component) };
		}
	}

#if defined(RTM_SSE2_INTRINSICS)
	//////////////////////////////////////////////////////////////////////////
	// Returns a new 3x3 matrix where the specified axis/component has been replaced on the input matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_set_component(matrix3x3d_arg0 input, scalard_arg7 component_value, axis3 axis, component3 component) RTM_NO_EXCEPT
	{
		switch (axis)
		{
			default:
			case axis3::x:	return matrix3x3d{ vector_set_component3(input.x_axis, component_value, component), input.y_axis, input.z_axis };
			case axis3::y:	return matrix3x3d{ input.x_axis, vector_set_component3(input.y_axis, component_value, component), input.z_axis };
			case axis3::z:	return matrix3x3d{ input.x_axis, input.y_axis, vector_set_component3(input.z_axis, component_value, component) };
		}
	}
#endif

	//////////////////////////////////////////////////////////////////////////
	// Converts a 3x3 matrix into a rotation quaternion.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline quatd RTM_SIMD_CALL quat_from_matrix(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return rtm_impl::quat_from_matrix(input.x_axis, input.y_axis, input.z_axis);
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies two 3x3 matrices.
	// Multiplication order is as follow: local_to_world = matrix_mul(local_to_object, object_to_world)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_mul(matrix3x3d_arg0 lhs, matrix3x3d_arg1 rhs) RTM_NO_EXCEPT
	{
		vector4d tmp = vector_mul(vector_dup_x(lhs.x_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.x_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.x_axis), rhs.z_axis, tmp);
		vector4d x_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.y_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.y_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.y_axis), rhs.z_axis, tmp);
		vector4d y_axis = tmp;

		tmp = vector_mul(vector_dup_x(lhs.z_axis), rhs.x_axis);
		tmp = vector_mul_add(vector_dup_y(lhs.z_axis), rhs.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(lhs.z_axis), rhs.z_axis, tmp);
		vector4d z_axis = tmp;

		return matrix3x3d{ x_axis, y_axis, z_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Multiplies a 3x3 matrix and a 3D vector.
	// Multiplication order is as follow: world_position = matrix_mul(local_vector, local_to_world)
	// Note: The proper way to transform a normal by a matrix with non-uniform scale
	// is to multiply the normal with the cofactor matrix.
	// See: https://github.com/graphitemaster/normals_revisited
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE vector4d RTM_SIMD_CALL matrix_mul_vector3(vector4d_arg0 vec3, matrix3x3d_argn mtx) RTM_NO_EXCEPT
	{
		vector4d tmp;

		tmp = vector_mul(vector_dup_x(vec3), mtx.x_axis);
		tmp = vector_mul_add(vector_dup_y(vec3), mtx.y_axis, tmp);
		tmp = vector_mul_add(vector_dup_z(vec3), mtx.z_axis, tmp);

		return tmp;
	}

	//////////////////////////////////////////////////////////////////////////
	// Transposes a 3x3 matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK RTM_FORCE_INLINE matrix3x3d RTM_SIMD_CALL matrix_transpose(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		vector4d x_axis;
		vector4d y_axis;
		vector4d z_axis;
		RTM_MATRIXD_TRANSPOSE_3X3(input.x_axis, input.y_axis, input.z_axis, x_axis, y_axis, z_axis);
		return matrix3x3d{ x_axis, y_axis, z_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 3x3 matrix.
	// If the input matrix is not invertible, the result is undefined.
	// For a safe alternative, supply a fallback value and a threshold.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK matrix3x3d RTM_SIMD_CALL matrix_inverse(matrix3x3d_arg0 input) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Inverses a 3x3 matrix.
	// If the input matrix has a determinant whose absolute value is below the supplied threshold, the
	// fall back value is returned instead.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK matrix3x3d RTM_SIMD_CALL matrix_inverse(matrix3x3d_arg0 input, matrix3x3d_arg1 fallback, double threshold = 1.0E-8) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Returns the determinant of the input 3x3 matrix.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK scalard RTM_SIMD_CALL matrix_determinant(matrix3x3d_arg0 input) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Returns the minor of the input 3x3 matrix.
	// See: https://en.wikipedia.org/wiki/Minor_(linear_algebra)
	// The minor is the determinant of the sub-matrix input when the specified
	// row and column are removed.
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK scalard RTM_SIMD_CALL matrix_minor(matrix3x3d_arg0 input, axis3 row, axis3 column) RTM_NO_EXCEPT;

	//////////////////////////////////////////////////////////////////////////
	// Returns the cofactor matrix of the input 3x3 matrix.
	// See: https://en.wikipedia.org/wiki/Minor_(linear_algebra)#Cofactor_expansion_of_the_determinant
	// Note: The proper way to transform a normal by a matrix with non-uniform scale
	// is to multiply the normal with the cofactor matrix.
	// See: https://github.com/graphitemaster/normals_revisited
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_cofactor(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		const vector4d x_axis = vector_cross3(input.y_axis, input.z_axis);
		const vector4d y_axis = vector_cross3(input.z_axis, input.x_axis);
		const vector4d z_axis = vector_cross3(input.x_axis, input.y_axis);
		return matrix3x3d{ x_axis, y_axis, z_axis };
	}

	//////////////////////////////////////////////////////////////////////////
	// Returns the adjugate of the input matrix.
	// See: https://en.wikipedia.org/wiki/Adjugate_matrix
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_adjugate(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		return matrix_transpose(matrix_cofactor(input));
	}

	//////////////////////////////////////////////////////////////////////////
	// Removes the 3D scale from a 3x3 matrix.
	// Note that if the scaling is 0.0 for a particular axis, the original rotation axis cannot
	// be recovered trivially and no attempt is done to do so. In theory, we could use the other axes
	// to try and recover it.
	// TODO: Implement rotation recovering, perhaps in a separate function and rename this
	// one to matrix_remove_non_zero_scale(..)
	//////////////////////////////////////////////////////////////////////////
	RTM_DISABLE_SECURITY_COOKIE_CHECK inline matrix3x3d RTM_SIMD_CALL matrix_remove_scale(matrix3x3d_arg0 input) RTM_NO_EXCEPT
	{
		matrix3x3d result;
		result.x_axis = vector_normalize3(input.x_axis, input.x_axis);
		result.y_axis = vector_normalize3(input.y_axis, input.y_axis);
		result.z_axis = vector_normalize3(input.z_axis, input.z_axis);
		return result;
	}

	RTM_IMPL_VERSION_NAMESPACE_END
}

RTM_IMPL_FILE_PRAGMA_POP
