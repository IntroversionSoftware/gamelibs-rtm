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

#include "rtm/config.h"
#include "rtm/version.h"
#include "rtm/impl/compiler_utils.h"
#include "rtm/impl/detect_compiler.h"
#include "rtm/impl/detect_cpp_version.h"

RTM_IMPL_FILE_PRAGMA_PUSH

// See config.h for details on how to configure asserts for your project

#if defined(RTM_ON_ASSERT_ABORT)

	#include <cstdio>
	#include <cstdarg>
	#include <cstdlib>

	namespace rtm
	{
		RTM_IMPL_VERSION_NAMESPACE_BEGIN

		namespace rtm_impl
		{
			[[noreturn]] RTM_DISABLE_SECURITY_COOKIE_CHECK inline void on_assert_abort(const char* expression, int line, const char* file, const char* format, ...)
			{
				(void)expression;
				(void)line;
				(void)file;

				va_list args;
				va_start(args, format);

				std::vfprintf(stderr, format, args);
				std::fprintf(stderr, "\n");

				va_end(args);

				std::abort();
			}
		}

		RTM_IMPL_VERSION_NAMESPACE_END
	}

	#define RTM_ASSERT(expression, format, ...) do { if (!(expression)) RTM_IMPL_NAMESPACE::rtm_impl::on_assert_abort(#expression, __LINE__, __FILE__, (format), ## __VA_ARGS__); } while(false)
	#define RTM_HAS_ASSERT_CHECKS
	#define RTM_NO_EXCEPT noexcept

#elif defined(RTM_ON_ASSERT_THROW)

	#include <cstdio>
	#include <cstdarg>
	#include <string>
	#include <stdexcept>

	namespace rtm
	{
		RTM_IMPL_VERSION_NAMESPACE_BEGIN

		class runtime_assert final : public std::runtime_error
		{
			runtime_assert() = delete;					// Default constructor not needed

			using std::runtime_error::runtime_error;	// Inherit constructors
		};

		namespace rtm_impl
		{
			[[noreturn]] RTM_DISABLE_SECURITY_COOKIE_CHECK inline void on_assert_throw(const char* expression, int line, const char* file, const char* format, ...)
			{
				(void)expression;
				(void)line;
				(void)file;

				constexpr int buffer_size = 1 * 1024;
				char buffer[buffer_size];

				va_list args;
				va_start(args, format);

				const int count = vsnprintf(buffer, buffer_size, format, args);

				va_end(args);

				if (count >= 0 && count < buffer_size)
					throw runtime_assert(std::string(&buffer[0], static_cast<std::string::size_type>(count)));
				else
					throw runtime_assert("Failed to format assert message!\n");
			}
		}

		RTM_IMPL_VERSION_NAMESPACE_END
	}

	#define RTM_ASSERT(expression, format, ...) do { if (!(expression)) RTM_IMPL_NAMESPACE::rtm_impl::on_assert_throw(#expression, __LINE__, __FILE__, (format), ## __VA_ARGS__); } while(false)
	#define RTM_HAS_ASSERT_CHECKS
	#define RTM_NO_EXCEPT

#elif defined(RTM_ON_ASSERT_CUSTOM)

	#if !defined(RTM_ASSERT)
		#define RTM_ASSERT(expression, format, ...) do { if (!(expression)) RTM_ON_ASSERT_CUSTOM(#expression, __LINE__, __FILE__, (format), ## __VA_ARGS__); } while(false)
	#endif

	#define RTM_HAS_ASSERT_CHECKS
	#define RTM_NO_EXCEPT

#else

	#define RTM_ASSERT(expression, format, ...) ((void)0)
	#define RTM_NO_EXCEPT noexcept

#endif

//////////////////////////////////////////////////////////////////////////
// Deprecation support
//////////////////////////////////////////////////////////////////////////

// Use RTM_NO_DEPRECATION to disable all deprecation warnings
#if !defined(RTM_NO_DEPRECATION)
	#if defined(__has_cpp_attribute) && RTM_CPP_VERSION >= RTM_CPP_VERSION_14
		#if __has_cpp_attribute(deprecated)
			#define RTM_DEPRECATED(msg) [[deprecated(msg)]]
		#endif
	#endif

	#if !defined(RTM_DEPRECATED)
		#if defined(RTM_COMPILER_GCC) || defined(RTM_COMPILER_CLANG)
			#define RTM_DEPRECATED(msg) __attribute__((deprecated))
		#elif defined(RTM_COMPILER_MSVC)
			#define RTM_DEPRECATED(msg) __declspec(deprecated)
		#endif
	#endif
#endif

// If not defined, suppress all deprecation warnings
#if !defined(RTM_DEPRECATED)
	#define RTM_DEPRECATED(msg)
#endif

RTM_IMPL_FILE_PRAGMA_POP
