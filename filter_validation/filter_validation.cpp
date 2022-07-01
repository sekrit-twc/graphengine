#include <algorithm>
#include <array>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "graphengine/filter.h"
#include "graphengine/filter_validation.h"
#include "graphengine/types.h"

extern "C" {
#include "sha1/sha1.h"
} // extern "C"

#include "gtest/gtest.h"

#ifdef _WIN32
  #include <malloc.h>
  static void *aligned_malloc(size_t size, size_t alignment) { return _aligned_malloc(size, alignment); }
  static void aligned_free(void *ptr) { _aligned_free(ptr); }
#else
  #include <stdlib.h>
  static void *aligned_malloc(size_t size, size_t alignment) { void *p; if (posix_memalign(&p, alignment, size)) return nullptr; else return p; }
  static void aligned_free(void *ptr) { free(ptr); }
#endif

namespace graphengine {

namespace {

/**
 * STL allocator class which returns aligned buffers.
 *
 * @tparam T type of object to allocate
 */
template <class T>
struct AlignedAllocator {
	typedef T value_type;

	AlignedAllocator() = default;

	template <class U>
	AlignedAllocator(const AlignedAllocator<U> &) noexcept {}

	T *allocate(size_t n) const
	{
		T *ptr = static_cast<T *>(aligned_malloc(n * sizeof(T), 64));

		if (!ptr)
			throw std::bad_alloc{};

		return ptr;
	}

	void deallocate(void *ptr, size_t) const noexcept
	{
		aligned_free(ptr);
	}

	bool operator==(const AlignedAllocator &) const noexcept { return true; }
	bool operator!=(const AlignedAllocator &) const noexcept { return false; }
};

/**
 * std::vector specialization using AlignedAllocator.
 */
template <class T>
using AlignedVector = std::vector<T, AlignedAllocator<T>>;


unsigned ceil_log2(unsigned count) noexcept
{
	unsigned long lzcnt;

	if (count <= 1)
		return 0;

#if defined(_MSC_VER)
	unsigned long msb;
	_BitScanReverse(&msb, count - 1);
	lzcnt = 31 - msb;
#elif defined(__GNUC__)
	lzcnt = __builtin_clz(count - 1);
#else
	lzcnt = 0;
	count -= 1;
	while (!(count & (1U << (std::numeric_limits<unsigned>::digits - 1)))) {
		count <<= 1;
		++lzcnt;
	}
#endif

	return 32 - lzcnt;
}


template <class T, class U>
T bit_cast(const U &x) noexcept
{
	static_assert(sizeof(T) == sizeof(U), "object sizes must match");
	static_assert(std::is_pod<T>::value && std::is_pod<U>::value, "object types must be POD");

	T ret;
	std::copy_n(reinterpret_cast<const char *>(&x), sizeof(x), reinterpret_cast<char *>(&ret));
	return ret;
}

#define FLOAT_HALF_MANT_SHIFT (23 - 10)
#define FLOAT_HALF_EXP_ADJUST (127 - 15)
float half_to_float(uint16_t f16w) noexcept
{
	constexpr unsigned exp_nonfinite_f16 = 0x1F;
	constexpr unsigned exp_nonfinite_f32 = 0xFF;

	constexpr uint32_t mant_qnan_f32 = 0x00400000UL;

	uint16_t sign = (f16w & 0x8000U) >> 15;
	uint16_t exp = (f16w & 0x7C00U) >> 10;
	uint16_t mant = (f16w & 0x03FFU) >> 0;

	uint32_t f32dw;
	uint32_t exp_f32;
	uint32_t mant_f32;

	// Non-finite.
	if (exp == exp_nonfinite_f16) {
		exp_f32 = exp_nonfinite_f32;

		// Zero extend mantissa and convert sNaN to qNaN.
		if (mant)
			mant_f32 = (mant << FLOAT_HALF_MANT_SHIFT) | mant_qnan_f32;
		else
			mant_f32 = 0;
	} else {
		uint16_t mant_adjust;

		// Denormal.
		if (exp == 0) {
			// Special zero denorm.
			if (mant == 0) {
				mant_adjust = 0;
				exp_f32 = 0;
			} else {
				unsigned renorm = 0;
				mant_adjust = mant;

				while ((mant_adjust & 0x0400) == 0) {
					mant_adjust <<= 1;
					++renorm;
				}

				mant_adjust &= ~0x0400;
				exp_f32 = FLOAT_HALF_EXP_ADJUST - renorm + 1;
			}
		} else {
			mant_adjust = mant;
			exp_f32 = exp + FLOAT_HALF_EXP_ADJUST;
		}

		mant_f32 = static_cast<uint32_t>(mant_adjust) << FLOAT_HALF_MANT_SHIFT;
	}

	f32dw = (static_cast<uint32_t>(sign) << 31) | (exp_f32 << 23) | mant_f32;
	return bit_cast<float>(f32dw);
}

uint16_t float_to_half(float f32) noexcept
{
	constexpr unsigned exp_nonfinite_f32 = 0xFF;
	constexpr unsigned exp_nonfinite_f16 = 0x1F;

	constexpr unsigned mant_qnan_f16 = 0x0200;
	constexpr unsigned mant_max_f16 = 0x03FF;

	uint32_t f32dw = bit_cast<uint32_t>(f32);
	uint32_t sign = (f32dw & 0x80000000UL) >> 31;
	uint32_t exp = (f32dw & 0x7F800000UL) >> 23;
	uint32_t mant = (f32dw & 0x007FFFFFUL) >> 0;

	uint32_t exp_f16;
	uint32_t mant_f16;

	// Non-finite.
	if (exp == exp_nonfinite_f32) {
		exp_f16 = exp_nonfinite_f16;

		// Truncate mantissa and convert sNaN to qNaN.
		if (mant)
			mant_f16 = (mant >> FLOAT_HALF_MANT_SHIFT) | mant_qnan_f16;
		else
			mant_f16 = 0;
	} else {
		uint32_t mant_adjust;
		uint32_t shift;
		uint32_t half;

		// Denormal.
		if (exp <= FLOAT_HALF_EXP_ADJUST) {
			shift = FLOAT_HALF_MANT_SHIFT + FLOAT_HALF_EXP_ADJUST - exp + 1;

			if (shift > 31)
				shift = 31;

			mant_adjust = mant | (1UL << 23);
			exp_f16 = 0;
		} else {
			shift = FLOAT_HALF_MANT_SHIFT;
			mant_adjust = mant;
			exp_f16 = exp - FLOAT_HALF_EXP_ADJUST;
		}

		half = 1UL << (shift - 1);

		// Round half to even.
		mant_f16 = (mant_adjust + half - 1 + ((mant_adjust >> shift) & 1)) >> shift;

		// Detect overflow.
		if (mant_f16 > mant_max_f16) {
			mant_f16 &= mant_max_f16;
			exp_f16 += 1;
		}
		if (exp_f16 >= exp_nonfinite_f16) {
			exp_f16 = exp_nonfinite_f16;
			mant_f16 = 0;
		}
	}

	return (static_cast<uint16_t>(sign) << 15) | (static_cast<uint16_t>(exp_f16) << 10) | static_cast<uint16_t>(mant_f16);
}
#undef FLOAT_HALF_MANT_SHIFT
#undef FLOAT_HALF_EXP_ADJUST


template <class T, class Gen>
T random_int_val(Gen &gen, const FilterValidation::PixelDescriptor &desc)
{
	return static_cast<T>(gen() & ((static_cast<uint32_t>(1) << desc.bits_per_sample) - 1));
}

template <class Gen>
float random_float_val(Gen &gen, const FilterValidation::PixelDescriptor &desc)
{
	double x = static_cast<double>(gen() - gen.min()) / static_cast<double>(gen.max() - gen.min());
	return static_cast<float>(x) - (desc.chroma ? 0.5f : 0.0f);
}

void random_fill_tile(const BufferDescriptor &buffer, const FilterValidation::PixelDescriptor &desc, unsigned bytes_per_sample,
                      unsigned top, unsigned bottom, unsigned left, unsigned right, uint32_t base_seed)
{
	for (unsigned i = top; i < bottom; ++i) {
		void *ptr = buffer.get_line<uint8_t>(i) + static_cast<size_t>(left) * bytes_per_sample;
		unsigned n = right - left;

		uint32_t seed = base_seed ^ static_cast<uint32_t>(i);
		std::mt19937 rng{ seed };
		rng.discard(left);

		if (bytes_per_sample == 1 && !desc.floating_point) {
			std::generate_n(static_cast<uint8_t *>(ptr), n, [&]() { return random_int_val<uint8_t>(rng, desc); });
		} else if (bytes_per_sample == 2 && !desc.floating_point) {
			std::generate_n(static_cast<uint16_t *>(ptr), n, [&]() { return random_int_val<uint16_t>(rng, desc); });
		} else if (bytes_per_sample == 2 && desc.floating_point) {
			std::generate_n(static_cast<uint16_t *>(ptr), n, [&]() { return float_to_half(random_float_val(rng, desc)); });
		} else if (bytes_per_sample == 4 && desc.floating_point) {
			std::generate_n(static_cast<float *>(ptr), n, [&]() { return random_float_val(rng, desc); });
		}
	}
}

void default_fill_tile(const BufferDescriptor &buffer, unsigned bytes_per_sample,
                       unsigned top, unsigned bottom, unsigned left, unsigned right, uint32_t val)
{
	for (unsigned i = top; i < bottom; ++i) {
		void *ptr = buffer.get_line<uint8_t>(i) + static_cast<size_t>(left) * bytes_per_sample;
		unsigned n = right - left;

		if (bytes_per_sample == 1)
			std::fill_n(static_cast<uint8_t *>(ptr), n, static_cast<uint8_t>(val));
		else if (bytes_per_sample == 2)
			std::fill_n(static_cast<uint16_t *>(ptr), n, static_cast<uint16_t>(val));
		else if (bytes_per_sample == 4)
			std::fill_n(static_cast<uint32_t *>(ptr), n, static_cast<uint32_t>(val));
	}
}


template <class T>
struct ToDouble {
	double operator()(const T &x) { return static_cast<double>(x); }
};

template <class T, class Pred = ToDouble<T>>
std::pair<double, double> snr_partial(const T *ref, const T *dis, unsigned n, Pred pred = Pred())
{
	double signal = 0.0;
	double noise = 0.0;

	for (unsigned i = 0; i < n; ++i) {
		double ref_val = pred(ref[i]);
		double dis_val = pred(dis[i]);

		signal += ref_val * ref_val;
		noise += (ref_val - dis_val) * (ref_val - dis_val);
	}

	return{ signal, noise };
}

double snr_tile(const BufferDescriptor &ref, const BufferDescriptor &dis, unsigned bytes_per_sample, bool floating_point,
                unsigned top, unsigned bottom, unsigned left, unsigned right)
{
	double signal = 0.0;
	double noise = 0.0;

	for (unsigned i = top; i < bottom; ++i) {
		const void *ref_ptr = ref.get_line<uint8_t>(i) + static_cast<size_t>(left) * bytes_per_sample;
		const void *dis_ptr = dis.get_line<uint8_t>(i) + static_cast<size_t>(left) * bytes_per_sample;
		unsigned n = right - left;

		std::pair<double, double> snr_result;

		if (bytes_per_sample == 1 && !floating_point) {
			snr_result = snr_partial(static_cast<const uint8_t *>(ref_ptr), static_cast<const uint8_t *>(dis_ptr), n);
		} else if (bytes_per_sample == 2 && !floating_point) {
			snr_result = snr_partial(static_cast<const uint16_t *>(ref_ptr), static_cast<const uint16_t *>(dis_ptr), n);
		} else if (bytes_per_sample == 2 && floating_point) {
			snr_result = snr_partial(static_cast<const uint16_t *>(ref_ptr), static_cast<const uint16_t *>(dis_ptr), n,
				[](uint16_t x) { return half_to_float(x); });
		} else if (bytes_per_sample == 4 && floating_point) {
			snr_result = snr_partial(static_cast<const float *>(ref_ptr), static_cast<const float *>(dis_ptr), n);
		}

		signal += snr_result.first;
		noise += snr_result.second;
	}

	return 10.0 * std::log10(signal / noise);
}


template <class T>
bool find_noteq_T(const T *ptr, size_t n, T val)
{
	return std::find_if(ptr, ptr + n, [=](T x) { return x != val; }) != ptr + n;
}

bool find_noteq(const void *ptr, size_t n, unsigned bytes_per_sample, uint32_t val)
{
	if (bytes_per_sample == 1)
		return find_noteq_T(static_cast<const uint8_t *>(ptr), n, static_cast<uint8_t>(val));
	else if (bytes_per_sample == 2)
		return find_noteq_T(static_cast<const uint16_t *>(ptr), n, static_cast<uint16_t>(val));
	else if (bytes_per_sample == 4)
		return find_noteq_T(static_cast<const uint32_t *>(ptr), n, static_cast<uint32_t>(val));

	return true;
}


template <class T, class Pred = std::equal_to<T>>
std::pair<bool, unsigned> compare_range(const T *a, const T *b, unsigned n, Pred pred = Pred())
{
	for (unsigned i = 0; i < n; ++i) {
		if (!pred(a[i], b[i]))
			return{ false, i };
	}
	return { true, 0 };
}

bool compare_tile(const BufferDescriptor &a, const BufferDescriptor &b, unsigned bytes_per_sample, bool floating_point,
                  unsigned top, unsigned bottom, unsigned left, unsigned right, unsigned &where_i, unsigned &where_j)
{
	for (unsigned i = top; i < bottom; ++i) {
		const void *a_ptr = a.get_line<uint8_t>(i) + static_cast<size_t>(left) * bytes_per_sample;
		const void *b_ptr = b.get_line<uint8_t>(i) + static_cast<size_t>(left) * bytes_per_sample;
		unsigned n = right - left;

#define CHECK(result) do { if (!((result).first)) { where_i = i; where_j = (result).second + left; return false; } } while (0)
		if (bytes_per_sample == 1 && !floating_point) {
			auto result = compare_range(static_cast<const uint8_t *>(a_ptr), static_cast<const uint8_t *>(b_ptr), n);
			CHECK(result);
		} else if (bytes_per_sample == 2 && !floating_point) {
			auto result = compare_range(static_cast<const uint16_t *>(a_ptr), static_cast<const uint16_t *>(b_ptr), n);
			CHECK(result);
		} else if (bytes_per_sample == 2 && floating_point) {
			auto result = compare_range(static_cast<const uint16_t *>(a_ptr), static_cast<const uint16_t *>(b_ptr), n,
				[](uint16_t lhs, uint16_t rhs)
			{
				return half_to_float(lhs) == half_to_float(rhs);
			});
			CHECK(result);
		} else if (bytes_per_sample == 4) {
			auto result = compare_range(static_cast<const float *>(a_ptr), static_cast<const float *>(b_ptr), n);
			CHECK(result);
		}
#undef CHECK
	}

	return true;
}

std::string pixel_to_str(const void *ptr, unsigned bytes_per_sample, bool floating_point)
{
	if (bytes_per_sample == 1 && !floating_point)
		return std::to_string(*static_cast<const uint8_t *>(ptr));
	else if (bytes_per_sample == 2 && !floating_point)
		return std::to_string(*static_cast<const uint16_t *>(ptr));
	else if (bytes_per_sample == 2 && floating_point)
		return std::to_string(half_to_float(*static_cast<const uint16_t *>(ptr)));
	else if (bytes_per_sample == 4 && floating_point)
		return std::to_string(*static_cast<const float *>(ptr));

	return{};
}


class Image {
public:
	enum : uint8_t { GUARD_BYTE = 0xFE };
	enum : unsigned { GUARD_ROWS = 64 };
	enum : unsigned { GUARD_COLS = 64 };

	enum : uint32_t { FILL_VAL = 0xCDCDCDCD };
private:
	std::vector<AlignedVector<uint8_t>> m_plane_data;
	std::vector<BufferDescriptor> m_buffer_desc;
	std::vector<FilterValidation::PixelDescriptor> m_pixel_desc;
	std::vector<PlaneDescriptor> m_plane_desc;
public:
	static uint32_t base_seed(unsigned p) { return static_cast<uint32_t>(p) << 30; }

	Image(const PlaneDescriptor &plane_desc, const FilterValidation::PixelDescriptor pixel_desc[], unsigned num_planes) :
		Image(std::vector<PlaneDescriptor>(num_planes, plane_desc).data(), pixel_desc, num_planes)
	{}

	Image(const PlaneDescriptor plane_desc[], const FilterValidation::PixelDescriptor pixel_desc[], unsigned num_planes) :
		m_plane_data(num_planes),
		m_buffer_desc(num_planes),
		m_pixel_desc(num_planes),
		m_plane_desc(num_planes)
	{
		for (unsigned p = 0; p < num_planes; ++p) {
			size_t rowsize = (static_cast<size_t>(plane_desc[p].width) + GUARD_COLS * 2) * plane_desc[p].bytes_per_sample;
			rowsize = (rowsize + 63) & ~static_cast<size_t>(63);

			size_t inner_rowsize = static_cast<size_t>(plane_desc[p].width) * plane_desc[p].bytes_per_sample;
			inner_rowsize = (inner_rowsize + 63) & ~static_cast<size_t>(63);

			size_t buffer_size = rowsize * (plane_desc[p].height + GUARD_COLS * 2);
			size_t pixel_offset = rowsize * GUARD_ROWS + GUARD_COLS * plane_desc[p].bytes_per_sample;


			m_plane_data[p].resize(buffer_size);

			// Fill guard header and footer.
			std::fill(m_plane_data[p].begin(), m_plane_data[p].begin() + rowsize * GUARD_ROWS, GUARD_BYTE);
			std::fill(m_plane_data[p].end() - rowsize * GUARD_ROWS, m_plane_data[p].end(), GUARD_BYTE);

			m_buffer_desc[p].ptr = m_plane_data[p].data() + pixel_offset;
			m_buffer_desc[p].stride = rowsize;
			m_buffer_desc[p].mask = BUFFER_MAX;

			// Fill row guard bytes.
			for (unsigned i = 0; i < plane_desc[p].height; ++i) {
				uint8_t *ptr = m_buffer_desc[p].get_line<uint8_t>(i);
				std::fill(ptr - GUARD_COLS * plane_desc[p].bytes_per_sample, ptr, GUARD_BYTE);
				std::fill(ptr + inner_rowsize, ptr + rowsize, GUARD_BYTE);
			}

			m_pixel_desc[p] = pixel_desc[p];
			m_plane_desc[p] = plane_desc[p];
		}
	}

	const BufferDescriptor *buffer_desc() const { return m_buffer_desc.data(); }
	BufferDescriptor *mutable_buffer_desc() { return m_buffer_desc.data(); }

	void default_fill(uint32_t val = FILL_VAL)
	{
		for (unsigned p = 0; p < m_buffer_desc.size(); ++p) {
			default_fill_tile(m_buffer_desc[p], m_plane_desc[p].bytes_per_sample, 0, m_plane_desc[p].height, 0, m_plane_desc[p].width, val);
		}
	}

	void random_fill(uint32_t seed = 0)
	{
		for (unsigned p = 0; p < static_cast<unsigned>(m_buffer_desc.size()); ++p) {
			random_fill_tile(m_buffer_desc[p], m_pixel_desc[p], m_plane_desc[p].bytes_per_sample,
			                 0, m_plane_desc[p].height, 0, m_plane_desc[p].width, base_seed(p) ^ seed);
		}
	}

	bool has_write(unsigned p, unsigned i, uint32_t val = FILL_VAL) const
	{
		const void *ptr = m_buffer_desc[p].get_line(i);
		return find_noteq(ptr, m_plane_desc[p].width, m_plane_desc[p].bytes_per_sample, val);
	}

	std::string sha1(unsigned p) const
	{
		const BufferDescriptor &buffer = m_buffer_desc[p];
		size_t rowsize = static_cast<size_t>(m_plane_desc[p].width) * m_plane_desc[p].bytes_per_sample;

		SHA1_CTX ctx;
		SHA1Init(&ctx);

		for (unsigned i = 0; i < m_plane_desc[p].height; ++i) {
			SHA1Update(&ctx, buffer.get_line<unsigned char>(i), static_cast<uint32_t>(rowsize));
		}

		unsigned char sha1_binary[20];
		SHA1Final(sha1_binary, &ctx);

		std::string sha1_str;
		sha1_str.reserve(40);

		for (unsigned char byte : sha1_binary) {
			char hex[3];
			std::sprintf(hex, "%02x", byte);
			sha1_str.push_back(hex[0]);
			sha1_str.push_back(hex[1]);
		}

		return sha1_str;
	}

	std::string pixel_val(unsigned p, unsigned i, unsigned j) const
	{
		const void *ptr = m_buffer_desc[p].get_line<uint8_t>(i) + static_cast<size_t>(j) * m_plane_desc[p].bytes_per_sample;
		return pixel_to_str(ptr, m_plane_desc[p].bytes_per_sample, m_pixel_desc[p].floating_point);
	}

	bool check_guard_header_and_footer(unsigned p) const
	{
		size_t rowsize = m_buffer_desc[p].stride;
		size_t header_size = rowsize * GUARD_COLS;

		if (find_noteq(m_plane_data[p].data(), header_size, 1, GUARD_BYTE))
			return false;
		if (find_noteq(&*(m_plane_data[p].end() - header_size), header_size, 1, GUARD_BYTE))
			return false;
		return true;
	}

	bool check_row_guard_bytes(unsigned p, unsigned i) const
	{
		size_t inner_rowsize = static_cast<size_t>(m_plane_desc[p].width) * m_plane_desc[p].bytes_per_sample;
		inner_rowsize = (inner_rowsize + 63) & ~static_cast<size_t>(63);

		size_t guard_count = GUARD_COLS * static_cast<size_t>(m_plane_desc[p].bytes_per_sample);

		const uint8_t *ptr = m_buffer_desc[p].get_line<uint8_t>(i);
		if (find_noteq(ptr - GUARD_COLS * m_plane_desc[p].bytes_per_sample, guard_count, 1, GUARD_BYTE))
			return false;
		if (find_noteq(ptr + inner_rowsize, guard_count, 1, GUARD_BYTE))
			return false;
		return true;
	}
};

} // namespace


class FilterValidation::impl {
	struct stop_test {};

	const Filter *m_filter;
	const Filter *m_ref_filter;
	std::array<PlaneDescriptor, FILTER_MAX_DEPS> m_dep_format;
	std::array<PixelDescriptor, FILTER_MAX_DEPS> m_dep_pixel_format;
	std::array<PixelDescriptor, FILTER_MAX_PLANES> m_filter_pixel_format;
	std::string m_sha1[FILTER_MAX_PLANES];
	double m_snr;

	std::unique_ptr<Image> m_cached_image;

	static void throw_if_failed()
	{
		if (::testing::Test::HasFailure())
			throw stop_test{};
	}

	template <class Prologue, class Epilogue>
	void process_tile(const Image &input, const Image &output, void *context, void *scratchpad,
	                  unsigned top, unsigned bottom, unsigned left, unsigned right, unsigned step,
	                  Prologue prologue, Epilogue epilogue)
	{
		const FilterDescriptor &desc = m_filter->descriptor();

		for (unsigned i = top; i < bottom; i += step) {
			SCOPED_TRACE(i);
			unsigned cur_tile_bottom = std::min(i + desc.step, desc.format.height);

			Filter::pair_unsigned row_range = m_filter->get_row_deps(i);
			prologue(i, cur_tile_bottom, row_range);
			throw_if_failed();

			m_filter->process(input.buffer_desc(), output.buffer_desc(), i, left, right, context, scratchpad);

			for (unsigned p = 0; p < desc.num_planes; ++p) {
				SCOPED_TRACE(p);

				for (unsigned ii = i; ii < cur_tile_bottom; ++ii) {
					SCOPED_TRACE(ii);
					output.check_row_guard_bytes(p, ii);
					throw_if_failed();

					const uint8_t *ptr = output.buffer_desc()[p].get_line<uint8_t>(ii);
					EXPECT_FALSE(find_noteq(ptr, left, desc.format.bytes_per_sample, Image::FILL_VAL)) << "write outside of tile bounds";
					EXPECT_FALSE(find_noteq(ptr + static_cast<size_t>(right) * desc.format.bytes_per_sample,
						desc.format.width - right, desc.format.bytes_per_sample, Image::FILL_VAL)) << "write outside of tile bounds";
					throw_if_failed();
				}
			}

			epilogue(i, cur_tile_bottom, row_range);
			throw_if_failed();
		}

		for (unsigned p = 0; p < desc.num_planes; ++p) {
			EXPECT_TRUE(output.check_guard_header_and_footer(p)) << "guard bytes corrupted: write to invalid row";
			throw_if_failed();
		}
	}

#define TRY try
#define CATCH catch (const stop_test &) { return false; }
	bool validate_filter_descriptor()
	{
		SCOPED_TRACE("validate_descriptor");

		auto validate_plane = [=](const PlaneDescriptor &desc)
		{
			ASSERT_TRUE(desc.bytes_per_sample == 1 || desc.bytes_per_sample == 2 || desc.bytes_per_sample == 4) << desc.bytes_per_sample;
			ASSERT_LE(desc.width, UINT_MAX & ~63U);
			ASSERT_LE(desc.width, (static_cast<size_t>(PTRDIFF_MAX) & ~static_cast<size_t>(63)) / desc.bytes_per_sample);
			ASSERT_LE(desc.height, static_cast<size_t>(PTRDIFF_MAX) / (static_cast<size_t>(desc.width) * desc.bytes_per_sample));
		};

		auto validate_pixel_format = [=](const PlaneDescriptor &plane, const PixelDescriptor &pixel)
		{
			EXPECT_LE(pixel.bits_per_sample, plane.bytes_per_sample * CHAR_BIT);
			if (pixel.floating_point) {
				EXPECT_TRUE(plane.bytes_per_sample == 2 || plane.bytes_per_sample == 4) << "half and single precision only: " << plane.bytes_per_sample;
				EXPECT_EQ(plane.bytes_per_sample * CHAR_BIT, pixel.bits_per_sample);
			}
		};

		const FilterDescriptor &desc = m_filter->descriptor();
		validate_plane(desc.format);

		EXPECT_LE(desc.num_deps, FILTER_MAX_DEPS);
		EXPECT_LE(desc.num_planes, FILTER_MAX_PLANES);

		for (unsigned p = 0; p < desc.num_deps; ++p) {
			validate_plane(m_dep_format[p]);
			validate_pixel_format(m_dep_format[p], m_dep_pixel_format[p]);
			EXPECT_EQ(m_dep_format[0].width, m_dep_format[p].width);
			EXPECT_EQ(m_dep_format[0].height, m_dep_format[p].height);
		}
		for (unsigned p = 0; p < desc.num_planes; ++p) {
			validate_pixel_format(desc.format, m_filter_pixel_format[p]);
		}

		EXPECT_LE(desc.alignment_mask, 63U);
		EXPECT_EQ(0U, desc.alignment_mask & (desc.alignment_mask + 1)) << desc.alignment_mask;

		if (desc.flags.entire_col) {
			EXPECT_GE(desc.step, desc.format.height);
		}

		return !testing::Test::HasFailure();
	}

	bool test_row_deps() TRY
	{
		SCOPED_TRACE("test_row_deps");

		unsigned height = m_filter->descriptor().format.height;
		Filter::pair_unsigned prev{};

		for (unsigned i = 0; i < height; ++i) {
			SCOPED_TRACE(i);

			Filter::pair_unsigned cur = m_filter->get_row_deps(i);
			EXPECT_LE(cur.first, cur.second) << i;
			EXPECT_GE(cur.first, prev.first) << i;
			EXPECT_GE(cur.second, prev.second) << i;
			throw_if_failed();
		}

		return true;
	} CATCH

	bool test_simple() TRY
	{
		SCOPED_TRACE("test_simple");

		const FilterDescriptor &desc = m_filter->descriptor();

		Image input{ m_dep_format.data(), m_dep_pixel_format.data(), desc.num_deps};
		Image output{ desc.format, m_filter_pixel_format.data(),  desc.num_planes };

		AlignedVector<uint8_t> context(desc.context_size);
		AlignedVector<uint8_t> scratchpad(desc.scratchpad_size);

		input.random_fill();
		output.default_fill();

		m_filter->init_context(context.data());

		process_tile(input, output, context.data(), scratchpad.data(), 0, desc.format.height, 0, desc.format.width, desc.step,
			[](unsigned i, unsigned i_next, Filter::pair_unsigned dep_range) {},
			[&](unsigned i, unsigned i_next, Filter::pair_unsigned dep_range)
			{
				for (unsigned p = 0; p < desc.num_planes; ++p) {
					SCOPED_TRACE(p);
					for (unsigned ii = i_next; ii < desc.format.height; ++ii) {
						SCOPED_TRACE(ii);
						EXPECT_FALSE(output.has_write(p, ii)) << "write out of bounds";
						throw_if_failed();
					}
				}
			});

		for (unsigned p = 0; p < desc.num_planes; ++p) {
			if (m_sha1[p].empty())
				continue;

			SCOPED_TRACE(p);
			EXPECT_EQ(m_sha1[p], output.sha1(p)) << "hash mismatch";
		}
		throw_if_failed();

		m_cached_image = std::make_unique<Image>(std::move(output));
		return true;
	} CATCH

	bool test_reference() TRY
	{
		SCOPED_TRACE("test_reference");
		assert(m_cached_image);

		const FilterDescriptor &desc = m_ref_filter->descriptor();

		Image input{ m_dep_format.data(), m_dep_pixel_format.data(), desc.num_deps};
		Image output{ desc.format, m_filter_pixel_format.data(),  desc.num_planes };

		AlignedVector<uint8_t> context(desc.context_size);
		AlignedVector<uint8_t> scratchpad(desc.scratchpad_size);

		input.random_fill();
		output.default_fill();

		m_ref_filter->init_context(context.data());
		for (unsigned i = 0; i < desc.format.height; i += desc.step) {
			m_ref_filter->process(input.buffer_desc(), output.buffer_desc(), i, 0, desc.format.width, context.data(), scratchpad.data());
		}

		for (unsigned p = 0; p < desc.num_planes; ++p) {
#define TILE desc.format.bytes_per_sample, m_filter_pixel_format[p].floating_point, 0, desc.format.height, 0, desc.format.width
			SCOPED_TRACE(p);
			double snr = snr_tile(output.buffer_desc()[p], m_cached_image->buffer_desc()[p], TILE);
			EXPECT_GE(snr, m_snr) << "SNR tolerance exceeded";

			if (m_snr == INFINITY && snr < INFINITY) {
				unsigned i, j;
				compare_tile(output.buffer_desc()[p], m_cached_image->buffer_desc()[p], TILE, i, j);
				ADD_FAILURE() << "mismatch at (" << i << ", " << j << "): " << output.pixel_val(p, i, j) << " vs " << m_cached_image->pixel_val(p, i, j);
			}
#undef TILE
		}
		throw_if_failed();

		return true;
	} CATCH

	bool test_tiled() TRY
	{
		SCOPED_TRACE("test_tiled");
		assert(m_cached_image);

		const FilterDescriptor &desc = m_filter->descriptor();

		// Process an inner rectangle.
		unsigned top = (desc.flags.entire_col || desc.flags.stateful) ? 0 : desc.format.height / 4;
		unsigned bottom = desc.flags.entire_col ? desc.format.height : desc.format.height - desc.format.height / 4;

		unsigned left = desc.flags.entire_row ? 0 : desc.format.width / 4;
		unsigned right = desc.flags.entire_row ? desc.format.width : desc.format.width - desc.format.width / 4;

		if (desc.alignment_mask) {
			// Honor alignment.
			left = left & ~desc.alignment_mask;
			right = (right + desc.alignment_mask) & ~desc.alignment_mask;
		} else {
			// Misalign the boundaries.
			left = std::max(left & ~15U, 3U) - 3U;
			right = ((right + 15U) & ~15U) + 3U;
		}
		right = std::min(right, desc.format.width);

		// Test overlapping ranges.
		unsigned step = (desc.flags.entire_col || desc.flags.stateful) ? desc.step : (desc.step + 1) / 2;

		// Calculate buffer masks;
		unsigned input_buffering = 0;
		unsigned input_rows = 0;

		if (!desc.flags.entire_col) {
			for (unsigned i = top; i < bottom; i += step) {
				Filter::pair_unsigned range = m_filter->get_row_deps(i);
				input_buffering = std::max(input_buffering, range.second - range.first);
			}
			input_buffering = (1U << ceil_log2(input_buffering)) - 1;
			input_rows = input_buffering + 1;
		} else {
			input_buffering = BUFFER_MAX;
			input_rows = m_dep_format[0].height;
		}

		unsigned output_buffering = 0;
		unsigned output_rows = 0;

		if (!desc.flags.entire_col) {
			output_buffering = (1U << ceil_log2(desc.step)) - 1;
			output_rows = output_buffering + 1;
		} else {
			output_buffering = BUFFER_MAX;
			output_rows = desc.format.height;
		}

		// Setup buffers.
		std::array<PlaneDescriptor, FILTER_MAX_DEPS> input_format = m_dep_format;
		for (PlaneDescriptor &format : input_format) {
			format.height = input_rows;
		}

		Image input(input_format.data(), m_dep_pixel_format.data(), desc.num_deps);
		Image output({ desc.format.width, output_rows, desc.format.bytes_per_sample }, m_filter_pixel_format.data(), desc.num_planes);

		for (unsigned p = 0; p < desc.num_deps; ++p) {
			input.mutable_buffer_desc()[p].mask = input_buffering;
		}
		for (unsigned p = 0; p < desc.num_planes; ++p) {
			output.mutable_buffer_desc()[p].mask = output_buffering;
		}

		// Test.
		AlignedVector<uint8_t> context(desc.context_size);
		AlignedVector<uint8_t> scratchpad(desc.scratchpad_size);

		m_filter->init_context(context.data());
		Filter::pair_unsigned col_range = m_filter->get_col_deps(left, right);

		process_tile(input, output, context.data(), scratchpad.data(), top, bottom, left, right, step,
			[&](unsigned i, unsigned i_next, Filter::pair_unsigned dep_range)
			{
				input.default_fill();
				output.default_fill();

				for (unsigned p = 0; p < desc.num_deps; ++p) {
					random_fill_tile(input.buffer_desc()[p], m_dep_pixel_format[p], m_dep_format[p].bytes_per_sample,
						dep_range.first, dep_range.second, col_range.first, col_range.second, Image::base_seed(p));
				}

			},
			[&](unsigned i, unsigned i_next, Filter::pair_unsigned dep_range)
			{
				for (unsigned p = 0; p < desc.num_planes; ++p) {
					unsigned where_i, where_j;
					EXPECT_TRUE(compare_tile(m_cached_image->buffer_desc()[p], output.buffer_desc()[p], desc.format.bytes_per_sample,
						m_filter_pixel_format[p].floating_point, i, i_next, left, right, where_i, where_j))
						<< "mismatch at (" << where_i << ", " << where_j << "): " << output.pixel_val(p, where_i, where_j)
						<< " vs " << m_cached_image->pixel_val(p, where_i, where_j);
				}
			});

		return true;
	} CATCH

	bool test_inplace(bool honor_preferred = true) TRY
	{
		SCOPED_TRACE(honor_preferred ? "test_inplace" : "test_inplace_nonpreferred");
		assert(m_cached_image);

		const FilterDescriptor &desc = m_filter->descriptor();
		std::vector<unsigned> dep_to_plane(desc.num_deps, UINT_MAX);
		std::vector<unsigned char> taken(desc.num_deps, 0);

		// Validate hints.
		if (desc.inplace_hint.enabled) {
			EXPECT_LT(desc.inplace_hint.preferred_index, desc.num_deps);
			throw_if_failed();

			const PlaneDescriptor &dep_plane = m_dep_format[desc.inplace_hint.preferred_index];
			if ((static_cast<size_t>(dep_plane.width) * dep_plane.bytes_per_sample !=
					static_cast<size_t>(desc.format.width) * desc.format.bytes_per_sample) ||
				dep_plane.height != desc.format.height)
			{
				ADD_FAILURE() << "hinted dep plane is incompatible with filter: dep "
					<< dep_plane.width << "x" << dep_plane.height << "/" << dep_plane.bytes_per_sample
					<< ", filter " << desc.format.width << "x" << desc.format.height << "/" << desc.format.bytes_per_sample;
				return false;
			}

			if (!honor_preferred)
				taken[desc.inplace_hint.preferred_index] = 1;
		}

		// Search for matching planes.
		for (unsigned p = desc.num_planes; p != 0; --p)  {
			// Honor hints.
			if (desc.inplace_hint.enabled && !taken[desc.inplace_hint.preferred_index]) {
				dep_to_plane[desc.inplace_hint.preferred_index] = p - 1;
				taken[desc.inplace_hint.preferred_index] = 1;
				continue;
			}

			// Scan deps for compatible planes.
			for (unsigned q = desc.num_deps; q != 0; --q) {
				if (taken[q - 1])
					continue;
				if (desc.inplace_hint.enabled && (desc.inplace_hint.disallow_mask & (1U << (q - 1))))
					continue;

				const PlaneDescriptor &dep_plane = m_dep_format[q - 1];
				if ((static_cast<size_t>(dep_plane.width) * dep_plane.bytes_per_sample !=
						static_cast<size_t>(desc.format.width) * desc.format.bytes_per_sample) ||
					dep_plane.height != desc.format.height)
				{
					continue;
				}

				dep_to_plane[q - 1] = p - 1;
				taken[q - 1] = 1;
				break;
			}
		}

		if (desc.inplace_hint.enabled && !honor_preferred)
			taken[desc.inplace_hint.preferred_index] = 0;

		EXPECT_GT(std::count(taken.begin(), taken.end(), 1), 0) << "in_place flag set, but filter can not operate in-place";
		throw_if_failed();

		// Setup buffers.
		Image input(m_dep_format.data(), m_dep_pixel_format.data(), desc.num_deps);
		Image output(desc.format, m_filter_pixel_format.data(), desc.num_planes);
		output.default_fill();

		for (unsigned q = 0; q < desc.num_deps; ++q) {
			unsigned p = dep_to_plane[q];
			if (p == UINT_MAX)
				continue;

			input.mutable_buffer_desc()[q] = output.buffer_desc()[p];
		}

		// Process an inner rectangle.
		unsigned top = (desc.flags.entire_col || desc.flags.stateful) ? 0 : desc.format.height / 4;
		unsigned bottom = desc.flags.entire_col ? desc.format.height : desc.format.height - desc.format.height / 4;

		unsigned left = desc.flags.entire_row ? 0 : desc.format.width / 4;
		unsigned right = desc.flags.entire_row ? desc.format.width : desc.format.width - desc.format.width / 4;

		// Honor alignment.
		left = left & ~desc.alignment_mask;
		right = (right + desc.alignment_mask) & ~desc.alignment_mask;

		// Test.
		AlignedVector<uint8_t> context(desc.context_size);
		AlignedVector<uint8_t> scratchpad(desc.scratchpad_size);

		m_filter->init_context(context.data());
		Filter::pair_unsigned col_range = m_filter->get_col_deps(left, right);
		EXPECT_EQ(col_range.first, left) << "in-place requires 1:1 mapping of pixels";
		EXPECT_EQ(col_range.second, right) << "in-place requires 1:1 mapping of pixels";
		throw_if_failed();

		process_tile(input, output, context.data(), scratchpad.data(), top, bottom, left, right, desc.step,
			[&](unsigned i, unsigned i_next, Filter::pair_unsigned dep_range)
			{
				EXPECT_EQ(dep_range.first, i) << "in-place requires 1:1 mapping of pixels";
				EXPECT_EQ(dep_range.second, i_next) << "in-place requires 1:1 mapping of pixels";

				for (unsigned p = 0; p < desc.num_deps; ++p) {
					random_fill_tile(input.buffer_desc()[p], m_dep_pixel_format[p], m_dep_format[p].bytes_per_sample,
						dep_range.first, dep_range.second, col_range.first, col_range.second, Image::base_seed(p));
				}
			},
			[&](unsigned i, unsigned i_next, Filter::pair_unsigned dep_range)
			{
				for (unsigned p = 0; p < desc.num_planes; ++p) {
					unsigned where_i, where_j;
					EXPECT_TRUE(compare_tile(m_cached_image->buffer_desc()[p], output.buffer_desc()[p], desc.format.bytes_per_sample,
						m_filter_pixel_format[p].floating_point, i, i_next, left, right, where_i, where_j))
						<< "mismatch at (" << where_i << ", " << where_j << "): " << output.pixel_val(p, where_i, where_j)
						<< " vs " << m_cached_image->pixel_val(p, where_i, where_j);
				}
			});

		return true;
	} CATCH

	bool test_inplace_nonpreferred_plane() { return test_inplace(false); }
#undef CATCH
#undef TRY
public:
	impl(const Filter *filter, const PlaneDescriptor &dep_format, const unsigned bytes_per_sample[]) :
		m_filter{ filter },
		m_ref_filter{},
		m_dep_format{},
		m_dep_pixel_format{},
		m_filter_pixel_format{},
		m_snr{ INFINITY }
	{
		const bool have_dep_format = dep_format.width || dep_format.height;

		const FilterDescriptor &desc = m_filter->descriptor();
		if (desc.num_deps > 0 && !have_dep_format)
			throw std::invalid_argument{ "must specify input format" };
		if (!dep_format.bytes_per_sample && !bytes_per_sample)
			throw std::invalid_argument{ "must specify bytes_per_sample" };

		for (unsigned p = 0; p < desc.num_deps; ++p) {
			m_dep_format[p] = dep_format;
			if (bytes_per_sample)
				m_dep_format[p].bytes_per_sample = bytes_per_sample[p];
			m_dep_pixel_format[p].bits_per_sample = m_dep_format[p].bytes_per_sample * CHAR_BIT;
		}
		for (unsigned p = 0; p < desc.num_planes; ++p) {
			m_filter_pixel_format[p].bits_per_sample = desc.format.bytes_per_sample * CHAR_BIT;
		}
	}

	void set_reference_filter(const Filter *ref_filter, double snr)
	{
		const FilterDescriptor &desc = m_filter->descriptor();
		const FilterDescriptor &ref_desc = ref_filter->descriptor();

		if (desc.format.width != ref_desc.format.width ||
			desc.format.height != ref_desc.format.height ||
			desc.format.bytes_per_sample != ref_desc.format.bytes_per_sample ||
			desc.num_deps != ref_desc.num_deps ||
			desc.num_planes != ref_desc.num_planes)
		{
			throw std::invalid_argument{ "ref and test filters incompatible" };
		}

		m_ref_filter = ref_filter;
		m_snr = snr;
	}

	void set_input_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc)
	{
		const FilterDescriptor &desc = m_filter->descriptor();
		if (plane > desc.num_deps && plane != UINT_MAX)
			throw std::invalid_argument{ "plane index out of bounds" };
		if (pixel_desc.bits_per_sample > m_dep_format[plane == UINT_MAX ? 0 : plane].bytes_per_sample * CHAR_BIT)
			throw std::invalid_argument{ "bit depth out of bounds" };

		if (plane == UINT_MAX)
			std::fill(m_dep_pixel_format.begin(), m_dep_pixel_format.end(), pixel_desc);
		else
			m_dep_pixel_format[plane] = pixel_desc;
	}

	void set_output_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc)
	{
		const FilterDescriptor &desc = m_filter->descriptor();
		if (plane > desc.num_planes && plane != UINT_MAX)
			throw std::invalid_argument{ "plane index out of bounds" };
		if (pixel_desc.bits_per_sample > desc.format.bytes_per_sample * CHAR_BIT)
			throw std::invalid_argument{ "bit depth out of bounds" };

		if (plane == UINT_MAX)
			std::fill(m_filter_pixel_format.begin(), m_filter_pixel_format.end(), pixel_desc);
		else
			m_filter_pixel_format[plane] = pixel_desc;
	}

	void set_sha1(unsigned plane, const char sha1[40])
	{
		const FilterDescriptor &desc = m_filter->descriptor();
		if (plane > desc.num_planes)
			throw std::invalid_argument{ "plane index out of bounds" };

		m_sha1[plane].assign(sha1, 40);
	}

	bool run()
	{
		const FilterDescriptor &desc = m_filter->descriptor();

#define TEST_CASE(test) \
  do { \
    if (!(test())) \
      return false; \
  } while (0)
		TEST_CASE(validate_filter_descriptor);

		TEST_CASE(test_row_deps);

		// test_simple generates the baseline image data for the other tests.
		TEST_CASE(test_simple);

		if (m_ref_filter)
			TEST_CASE(test_reference);

		if (!(desc.flags.entire_row && desc.flags.entire_col))
			TEST_CASE(test_tiled);

		if (desc.flags.in_place)
			TEST_CASE(test_inplace);

		if (desc.flags.in_place && desc.inplace_hint.enabled)
			TEST_CASE(test_inplace_nonpreferred_plane);
#undef TEST_CASE

		return true;
	}
};


FilterValidation::FilterValidation(const Filter *filter) :
	FilterValidation(filter, PlaneDescriptor{})
{}

FilterValidation::FilterValidation(const Filter *filter, const PlaneDescriptor &dep_format, const unsigned bytes_per_sample[]) :
	m_impl(new impl(filter, dep_format, bytes_per_sample))
{}

FilterValidation::FilterValidation(FilterValidation &&other) noexcept = default;
FilterValidation::~FilterValidation() = default;
FilterValidation &FilterValidation::operator=(FilterValidation &&other) noexcept = default;

FilterValidation &FilterValidation::set_reference_filter(const Filter *ref_filter, double snr_thresh)
{
	m_impl->set_reference_filter(ref_filter, snr_thresh);
	return *this;
}

FilterValidation &FilterValidation::set_input_pixel_format(const PixelDescriptor &pixel_desc)
{
	return set_input_pixel_format(UINT_MAX, pixel_desc);
}

FilterValidation &FilterValidation::set_input_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc)
{
	m_impl->set_input_pixel_format(plane, pixel_desc);
	return *this;
}

FilterValidation &FilterValidation::set_output_pixel_format(const PixelDescriptor &pixel_desc)
{
	return set_output_pixel_format(UINT_MAX, pixel_desc);
}

FilterValidation &FilterValidation::set_output_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc)
{
	m_impl->set_output_pixel_format(plane, pixel_desc);
	return *this;
}

FilterValidation &FilterValidation::set_sha1(unsigned plane, const char sha1[40])
{
	m_impl->set_sha1(plane, sha1);
	return *this;
}

bool FilterValidation::run()
{
	return m_impl->run();
}

} // namespace graphengine
