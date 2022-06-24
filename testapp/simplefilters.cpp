#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include "simplefilters.h"

#ifdef _MSC_VER
  #include <emmintrin.h>
  static long xlrintf(float x) { return _mm_cvtss_si32(_mm_set_ss(x)); }
  static float xsqrtf(float x) { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_set_ss(x))); }
#else
  #define xlrintf lrintf
  #define xsqrtf sqrtf
#endif

namespace {

class Spatial3x3Filter : public graphengine::Filter {
protected:
	graphengine::FilterDescriptor m_desc = {};
public:
	Spatial3x3Filter(unsigned width, unsigned height)
	{
		m_desc.format.width = width;
		m_desc.format.height = height;
		m_desc.format.bytes_per_sample = 1;

		m_desc.num_deps = 1;
		m_desc.num_planes = 1;
		m_desc.step = 1;
		m_desc.alignment_mask = 7;
	}

	int version() const noexcept override { return VERSION; }

	const graphengine::FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned i) const noexcept override
	{
		unsigned topdep = std::max(i, 1U) - 1;
		unsigned bottomdep = static_cast<unsigned>(
			std::min(static_cast<size_t>(i) + 2, static_cast<size_t>(m_desc.format.height)));
		return{ topdep, bottomdep };
	}

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override
	{
		unsigned leftdep = std::max(left, 1U) - 1;
		unsigned rightdep = static_cast<unsigned>(
			std::min(static_cast<size_t>(right) + 1, static_cast<size_t>(m_desc.format.width)));
		return{ leftdep, rightdep };
	}

	void init_context(void *) const noexcept override {}

};

class BoxBlurFilter final : public Spatial3x3Filter {
public:
	using Spatial3x3Filter::Spatial3x3Filter;

	void process(const graphengine::BufferDescriptor *in, const graphengine::BufferDescriptor *out,
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept override
	{
		auto range = get_row_deps(i);
		const uint8_t *y0 = in->get_line<uint8_t>(range.first);
		const uint8_t *y1 = in->get_line<uint8_t>(i);
		const uint8_t *y2 = in->get_line<uint8_t>(range.second - 1);
		uint8_t *dstp = out->get_line<uint8_t>(i);

		for (ptrdiff_t j = left; j < static_cast<ptrdiff_t>(right); j += 8) {
			__m128i v00 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y0 + j - 1)), _mm_setzero_si128());
			__m128i v01 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y0 + j + 0)), _mm_setzero_si128());
			__m128i v02 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y0 + j + 1)), _mm_setzero_si128());
			__m128i v10 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y1 + j - 1)), _mm_setzero_si128());
			__m128i v11 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y1 + j + 0)), _mm_setzero_si128());
			__m128i v12 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y1 + j + 1)), _mm_setzero_si128());
			__m128i v20 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y2 + j - 1)), _mm_setzero_si128());
			__m128i v21 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y2 + j + 0)), _mm_setzero_si128());
			__m128i v22 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y2 + j + 1)), _mm_setzero_si128());

			__m128i sum = _mm_add_epi16(v00, v01);
			sum = _mm_add_epi16(sum, v02);
			sum = _mm_add_epi16(sum, v10);
			sum = _mm_add_epi16(sum, v11);
			sum = _mm_add_epi16(sum, v12);
			sum = _mm_add_epi16(sum, v20);
			sum = _mm_add_epi16(sum, v21);
			sum = _mm_add_epi16(sum, v22);
			sum = _mm_add_epi16(sum, _mm_set1_epi16(4));

			__m128i val = _mm_mulhi_epu16(sum, _mm_set1_epi16(58255));
			val = _mm_srli_epi16(val, 3);
			val = _mm_packus_epi16(val, val);
			_mm_storel_epi64((__m128i *)(dstp + j), val);
		}
	}
};

class SobelFilter final : public Spatial3x3Filter {
public:
	using Spatial3x3Filter::Spatial3x3Filter;

	void process(const graphengine::BufferDescriptor *in, const graphengine::BufferDescriptor *out,
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept override
	{
		auto range = get_row_deps(i);
		const uint8_t *y0 = in->get_line<uint8_t>(range.first);
		const uint8_t *y1 = in->get_line<uint8_t>(i);
		const uint8_t *y2 = in->get_line<uint8_t>(range.second - 1);
		uint8_t *dstp = out->get_line<uint8_t>(i);

		for (ptrdiff_t j = left; j < static_cast<ptrdiff_t>(right); j += 8) {
			__m128i v00 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y0 + j - 1)), _mm_setzero_si128());
			__m128i v01 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y0 + j + 0)), _mm_setzero_si128());
			__m128i v02 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y0 + j + 1)), _mm_setzero_si128());
			__m128i v10 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y1 + j - 1)), _mm_setzero_si128());
			__m128i v11 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y1 + j + 0)), _mm_setzero_si128());
			__m128i v12 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y1 + j + 1)), _mm_setzero_si128());
			__m128i v20 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y2 + j - 1)), _mm_setzero_si128());
			__m128i v21 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y2 + j + 0)), _mm_setzero_si128());
			__m128i v22 = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(y2 + j + 1)), _mm_setzero_si128());

			__m128i v22_sub_v00 = _mm_sub_epi16(v22, v00);

			__m128i gx = _mm_add_epi16(_mm_add_epi16(v20, _mm_add_epi16(v21, v21)), v22_sub_v00); // gx = v20 + 2 * v21 + v22 - v00
			gx = _mm_sub_epi16(_mm_sub_epi16(gx, _mm_add_epi16(v01, v01)), v02);                  //    - 2 * v01 - v02

			__m128i gy = _mm_add_epi16(_mm_add_epi16(v02, _mm_add_epi16(v12, v12)), v22_sub_v00); // gy = v02 + 2 * v12 + v22 - v00
			gy = _mm_sub_epi16(_mm_sub_epi16(gy, _mm_add_epi16(v10, v10)), v20);                  //    - 2 * v10 - v20

			__m128i gx_lo = _mm_unpacklo_epi16(gx, _mm_setzero_si128());
			__m128i gx_hi = _mm_unpackhi_epi16(gx, _mm_setzero_si128());
			__m128i gy_lo = _mm_unpacklo_epi16(gy, _mm_setzero_si128());
			__m128i gy_hi = _mm_unpackhi_epi16(gy, _mm_setzero_si128());

			__m128 gx_lo_f32 = _mm_cvtepi32_ps(gx_lo);
			__m128 gy_lo_f32 = _mm_cvtepi32_ps(gy_lo);
			__m128 lo_f32 = _mm_sqrt_ps(_mm_add_ps(_mm_mul_ps(gx_lo_f32, gx_lo_f32), _mm_mul_ps(gy_lo_f32, gy_lo_f32)));

			__m128 gx_hi_f32 = _mm_cvtepi32_ps(gx_hi);
			__m128 gy_hi_f32 = _mm_cvtepi32_ps(gy_hi);
			__m128 hi_f32 = _mm_sqrt_ps(_mm_add_ps(_mm_mul_ps(gx_hi_f32, gx_hi_f32), _mm_mul_ps(gy_hi_f32, gy_hi_f32)));

			__m128i lo = _mm_cvtps_epi32(lo_f32);
			__m128i hi = _mm_cvtps_epi32(hi_f32);
			__m128i out = _mm_packs_epi32(lo, hi);
			out = _mm_packus_epi16(out, out);
			_mm_storel_epi64((__m128i *)(dstp + j), out);
		}
	}
};


class MaskedMergeFilter final : public graphengine::Filter {
	graphengine::FilterDescriptor m_desc = {};
public:
	MaskedMergeFilter(unsigned width, unsigned height)
	{
		m_desc.format.width = width;
		m_desc.format.height = height;
		m_desc.format.bytes_per_sample = 1;

		m_desc.num_deps = 3;
		m_desc.num_planes = 1;
		m_desc.step = 1;
		m_desc.alignment_mask = 7;

		m_desc.flags.in_place = true;
	}

	int version() const noexcept override { return VERSION; }

	const graphengine::FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned i) const noexcept override { return{ i, i + 1 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override { return{ left, right }; }

	void init_context(void *) const noexcept override {}

	void process(const graphengine::BufferDescriptor in[3], const graphengine::BufferDescriptor *out,
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept override
	{
		const uint8_t *src1 = in[0].get_line<uint8_t>(i);
		const uint8_t *src2 = in[1].get_line<uint8_t>(i);
		const uint8_t *mask = in[2].get_line<uint8_t>(i);
		uint8_t *dstp = out->get_line<uint8_t>(i);

		for (unsigned j = left; j < right; j += 8) {
			__m128i src1val = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(src1 + j)), _mm_setzero_si128());
			__m128i src2val = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(src2 + j)), _mm_setzero_si128());
			__m128i maskval = _mm_unpacklo_epi8(_mm_loadl_epi64((const __m128i *)(mask + j)), _mm_setzero_si128());
			__m128i invmask = _mm_sub_epi16(_mm_set1_epi16(255), maskval);

			__m128i val = _mm_add_epi16(_mm_mullo_epi16(src1val, maskval), _mm_mullo_epi16(src2val, invmask));
			val = _mm_mulhi_epu16(val, _mm_set1_epi16(0x8081));
			val = _mm_srli_epi16(val, 7);
			val = _mm_packus_epi16(val, val);
			_mm_storel_epi64((__m128i *)(dstp + j), val);
		}
	}
};


class VirtualPadFilter final : public graphengine::Filter {
	graphengine::FilterDescriptor m_desc = {};
	unsigned m_left;
	unsigned m_right;
	unsigned m_top;
	unsigned m_bottom;
public:
	VirtualPadFilter(unsigned src_width, unsigned src_height, unsigned left, unsigned right, unsigned top, unsigned bottom) :
		m_left{ left },
		m_right{ right },
		m_top{ top },
		m_bottom{ bottom }
	{
		if (UINT_MAX - src_width < left || UINT_MAX - src_width - left < right)
			throw std::runtime_error{ "padded dimensions too large" };
		if (UINT_MAX - src_height < top || UINT_MAX - src_height - top < bottom)
			throw std::runtime_error{ "padded dimensions too large" };

		m_desc.format.width = src_width + left + right;
		m_desc.format.height = src_height + top + bottom;
		m_desc.format.bytes_per_sample = 1;

		m_desc.num_deps = 1;
		m_desc.num_planes = 1;
		m_desc.step = 1;
	}

	int version() const noexcept override { return VERSION; }

	const graphengine::FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned i) const noexcept override
	{
		unsigned src_height = m_desc.format.height - m_top - m_bottom;

		ptrdiff_t srctop = static_cast<ptrdiff_t>(i) - static_cast<ptrdiff_t>(m_top);
		ptrdiff_t srcbot = srctop + 1;
		srctop = std::min(std::max(srctop, static_cast<ptrdiff_t>(0)), static_cast<ptrdiff_t>(src_height));
		srcbot = std::min(std::max(srcbot, static_cast<ptrdiff_t>(0)), static_cast<ptrdiff_t>(src_height));

		return{ static_cast<unsigned>(srctop), static_cast<unsigned>(srcbot) };
	}

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override
	{
		unsigned src_width = m_desc.format.width - m_left - m_right;

		ptrdiff_t srcleft = static_cast<ptrdiff_t>(left) - static_cast<ptrdiff_t>(m_left);
		ptrdiff_t srcright = static_cast<ptrdiff_t>(right) - static_cast<ptrdiff_t>(m_left);
		srcleft = std::min(std::max(srcleft, static_cast<ptrdiff_t>(0)), static_cast<ptrdiff_t>(src_width));
		srcright = std::min(std::max(srcright, static_cast<ptrdiff_t>(0)), static_cast<ptrdiff_t>(src_width));

		return{ static_cast<unsigned>(srcleft), static_cast<unsigned>(srcright) };
	}

	void init_context(void *) const noexcept override {}

	void process(const graphengine::BufferDescriptor *in, const graphengine::BufferDescriptor *out,
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept
	{
		if (i < m_top || i >= m_desc.format.height - m_bottom)
			return;

		const uint8_t *srcp = in->get_line<uint8_t>(i - m_top);
		uint8_t *dstp = out->get_line<uint8_t>(i);

		auto range = get_col_deps(left, right);
		std::memcpy(dstp + m_left + range.first, srcp + range.first, range.second - range.first);
	}
};


class OverlayFilter final : public graphengine::Filter {
	graphengine::FilterDescriptor m_desc = {};
	unsigned m_x0, m_x1, m_y0, m_y1;
public:
	OverlayFilter(unsigned width, unsigned height, unsigned x0, unsigned x1, unsigned y0, unsigned y1) :
		m_x0{ x0 }, m_x1{ x1 }, m_y0{ y0 }, m_y1{ y1 }
	{
		m_desc.format.width = width;
		m_desc.format.height = height;
		m_desc.format.bytes_per_sample = 1;

		m_desc.num_deps = 2;
		m_desc.num_planes = 1;
		m_desc.step = 1;

		// Make the base layer the in-place plane.
		m_desc.inplace_hint.enabled = 1;
		m_desc.inplace_hint.preferred_index = 0;

		m_desc.flags.in_place = true;
	}

	int version() const noexcept override { return VERSION; }

	const graphengine::FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned i) const noexcept override { return{ i, i + 1 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override { return{ left, right }; }

	void init_context(void *) const noexcept override {}

	void process(const graphengine::BufferDescriptor in[2], const graphengine::BufferDescriptor *out,
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept
	{
		const uint8_t *srcp0 = in[0].get_line<uint8_t>(i);
		uint8_t *dstp = out->get_line<uint8_t>(i);

		if (i < m_y0 || i >= m_y1) {
			if (dstp != srcp0)
				std::memcpy(dstp + left, srcp0 + left, right - left);
			return;
		}

		const uint8_t *srcp1 = in[1].get_line<uint8_t>(i);

		// Optimized path for in-place.
		if (dstp == srcp0) {
			unsigned span_left = std::max(left, m_x0);
			unsigned span_right = std::min(right, m_x1);
			if (span_left < span_right)
				std::memcpy(dstp + span_left, srcp1 + span_left, span_right - span_left);
			return;
		}

		// Left of overlay.
		unsigned span0_left = std::min(left, m_x0);
		unsigned span0_right = std::min(right, m_x0);

		// Overlap.
		unsigned span1_left = std::max(left, m_x0);
		unsigned span1_right = std::min(right, m_x1);

		// Right of overlay.
		unsigned span2_left = std::max(left, m_x1);
		unsigned span2_right = std::max(right, m_x1);

		if (span0_left < span0_right)
			std::memcpy(dstp + span0_left, srcp0 + span0_left, span0_right - span0_left);
		if (span1_left < span1_right)
			std::memcpy(dstp + span1_left, srcp1 + span1_left, span1_right - span1_left);
		if (span2_left < span2_right)
			std::memcpy(dstp + span2_left, srcp0 + span2_left, span2_right - span2_left);
	}
};

} // namespace


std::unique_ptr<graphengine::Filter> invoke_boxblur(unsigned width, unsigned height)
{
	return std::make_unique<BoxBlurFilter>(width, height);
}

std::unique_ptr<graphengine::Filter> invoke_sobel(unsigned width, unsigned height)
{
	return std::make_unique<SobelFilter>(width, height);
}

std::unique_ptr<graphengine::Filter> invoke_masked_merge(unsigned width, unsigned height)
{
	return std::make_unique<MaskedMergeFilter>(width, height);
}

std::unique_ptr<graphengine::Filter> invoke_virtual_pad(unsigned src_width, unsigned src_height, unsigned left, unsigned right, unsigned top, unsigned bottom)
{
	return std::make_unique<VirtualPadFilter>(src_width, src_height, left, right, top, bottom);
}

std::unique_ptr<graphengine::Filter> invoke_overlay(unsigned width, unsigned height, unsigned x0, unsigned x1, unsigned y0, unsigned y1)
{
	return std::make_unique<OverlayFilter>(width, height, x0, x1, y0, y1);
}
