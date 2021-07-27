#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <malloc.h>
#include <graphengine/filter.h>
#include <graphengine/graph.h>
#include "simplefilters.h"
#include "win32_bitmap.h"

// Example script:
//
// source = Source()
// overlay = Source()
// source_blur1 = Blur(source)
// source_blur2 = Blur(source_blur1)
// edge_mask = Sobel(source)
// edge_blur = MaskedMerge(source, source_blur2, edge_mask)
// result = Overlay(edge_blur, overlay, x=..., y=...)

namespace {

struct Frame {
	std::vector<std::shared_ptr<void>> allocs;
	graphengine::BufferDescriptor buffer[3];
	graphengine::PlaneDescriptor format[3];
};

Frame allocate_frame(unsigned width, unsigned height)
{
	Frame frame{};

	frame.format[0].width = width;
	frame.format[0].height = height;
	frame.format[0].bytes_per_sample = 1;

	frame.format[1] = frame.format[0];
	frame.format[2] = frame.format[0];

	size_t rowsize = (static_cast<size_t>(frame.format[0].width) + 63) & ~static_cast<size_t>(63);
	size_t planesize = rowsize * frame.format[0].height;

	for (unsigned p = 0; p < 3; ++p) {
		frame.allocs.push_back(std::shared_ptr<void>(_aligned_malloc(planesize, 64), _aligned_free));
		frame.buffer[p].ptr = frame.allocs.back().get();
		frame.buffer[p].stride = rowsize;
		frame.buffer[p].mask = graphengine::BUFFER_MAX;
	}

	return frame;
}

Frame load_bmp(const char *path)
{
	WindowsBitmap bmp{ path, WindowsBitmap::READ_TAG };
	Frame frame = allocate_frame(bmp.width(), bmp.height());

	const uint8_t *srcp = bmp.read_ptr();
	uint8_t *dstp0 = static_cast<uint8_t *>(frame.buffer[0].ptr);
	uint8_t *dstp1 = static_cast<uint8_t *>(frame.buffer[1].ptr);
	uint8_t *dstp2 = static_cast<uint8_t *>(frame.buffer[2].ptr);
	unsigned step = bmp.bit_count() / 8;

	for (unsigned i = 0; i < frame.format[0].height; ++i) {
		for (unsigned j = 0; j < frame.format[0].width; ++j) {
			dstp0[j] = srcp[j * step + 2];
			dstp1[j] = srcp[j * step + 1];
			dstp2[j] = srcp[j * step + 0];
		}

		srcp += bmp.stride();
		dstp0 += frame.buffer[0].stride;
		dstp1 += frame.buffer[1].stride;
		dstp2 += frame.buffer[2].stride;
	}

	return frame;
}

void write_bmp(const char *path, const Frame &frame)
{
	WindowsBitmap bmp{ path, static_cast<int>(frame.format[0].width), static_cast<int>(frame.format[0].height), 32 };

	const uint8_t *srcp0 = static_cast<uint8_t *>(frame.buffer[0].ptr);
	const uint8_t *srcp1 = static_cast<uint8_t *>(frame.buffer[1].ptr);
	const uint8_t *srcp2 = static_cast<uint8_t *>(frame.buffer[2].ptr);
	uint8_t *dstp = bmp.write_ptr();

	for (unsigned i = 0; i < frame.format[0].height; ++i) {
		for (unsigned j = 0; j < frame.format[0].width; ++j) {
			dstp[j * 4 + 0] = srcp2[j];
			dstp[j * 4 + 1] = srcp1[j];
			dstp[j * 4 + 2] = srcp0[j];
		}

		srcp0 += frame.buffer[0].stride;
		srcp1 += frame.buffer[1].stride;
		srcp2 += frame.buffer[2].stride;
		dstp += bmp.stride();
	}
}

} // namespace


int main(int argc, char **argv)
{
	using graphengine::node_id;

	if (argc < 4)
		return 1;

	try {
		Frame source_frame = load_bmp(argv[1]);
		Frame overlay_frame = load_bmp(argv[2]);

		for (unsigned p = 0; p < 3; ++p) {
			overlay_frame.format[p].width = std::min(source_frame.format[p].width, overlay_frame.format[p].width);
			overlay_frame.format[p].height = std::min(source_frame.format[p].height, overlay_frame.format[p].height);
		}

		unsigned source_w = source_frame.format[0].width;
		unsigned source_h = source_frame.format[0].height;
		unsigned overlay_w = overlay_frame.format[0].width;
		unsigned overlay_h = overlay_frame.format[0].height;

		std::vector<std::unique_ptr<graphengine::Filter>> filters;
		graphengine::Graph filtergraph;

		// source = Source()
		node_id source = filtergraph.add_source(3, source_frame.format);
		// overlay = Source()
		node_id overlay = filtergraph.add_source(3, overlay_frame.format);

		// source_blur1 = BoxBlur(source)
		node_id source_blur1[3];
		{
			std::unique_ptr<graphengine::Filter> blurfilter = invoke_boxblur(source_w, source_h);
			for (unsigned p = 0; p < 3; ++p) {
				graphengine::node_dep_desc deps[1] = { { source, p } };
				source_blur1[p] = filtergraph.add_transform(blurfilter.get(), deps);
			}
			filters.push_back(std::move(blurfilter));
		}

		// source_blur2 = BoxBlur(source_blur1)
		node_id source_blur2[3];
		{
			std::unique_ptr<graphengine::Filter> blurfilter = invoke_boxblur(source_w, source_h);
			for (unsigned p = 0; p < 3; ++p) {
				graphengine::node_dep_desc deps[1] = { { source_blur1[p], 0 } };
				source_blur2[p] = filtergraph.add_transform(blurfilter.get(), deps);
			}
			filters.push_back(std::move(blurfilter));
		}

		// edge_mask = Sobel(source)
		node_id edge_mask[3];
		{
			std::unique_ptr<graphengine::Filter> sobelfilter = invoke_sobel(source_w, source_h);
			for (unsigned p = 0; p < 3; ++p) {
				graphengine::node_dep_desc deps[1] = { { source, p } };
				edge_mask[p] = filtergraph.add_transform(sobelfilter.get(), deps);
			}
			filters.push_back(std::move(sobelfilter));
		}

		// edge_blur = MaskedMerge(source, source_blur2, edge_mask)
		node_id edge_blur[3];
		{
			std::unique_ptr<graphengine::Filter> mergefilter = invoke_masked_merge(source_w, source_h);
			for (unsigned p = 0; p < 3; ++p) {
				graphengine::node_dep_desc deps[3] = { { source, p }, { source_blur2[p], 0 }, { edge_mask[p], 0 } };
				edge_blur[p] = filtergraph.add_transform(mergefilter.get(), deps);
			}
			filters.push_back(std::move(mergefilter));
		}

		// result = Overlay(edge_blur, overlay, x=..., y=...)
		node_id result[3];
		{
			node_id overlaypadded[3];
			unsigned left = (source_w - overlay_w) / 2;
			unsigned top = (source_h - overlay_h) / 2;
			std::unique_ptr<graphengine::Filter> addbordersfilter = invoke_virtual_pad(
				overlay_w, overlay_h, left, source_w - overlay_w - left, top, source_h - overlay_h - top);
			for (unsigned p = 0; p < 3; ++p) {
				graphengine::node_dep_desc deps[1] = { { overlay, p } };
				overlaypadded[p] = filtergraph.add_transform(addbordersfilter.get(), deps);
			}
			filters.push_back(std::move(addbordersfilter));

			unsigned overlay_left = (source_w - overlay_w) / 2;
			unsigned overlay_top = (source_h - overlay_h) / 2;
			std::unique_ptr<graphengine::Filter> overlayfilter = invoke_overlay(
				source_w, source_h, overlay_left, source_w - overlay_left, overlay_top, source_h - overlay_top);
			for (unsigned p = 0; p < 3; ++p) {
				graphengine::node_dep_desc deps[2] = { { edge_blur[p], 0 }, { overlaypadded[p], 0 } };
				result[p] = filtergraph.add_transform(overlayfilter.get(), deps);
			}
			filters.push_back(std::move(overlayfilter));
		}

		graphengine::node_dep_desc output_nodes[3] = { { result[0], 0 }, { result[1], 0 }, { result[2], 0 } };
		node_id output = filtergraph.add_sink(3, output_nodes);

		Frame result_frame = allocate_frame(source_w, source_h);
		graphengine::Graph::EndpointConfiguration endpoints{};

		endpoints[0].id = source;
		std::copy_n(source_frame.buffer, 3, endpoints[0].buffer);

		endpoints[1].id = overlay;
		std::copy_n(overlay_frame.buffer, 3, endpoints[1].buffer);

		endpoints[2].id = output;
		std::copy_n(result_frame.buffer, 3, endpoints[2].buffer);

		std::shared_ptr<void> tmp{ _aligned_malloc(filtergraph.get_tmp_size(), 64), _aligned_free };
		filtergraph.run(endpoints, tmp.get());

		write_bmp(argv[3], result_frame);
	} catch (const std::exception &e) {
		std::cerr << e.what() << '\n';
		return 1;
	}

	return 0;
}
