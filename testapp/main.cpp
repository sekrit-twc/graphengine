#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <graphengine/filter.h>
#include <graphengine/graph.h>
#include "aligned_malloc.h"
#include "argparse.h"
#include "simplefilters.h"
#include "timer.h"
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
		frame.allocs.push_back(std::shared_ptr<void>(aligned_malloc(planesize, 64), aligned_free));
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


struct Arguments {
	const char *source_path;
	const char *overlay_path;
	const char *out_path;
	unsigned source_w = 512;
	unsigned source_h = 512;
	unsigned overlay_w = 64;
	unsigned overlay_h = 64;
	unsigned times = 1;
	unsigned threads = 1;
	unsigned char pipeline = 1;
	unsigned char autosize = 1;
	unsigned char fusion = 1;
	unsigned char planar = 1;
	unsigned char tile = 1;
	unsigned tile_width = 0;
};

const ArgparseOption program_switches[] = {
	{ OPTION_UINT, "w",     "width",          offsetof(Arguments, source_w),   nullptr, "source width (default: 512)" },
	{ OPTION_UINT, "h",     "height",         offsetof(Arguments, source_h),   nullptr, "source height (default: 512)" },
	{ OPTION_UINT, nullptr, "overlay-width",  offsetof(Arguments, overlay_w),  nullptr, "overlay width (default: 64)" },
	{ OPTION_UINT, nullptr, "overlay-height", offsetof(Arguments, overlay_h),  nullptr, "overlay height (default: 64)" },
	{ OPTION_UINT, "t",     "times",          offsetof(Arguments, times),      nullptr, "number of iterations (default: 1)" },
	{ OPTION_UINT, nullptr, "threads",        offsetof(Arguments, threads),    nullptr, "number of worker threads (default: 1)" },
	{ OPTION_FLAG, nullptr, "pipeline",       offsetof(Arguments, pipeline),   nullptr, "overlap filter execution (default: enabled)" },
	{ OPTION_FLAG, nullptr, "autosize",       offsetof(Arguments, autosize),   nullptr, "minimize internal buffer sizing (default: enabled)" },
	{ OPTION_FLAG, nullptr, "fusion",         offsetof(Arguments, fusion),     nullptr, "enable node fusion (default: enabled)" },
	{ OPTION_FLAG, nullptr, "planar",         offsetof(Arguments, planar),     nullptr, "allow filter chain to run per-plane (default: enabled)" },
	{ OPTION_FLAG, nullptr, "tile",           offsetof(Arguments, tile),       nullptr, "allow filter chain to be run in multiple passes (default: enabled)" },
	{ OPTION_UINT, nullptr, "tile-width",     offsetof(Arguments, tile_width), nullptr, "override tile width (default: auto)" },
	{ OPTION_NULL },
};

const ArgparseOption program_positional[] = {
	{ OPTION_NULL },
};

const ArgparseCommandLine program_commandline = { program_switches, program_positional, "testapp", };


void thread_func(graphengine::Graph *graph, std::atomic_int *counter, unsigned w, unsigned h, unsigned overlay_w, unsigned overlay_h,
                 graphengine::node_id source, graphengine::node_id overlay, graphengine::node_id output)
{
	Frame source_frame = allocate_frame(w, h);
	Frame overlay_frame = allocate_frame(overlay_w, overlay_h);
	Frame result_frame = allocate_frame(w, h);

	graphengine::Graph::Endpoint endpoints[] = {
		{ source, source_frame.buffer },
		{ overlay, overlay_frame.buffer },
		{ output, result_frame.buffer },
	};

	std::shared_ptr<void> tmp{ aligned_malloc(graph->get_tmp_size(), 64), aligned_free };

	while (true) {
		if ((*counter)-- <= 0)
			break;

		graph->run(endpoints, tmp.get());
	}
}

} // namespace


int main(int argc, char **argv)
{
	using graphengine::node_id;

	Arguments args{};
	int ret = argparse_parse(&program_commandline, &args, argc, argv);
	if (ret < 0)
		return ret == ARGPARSE_HELP_MESSAGE ? 0 : ret;

	// Paths are optional.
	if (ret + 0 < argc)
		args.source_path = argv[ret + 0];
	if (ret + 1 < argc)
		args.overlay_path = argv[ret + 1];
	if (ret + 2 < argc)
		args.out_path = argv[ret + 2];

	try {
		Frame source_frame = args.source_path ?  load_bmp(args.source_path) : allocate_frame(args.source_w, args.source_h);
		Frame overlay_frame = args.overlay_path ? load_bmp(args.overlay_path) : allocate_frame(args.overlay_w, args.overlay_h);

		for (unsigned p = 0; p < 3; ++p) {
			overlay_frame.format[p].width = std::min(source_frame.format[p].width, overlay_frame.format[p].width);
			overlay_frame.format[p].height = std::min(source_frame.format[p].height, overlay_frame.format[p].height);
		}

		unsigned source_w = source_frame.format[0].width;
		unsigned source_h = source_frame.format[0].height;
		unsigned overlay_w = overlay_frame.format[0].width;
		unsigned overlay_h = overlay_frame.format[0].height;

		std::vector<std::unique_ptr<graphengine::Filter>> filters;
		graphengine::GraphImpl filtergraph;

		filtergraph.set_pipelining_enabled(args.pipeline);
		filtergraph.set_buffer_sizing_enabled(args.autosize);
		filtergraph.set_fusion_enabled(args.fusion);
		filtergraph.set_planar_enabled(args.planar);
		filtergraph.set_tiling_enabled(args.tile);
		filtergraph.set_tile_width(args.tile_width);

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

		graphengine::Graph::BufferingRequirement buffering = filtergraph.get_buffering_requirement();
		for (unsigned i = 0; i < 3; ++i) {
			printf("endpoint %u: id %d, mask = 0x%x\n", i, buffering[i].id, buffering[i].mask);
		}
		printf("cache footprint: %zu\n", filtergraph.get_cache_footprint(!args.planar));
		printf("working set: %zu\n", filtergraph.get_tmp_size(!args.planar));
		printf("tile width: %u\n", filtergraph.get_tile_width(!args.planar));

		if (args.threads == 1) {
			Frame result_frame = allocate_frame(source_w, source_h);

			graphengine::Graph::Endpoint endpoints[] = {
				{ source, source_frame.buffer },
				{ overlay, overlay_frame.buffer },
				{ output, result_frame.buffer },
			};

			std::shared_ptr<void> tmp{ aligned_malloc(filtergraph.get_tmp_size(), 64), aligned_free };

			Timer timer;
			timer.start();
			for (unsigned n = 0; n < args.times; ++n) {
				filtergraph.run(endpoints, tmp.get());
			}
			timer.stop();
			printf("%u frames in %f s: %f fps\n", args.times, timer.elapsed(), args.times / timer.elapsed());

			if (args.out_path)
				write_bmp(args.out_path, result_frame);
		} else {
			std::vector<std::thread> threadpool;
			unsigned n = args.threads ? args.threads : std::thread::hardware_concurrency();
			std::atomic_int counter{ static_cast<int>(args.times) };

			Timer timer;
			timer.start();
			for (unsigned i = 0; i < n; ++i) {
				threadpool.emplace_back(thread_func, &filtergraph, &counter, source_w, source_h, overlay_w, overlay_h, source, overlay, output);
			}
			for (auto &th : threadpool) {
				th.join();
			}
			timer.stop();
			printf("%u frames in %f s: %f fps\n", args.times, timer.elapsed(), args.times / timer.elapsed());
		}
	} catch (const std::exception &e) {
		std::cerr << e.what() << '\n';
		return 1;
	}

	return 0;
}
