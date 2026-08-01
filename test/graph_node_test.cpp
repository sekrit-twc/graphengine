#include <cstdint>
#include <vector>
#include "graphengine/graph.h"
#include "gtest/gtest.h"
#include "validator.h"

namespace {

const bool is_debug = graphengine::GraphImpl::debug_mode();

size_t overhead_size(size_t num_nodes)
{
#if SIZE_MAX > UINT32_MAX
	size_t sz = is_debug ? 24 + 133 * num_nodes : 0 + 117 * num_nodes;
#else
	size_t sz = is_debug ? 12 + 73 * num_nodes : 0 + 65 * num_nodes;
#endif
	return (sz + 63) & ~static_cast<size_t>(63);
}

struct TestCase {
	GraphValidator validator;
	size_t num_nodes = 0;

	explicit TestCase(size_t num_nodes = 8) : num_nodes{ num_nodes } {}

	TestCase &cache_footprint(size_t sz)
	{
		validator.expect_cache_footprint_lt(sz + overhead_size(num_nodes));
		return *this;
	}

	TestCase &cache_footprint_planar(size_t sz)
	{
		validator.expect_cache_footprint_planar_lt(sz + overhead_size(num_nodes));
		return *this;
	}

	TestCase &tmp_size(size_t sz)
	{
		if (!is_debug)
			validator.expect_tmp_size_lt(sz + overhead_size(num_nodes));
		return *this;
	}

	TestCase &tmp_size_planar(size_t sz)
	{
		if (!is_debug)
			validator.expect_tmp_size_planar_lt(sz + overhead_size(num_nodes));
		return *this;
	}

	TestCase &buffering(const std::vector<unsigned> &mask)
	{
		validator.expect_buffering_requirement_eq(mask.data(), mask.size());
		return *this;
	}

	void run_test(const std::vector<ScriptStatement> &script)
	{
		validator.validate(script.data(), script.size());
	}
};


TEST(GraphAndNodeTest, test_noop)
{
	TestCase(3)
		.cache_footprint(640 * 2)
		.tmp_size(0)
		.buffering({ 0, 0 })
	.run_test({
		{ "clip", "Source", {}, { 640, 480 } },
		{ "clip", "Sink",   { { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_fewer_planes)
{
	TestCase(3)
		.cache_footprint(640 * 4)
		.cache_footprint_planar(640 * 2)
		.tmp_size(0)
		.buffering({ 0, 0 })
	.run_test({
		{ "clip", "Source", {}, { 640, 480, 1, 3 } },
		{ "clip", "Sink",   { { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_more_planes)
{
	TestCase(5)
		.cache_footprint(640 * 4)
		.tmp_size(0)
		.buffering({ 0, 0 })
	.run_test({
		{ "clip", "Source", {}, { 640, 480, 1, 1 } },
		{ "clip", "Sink",   { { "clip" }, { "clip" }, { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_simple)
{
	TestCase(4)
		.cache_footprint(640 * 2)
		.tmp_size(0)
		.buffering({ 0, 0 })
	.run_test({
		{ "clip", "Source", {}, { 640, 480 } },
		{ "clip", "Point",  { { "clip" } } },
		{ "clip", "Point",  { { "clip" } } },
		{ "clip", "Sink",   { { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_multiple_refs)
{
	TestCase(5)
		.cache_footprint(640 * 4)
		.tmp_size(0)
		.buffering({ 0, 0 })
	.run_test({
		{ "source",  "Source", {}, { 640, 480 } },
		{ "point.0", "Point",  { { "source" } } },
		{ "point.1", "Point",  { { "point.0" } } },
		{ "point.2", "Point",  { { "point.1" } } },
		{ "sink",    "Sink",   { { "point.0" }, { "point.1" }, { "point.2" } } },
	});
}

TEST(GraphAndNodeTest, test_subsampled)
{
	TestCase(5)
		.cache_footprint((640 * 3 / 2) * 2 * 2)
		.cache_footprint_planar(640 * 2)
		.tmp_size(0)
		.buffering({ 1, 1 })
	.run_test({
		{ "clip", "Source", {}, { 640, 480, 1, 3, 1, 1 } },
		{ "clip", "Sink",   { { "clip", 0 }, { "clip", 1 }, { "clip", 2 } } },
	});
}

TEST(GraphAndNodeTest, test_masktools_like)
{
	TestCase(14)
		.cache_footprint((640 * 3 / 2) * 8 + 640 * 5 + 320 * 5 * 2 + (640 * 3 / 2) * 2)
		.cache_footprint_planar(640 * 4 + 640 * 4 + 640 + 640)
		.tmp_size(640 * 5 + 320 * 5 * 2)
		.tmp_size_planar(640 * 5)
		.buffering({ 7, 1 })
	.run_test({
		{ "source",  "Source",      {}, { 640, 480, 1, 3, 1, 1 } },
		{ "blur1.0", "Convolution", { { "source", 0 } } },
		{ "blur1.1", "Convolution", { { "source", 1 } } },
		{ "blur1.2", "Convolution", { { "source", 2 } } },
		{ "blur2.0", "Convolution", { { "blur1.0" } } },
		{ "blur2.1", "Convolution", { { "blur1.1" } } },
		{ "blur2.2", "Convolution", { { "blur1.2" } } },
		{ "sobel.0", "Convolution", { { "source", 0 } } },
		{ "sobel.1", "Convolution", { { "source", 1 } } },
		{ "sobel.2", "Convolution", { { "source", 2 } } },
		{ "merge.0", "Merge",       { { "source", 0 }, { "blur2.0" }, { "sobel.0" } } },
		{ "merge.1", "Merge",       { { "source", 1 }, { "blur2.1" }, { "sobel.1" } } },
		{ "merge.2", "Merge",       { { "source", 2 }, { "blur2.2" }, { "sobel.2" } } },
		{ "clip",    "Sink",        { { "merge.0" }, { "merge.1" }, { "merge.2" } } },
	});
}

TEST(GraphAndNodeTest, test_zlib_like)
{
	TestCase(7)
		.cache_footprint((640 * 3 / 2) * 16 + 640 * 8 * 2 + 640 * 3)
		.tmp_size(640 * 8 * 2)
		.buffering({ 15, 0 })
	.run_test({
		{ "source",       "Source",     {}, { 640, 480, 1, 3, 1, 1 } },
		{ "chroma.422.1", "ScaleH",     { { "source", 1 } }, { 640 } },
		{ "chroma.444.1", "ScaleV",     { { "chroma.422.1" } }, { 480 } },
		{ "chroma.422.2", "ScaleH",     { { "source", 2 } }, { 640 } },
		{ "chroma.444.2", "ScaleV",     { { "chroma.422.2" } }, { 480 } },
		{ "colorspace",   "Colorspace", { { "source", 0 }, { "chroma.444.1" }, { "chroma.444.2" } } },
		{ "clip",         "Sink",       { { "colorspace", 0 }, { "colorspace", 1 }, { "colorspace", 2 } } },
	});
}

TEST(GraphAndNodeTest, test_unresize_like)
{
	TestCase(6)
		.cache_footprint(640 + 640 + 320 * 480 + 320 * 240)
		.tmp_size(640 + 320 * 480)
		.buffering({ 0, graphengine::BUFFER_MAX })
	.run_test({
		{ "source",      "Source",     {}, { 640, 480 } },
		{ "preprocess",  "Point",      { { "source" } } },
		{ "unresize.h",  "WholeLine",  { { "preprocess" } }, { 320 } },
		{ "unresize.v",  "WholePlane", { { "unresize.h" } }, { 320, 240 } },
		{ "postprocess", "Point",      { { "unresize.v" } } },
		{ "clip",        "Sink",       { { "postprocess" } } },
	});
}

TEST(GraphAndNodeTest, test_blocked_in_place)
{
	TestCase(8)
		.cache_footprint(640 * 3 * 4 + 320 * 3 * 8 + 320 * 8 + 320 * 4 * 2)
		.tmp_size(320 * 3 * 8)
		.buffering({ 3, 7 })
	.run_test({
		{ "source",     "Source",     {}, { 640, 480, 1, 3, 0, 0 } },
		{ "resizeh.0",  "ScaleH",     { { "source", 0 } }, { 320, 4 } },
		{ "resizeh.1",  "ScaleH",     { { "source", 1 } }, { 320, 4 } },
		{ "resizeh.2",  "ScaleH",     { { "source", 2 } }, { 320, 4 } },
		{ "colorspace", "Colorspace", { { "resizeh.0" }, { "resizeh.1"}, { "resizeh.2"} } },
		{ "resizev.1",  "ScaleV",     { { "colorspace", 1 } }, { 240, 4 } },
		{ "resizev.2",  "ScaleV",     { { "colorspace", 2 } }, { 240, 4 } },
		{ "clip",       "Sink",       { { "colorspace", 0}, { "resizev.1" }, { "resizev.2" } } },
	});
}

} // namespace
