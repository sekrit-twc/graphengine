#include <vector>
#include "gtest/gtest.h"
#include "validator.h"

namespace {

void run_test(const std::vector<ScriptStatement> &script)
{
	GraphValidator validator;
	validator.validate(script.data(), script.size());
}


TEST(GraphAndNodeTest, test_noop)
{
	run_test({
		{ "clip", "Source", {}, { 640, 480 } },
		{ "clip", "Sink",   { { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_fewer_planes)
{
	run_test({
		{ "clip", "Source", {}, { 640, 480, 1, 3 } },
		{ "clip", "Sink",   { { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_more_planes)
{
	run_test({
		{ "clip", "Source", {}, { 640, 480, 1, 1 } },
		{ "clip", "Sink",   { { "clip" }, { "clip" }, { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_simple)
{
	run_test({
		{ "clip", "Source", {}, { 640, 480 } },
		{ "clip", "Point",  { { "clip" } } },
		{ "clip", "Point",  { { "clip" } } },
		{ "clip", "Sink",   { { "clip" } } },
	});
}

TEST(GraphAndNodeTest, test_multiple_refs)
{
	run_test({
		{ "source",  "Source", {}, { 640, 480, 1, 1 } },
		{ "point.0", "Point",  { { "source" } } },
		{ "point.1", "Point",  { { "point.0" } } },
		{ "point.2", "Point",  { { "point.1" } } },
		{ "sink",    "Sink",   { { "point.0" }, { "point.1" }, { "point.2" } } },
	});
}

TEST(GraphAndNodeTest, test_masktools_like)
{
	run_test({
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
	run_test({
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
	run_test({
		{ "source",      "Source",     {}, { 640, 480 } },
		{ "preprocess",  "Point",      { { "source" } } },
		{ "unresize.h",  "WholeLine",  { { "preprocess" } }, { 240 } },
		{ "unresize.v",  "WholePlane", { { "unresize.h" } }, { 320, 240 } },
		{ "postprocess", "Point",      { { "unresize.v" } } },
		{ "clip",        "Sink",       { { "postprocess" } } },
	});
}

} // namespace
