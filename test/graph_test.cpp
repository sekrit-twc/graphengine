#include <climits>
#include <cstdint>
#include <utility>
#include <vector>
#include "graphengine/filter.h"
#include "graphengine/graph.h"
#include "graphengine/types.h"

#include "gtest/gtest.h"

namespace {

class DummyFilter : public graphengine::Filter {
	graphengine::FilterDescriptor m_desc = {};
public:
	DummyFilter(unsigned width, unsigned height)
	{
		m_desc.format = { width, height, 1 };
		m_desc.num_deps = 1;
		m_desc.num_planes = 1;
		m_desc.step = 1;
	}

	int version() const noexcept override { return VERSION; }

	graphengine::FilterDescriptor &mutable_descriptor() { return m_desc; }

	const graphengine::FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned i) const noexcept override { return{ i, i + 1 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override { return{ left, right }; }

	void init_context(void *context) const noexcept override {}

	void process(const graphengine::BufferDescriptor in[], const graphengine::BufferDescriptor out[],
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept override
	{}
};


auto invalid_node_formats()
{
	std::vector<std::pair<unsigned, std::vector<graphengine::PlaneDescriptor>>> formats;

	// Wrong number of planes.
	formats.push_back({ 0, {} });
	formats.push_back({ graphengine::NODE_MAX_PLANES + 1, std::vector<graphengine::PlaneDescriptor>(graphengine::NODE_MAX_PLANES + 1) });

	// Memory limits exceeded.
#if PTRDIFF_MAX / UINT_MAX < 4
	formats.push_back({ 1, { { UINT_MAX, 1, 4 } } });
#endif
#if SIZE_MAX / UINT_MAX / UINT_MAX < 2
	formats.push_back({ 1, { { UINT_MAX, UINT_MAX, 1 } } });
#endif

	// Bad subsampling.
	formats.push_back({ 2, { { 640, 480, 1 }, { 80, 60, 1 } } });
	formats.push_back({ 2, { { 640, 480, 1 }, { 319, 239, 1 } } });

	return formats;
}


TEST(GraphTest, test_add_source)
{
	for (unsigned n = 1; n < graphengine::NODE_MAX_PLANES; ++n) {
		SCOPED_TRACE(n);
		graphengine::GraphImpl graph;

		std::vector<graphengine::PlaneDescriptor> desc(n);
		for (unsigned nn = 0; nn < n; ++nn) {
			desc[nn] = { 640, 480, 1 };
		}

		graphengine::node_id id = graph.add_source(n, desc.data());
		EXPECT_GE(id, 0);
	}
}

TEST(GraphTest, test_add_source_invalid_desc)
{
	for (const auto &format : invalid_node_formats()) {
		graphengine::GraphImpl graph;
		EXPECT_THROW(graph.add_source(format.first, format.second.data()), graphengine::Exception);
	}
}

TEST(GraphTest, test_add_transform)
{
	for (unsigned num_deps = 0; num_deps < graphengine::FILTER_MAX_DEPS; ++num_deps) {
		SCOPED_TRACE(num_deps);

		for (unsigned num_planes = 1; num_planes < graphengine::FILTER_MAX_PLANES; ++num_planes) {
			SCOPED_TRACE(num_planes);

			graphengine::GraphImpl graph;
			for (unsigned n = 0; n < graphengine::FILTER_MAX_DEPS; ++n) {
				std::vector<graphengine::PlaneDescriptor> desc(graphengine::NODE_MAX_PLANES, { 640, 480, 1 });
				graph.add_source(static_cast<unsigned>(desc.size()), desc.data());
			}

			DummyFilter dummy{ 640, 480 };
			dummy.mutable_descriptor().num_deps = num_deps;
			dummy.mutable_descriptor().num_planes = num_planes;

			std::vector<graphengine::node_dep_desc> deps;
			for (unsigned n = 0; n < num_deps; ++n) {
				deps.push_back({ static_cast<graphengine::node_id>(n), n % graphengine::NODE_MAX_PLANES });
			}

			graphengine::node_id id = graph.add_transform(&dummy, deps.data());
			EXPECT_GE(id, 0);
		}
	}
}

TEST(GraphTest, test_add_transform_zero_planes)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	DummyFilter dummy{ 640, 480 };
	dummy.mutable_descriptor().num_planes = 0;

	std::vector<graphengine::node_dep_desc> deps(graphengine::FILTER_MAX_DEPS + 1, { source_id, 0 });
	EXPECT_THROW(graph.add_transform(&dummy, deps.data()), graphengine::Exception);
}

TEST(GraphTest, test_add_transform_excess_deps)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	DummyFilter dummy{ 640, 480 };
	dummy.mutable_descriptor().num_deps = graphengine::FILTER_MAX_DEPS + 1;

	std::vector<graphengine::node_dep_desc> deps(graphengine::FILTER_MAX_DEPS + 1, { source_id, 0 });
	EXPECT_THROW(graph.add_transform(&dummy, deps.data()), graphengine::Exception);
}

TEST(GraphTest, test_add_transform_excess_planes)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	DummyFilter dummy{ 640, 480 };
	dummy.mutable_descriptor().num_planes = graphengine::FILTER_MAX_PLANES + 1;

	graphengine::node_dep_desc dep = { source_id, 0 };
	EXPECT_THROW(graph.add_transform(&dummy, &dep), graphengine::Exception);
}

TEST(GraphTest, test_add_transform_invalid_desc)
{
	for (const auto &format : invalid_node_formats()) {
		if (format.first != 1)
			continue;

		graphengine::GraphImpl graph;
		graphengine::PlaneDescriptor desc{ 640, 480, 1 };
		graphengine::node_id source_id = graph.add_source(1, &desc);

		DummyFilter dummy{ format.second.front().width, format.second.front().height };
		dummy.mutable_descriptor().format.bytes_per_sample = format.second.front().bytes_per_sample;
		graphengine::node_dep_desc dep = { source_id, 0 };
		EXPECT_THROW(graph.add_transform(&dummy, &dep), graphengine::Exception);
	}
}

TEST(GraphTest, test_add_transform_bad_dep)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	DummyFilter dummy{ 640, 480 };
	graphengine::node_dep_desc dep;

	dep = { graphengine::null_node, 0 };
	EXPECT_THROW(graph.add_transform(&dummy, &dep), graphengine::Exception) << "{null, 0}";

	dep = { source_id + 1, 0 };
	EXPECT_THROW(graph.add_transform(&dummy, &dep), graphengine::Exception) << "{1, 0}";

	dep = { source_id, 1 };
	EXPECT_THROW(graph.add_transform(&dummy, &dep), graphengine::Exception) << "{0, 1}";
}

TEST(GraphTest, test_add_transform_dim_mismatch)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	graphengine::PlaneDescriptor desc2{ 639, 479, 1 };
	graphengine::node_id source2_id = graph.add_source(1, &desc2);

	DummyFilter dummy{ 640, 480 };
	dummy.mutable_descriptor().num_deps = 2;

	graphengine::node_dep_desc deps[] = { { source_id, 0 }, { source2_id, 0 } };
	EXPECT_THROW(graph.add_transform(&dummy, deps), graphengine::Exception);
}

TEST(GraphTest, test_add_sink)
{
	graphengine::GraphImpl graph;
	std::vector<graphengine::PlaneDescriptor> desc(graphengine::NODE_MAX_PLANES, { 640, 480, 1 });
	graphengine::node_id source_id = graph.add_source(static_cast<unsigned>(desc.size()), desc.data());

	std::vector<graphengine::node_dep_desc> deps;
	for (unsigned p = 0; p < graphengine::NODE_MAX_PLANES; ++p) {
		deps.push_back({ source_id, p });
	}

	graphengine::node_id sink_id = graph.add_sink(static_cast<unsigned>(deps.size()), deps.data());
	ASSERT_GE(sink_id, 0);
}

TEST(GraphTest, test_add_sink_no_sources)
{
	graphengine::GraphImpl graph;

	DummyFilter dummy{ 640, 480 };
	dummy.mutable_descriptor().num_deps = 0;
	dummy.mutable_descriptor().num_planes = 1;
	graphengine::node_id dummy_id = graph.add_transform(&dummy, nullptr);

	std::vector<graphengine::node_dep_desc> sink_deps{ { dummy_id, 0 } };
	graphengine::node_id sink_id = graph.add_sink(static_cast<unsigned>(sink_deps.size()), sink_deps.data());
	ASSERT_GE(sink_id, 0);
}

TEST(GraphTest, test_add_sink_duplicate_deps)
{
	graphengine::GraphImpl graph;
	std::vector<graphengine::PlaneDescriptor> desc(1, { 640, 480, 1 });
	graphengine::node_id source_id = graph.add_source(static_cast<unsigned>(desc.size()), desc.data());

	DummyFilter dummy{ 640, 480 };
	dummy.mutable_descriptor().num_deps = 1;
	dummy.mutable_descriptor().num_planes = 1;

	std::vector<graphengine::node_dep_desc> dummy_deps{ { source_id, 0 } };
	graphengine::node_id dummy_id = graph.add_transform(&dummy, dummy_deps.data());

	std::vector<graphengine::node_dep_desc> sink_deps{ { source_id, 0 }, { dummy_id, 0 }, { dummy_id, 0 } };
	graphengine::node_id sink_id = graph.add_sink(static_cast<unsigned>(sink_deps.size()), sink_deps.data());
	ASSERT_GE(sink_id, 0);
}

TEST(GraphTest, test_add_sink_zero_planes)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	EXPECT_THROW(graph.add_sink(0, nullptr), graphengine::Exception);
}

TEST(GraphTest, test_add_sink_excess_planes)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);
	std::vector<graphengine::node_dep_desc> deps(graphengine::NODE_MAX_PLANES + 1, { source_id, 0 });

	EXPECT_THROW(graph.add_sink(static_cast<unsigned>(deps.size()), deps.data()), graphengine::Exception);
}

TEST(GraphTest, test_add_sink_bad_subsampling)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source0 = graph.add_source(1, &desc);

	graphengine::PlaneDescriptor desc2{ 319, 239, 1 };
	graphengine::node_id source1 = graph.add_source(1, &desc2);
	graphengine::node_id source2 = graph.add_source(1, &desc2);

	graphengine::node_dep_desc deps[] = { { source0, 0 }, { source1, 0 }, { source2, 0 } };
	EXPECT_THROW(graph.add_sink(3, deps), graphengine::Exception);
}

TEST(GraphTest, test_add_sink_bad_dep)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);

	graphengine::node_dep_desc dep;

	dep = { graphengine::null_node, 0 };
	EXPECT_THROW(graph.add_sink(1, &dep), graphengine::Exception) << "{null, 0}";

	dep = { source_id + 1, 0 };
	EXPECT_THROW(graph.add_sink(1, &dep), graphengine::Exception) << "{1, 0}";

	dep = { source_id, 1 };
	EXPECT_THROW(graph.add_sink(1, &dep), graphengine::Exception) << "{0, 1}";
}

TEST(GraphTest, test_add_sink_twice)
{
	graphengine::GraphImpl graph;
	graphengine::PlaneDescriptor desc{ 640, 480, 1 };
	graphengine::node_id source_id = graph.add_source(1, &desc);
	graphengine::node_dep_desc dep = { source_id, 0 };

	ASSERT_GE(graph.add_sink(1, &dep), 0);
	EXPECT_THROW(graph.add_sink(1, &dep), graphengine::Exception);
}

} // namespace
