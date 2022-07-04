#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <malloc.h>
#include "graphengine/graph.h"
#include "graphengine/types.h"

#include "gtest/gtest.h"
#include "validator.h"

namespace {

using graphengine::node_id;
using graphengine::PlaneDescriptor;


class SimpleDependencyFilter : public ValidationFilter {
public:
	SimpleDependencyFilter(const char *name, const PlaneDescriptor &format, unsigned num_deps, unsigned num_planes) :
		ValidationFilter{ name }
	{
		desc.format = format;
		desc.num_deps = num_deps;
		desc.num_planes = num_planes;
		desc.step = 1;

		desc.flags.in_place = true;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept override { return{ i, i + 1 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override { return { left, right }; }
};


class FakeFilter : public ValidationFilter {
public:
	FakeFilter(const char *name, unsigned step) : ValidationFilter{ name }
	{
		desc.step = step;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept { return{}; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept { return {}; }
};


class GenerateFilter : public ValidationFilter {
public:
	static std::unique_ptr<GenerateFilter> create(const char *name, const PlaneDescriptor &, const std::vector<unsigned> &args)
	{
		if (args.size() < 2)
			throw std::invalid_argument{ "bad script" };

		unsigned width = args[0];
		unsigned height = args[1];
		unsigned bytes_per_sample = args.size() >= 3 ? args[2] : 1;
		return std::make_unique<GenerateFilter>(name, PlaneDescriptor{ width, height, bytes_per_sample });
	}

	GenerateFilter(const char *name, const PlaneDescriptor &format) : ValidationFilter{ name }
	{
		desc.format = format;
		desc.num_deps = 0;
		desc.num_planes = 1;
		desc.step = 1;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept { return{ 0, 0 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept { return { 0, 0 }; }
};


class PointFilter : public SimpleDependencyFilter {
public:
	static std::unique_ptr<PointFilter> create(const char *name, const PlaneDescriptor &source_format, const std::vector<unsigned> &args)
	{
		unsigned bytes_per_sample = args.size() >= 1 ? args[0] : source_format.bytes_per_sample;
		return std::make_unique<PointFilter>(name, source_format, bytes_per_sample);
	}

	PointFilter(const char *name, const PlaneDescriptor &source_format, unsigned bytes_per_sample) :
		SimpleDependencyFilter{ name, source_format, 1, 1 }
	{
		desc.format.bytes_per_sample = bytes_per_sample;
		desc.flags.in_place = bytes_per_sample == source_format.bytes_per_sample;
	}
};


class ConvolutionFilter : public ValidationFilter {
	unsigned m_kernel_dim;
	unsigned m_support;
public:
	static std::unique_ptr<ConvolutionFilter> create(const char *name, const PlaneDescriptor &format, const std::vector<unsigned> &args)
	{
		unsigned n = args.size() >= 1 ? args[0] : 3;
		return std::make_unique<ConvolutionFilter>(name, format, n);
	}

	ConvolutionFilter(const char *name, const PlaneDescriptor &format, unsigned n) :
		ValidationFilter{ name },
		m_kernel_dim{ n },
		m_support{ n / 2 }
	{
		desc.format = format;
		desc.num_deps = 1;
		desc.num_planes = 1;
		desc.step = 1;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept
	{
		return{ std::max(i, m_support) - m_support, std::min(i + 1 + m_support, desc.format.height) };
	}

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept
	{
		return{ std::max(left, m_support) - m_support, std::min(right + m_support, desc.format.width) };
	}
};


class ColorspaceFilter : public SimpleDependencyFilter {
public:
	static std::unique_ptr<ColorspaceFilter> create(const char *name, const PlaneDescriptor &format, const std::vector<unsigned> &)
	{
		return std::make_unique<ColorspaceFilter>(name, format);
	}

	ColorspaceFilter(const char *name, const PlaneDescriptor &format) : SimpleDependencyFilter{ name, format, 3, 3 } {}
};


class MergeFilter : public SimpleDependencyFilter {
public:
	static std::unique_ptr<MergeFilter> create(const char *name, const PlaneDescriptor &format, const std::vector<unsigned> &)
	{
		return std::make_unique<MergeFilter>(name, format);
	}

	MergeFilter(const char *name, const PlaneDescriptor &format) : SimpleDependencyFilter{ name, format, 3, 1 } {}
};


class Split2Filter : public SimpleDependencyFilter {
public:
	static std::unique_ptr<Split2Filter> create(const char *name, const PlaneDescriptor &format, const std::vector<unsigned> &)
	{
		return std::make_unique<Split2Filter>(name, format);
	}

	Split2Filter(const char *name, const PlaneDescriptor &format) : SimpleDependencyFilter{ name, format, 1, 2 } {}
};


class ScaleVFilter : public ValidationFilter {
	unsigned m_src_height;
public:
	static std::unique_ptr<ScaleVFilter> create(const char *name, const PlaneDescriptor &source_format, const std::vector<unsigned> &args)
	{
		if (args.size() < 1)
			throw std::invalid_argument{ "bad script" };

		unsigned n = args[0];
		return std::make_unique<ScaleVFilter>(name, source_format, n);
	}

	ScaleVFilter(const char *name, const PlaneDescriptor &source_format, unsigned height) :
		ValidationFilter{ name },
		m_src_height{ source_format.height }
	{
		desc.format = source_format;
		desc.format.height = height;

		desc.num_deps = 1;
		desc.num_planes = 1;
		desc.step = 1;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept
	{
		double ratio = static_cast<double>(m_src_height) / desc.format.height;
		double coord = i * ratio;

		unsigned top = std::lrint(std::floor(coord));
		unsigned bot = std::min(top + 2, m_src_height);
		return{ top, bot };
	}

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept { return{ left, right }; }
};


class ScaleHFilter : public ValidationFilter {
	unsigned m_src_width;
public:
	static std::unique_ptr<ScaleHFilter> create(const char *name, const PlaneDescriptor &source_format, const std::vector<unsigned> &args)
	{
		if (args.size() < 1)
			throw std::invalid_argument{ "bad script" };

		unsigned n = args[0];
		return std::make_unique<ScaleHFilter>(name, source_format, n);
	}


	ScaleHFilter(const char *name, const PlaneDescriptor &source_format, unsigned width) :
		ValidationFilter{ name },
		m_src_width{ source_format.width }
	{
		desc.format = source_format;
		desc.format.width = width;

		desc.num_deps = 1;
		desc.num_planes = 1;
		desc.step = 4;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept
	{
		unsigned i_mod4 = i & ~3U;
		return{ i_mod4, std::min(i_mod4 + 4, desc.format.height) };
	}

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept
	{
		double ratio = static_cast<double>(m_src_width) / desc.format.width;
		double coord_left = left * ratio;
		double coord_right = right * ratio;

		unsigned depleft = std::lrint(std::floor(coord_left));
		unsigned depright = std::min(static_cast<unsigned>(std::lrint(std::floor(coord_right)) + 1), m_src_width);
		return{ depleft, depright };
	}
};


class WholeLineFilter : public ValidationFilter {
	unsigned m_src_width;
public:
	static std::unique_ptr<WholeLineFilter> create(const char *name, const PlaneDescriptor &source_format, const std::vector<unsigned> &args)
	{
		unsigned n = args.size() >= 1 ? args[0] : source_format.width;
		return std::make_unique<WholeLineFilter>(name, source_format, n);
	}

	WholeLineFilter(const char *name, const PlaneDescriptor &source_format, unsigned width) :
		ValidationFilter{ name },
		m_src_width{ source_format.width }
	{
		desc.format = source_format;
		desc.format.width = width;

		desc.num_deps = 1;
		desc.num_planes = 1;
		desc.step = 1;

		desc.flags.stateful = 1;
		desc.flags.entire_row = 1;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept { return{ i, i + 1 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept { return{ 0, m_src_width }; }
};


class WholePlaneFilter : public ValidationFilter {
	PlaneDescriptor m_src_format;
public:
	static std::unique_ptr<WholePlaneFilter> create(const char *name, const PlaneDescriptor &source_format, const std::vector<unsigned> &args)
	{
		if (args.size() < 2)
			throw std::invalid_argument{ "bad script" };

		unsigned width = args[0];
		unsigned height = args[1];
		unsigned bytes_per_sample = args.size() >= 3 ? args[2] : 1;
		return std::make_unique<WholePlaneFilter>(name, source_format, PlaneDescriptor{ width, height, bytes_per_sample });
	}

	WholePlaneFilter(const char *name, const PlaneDescriptor &source_format, const PlaneDescriptor &format) :
		ValidationFilter{ name },
		m_src_format{ source_format }
	{
		desc.format = format;
		desc.num_deps = 1;
		desc.num_planes = 1;
		desc.step = UINT_MAX;

		desc.flags.stateful = 1;
		desc.flags.entire_row = 1;
		desc.flags.entire_col = 1;
	}

	pair_unsigned get_row_deps(unsigned i) const noexcept { return{ 0, m_src_format.height }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept { return { 0, m_src_format.width }; }
};

} // namespace


class ValidationFilter::ValidationState {
	friend class ValidationFilter;

	struct BufferRecord {
		const ValidationFilter *writer = nullptr;
		unsigned plane = UINT_MAX;
		unsigned row = UINT_MAX;
		unsigned left = UINT_MAX;
		unsigned right = UINT_MAX;
	};

	struct FilterHistoryRecord {
		unsigned row = UINT_MAX;
		unsigned left = UINT_MAX;
		unsigned right = UINT_MAX;
	};

	std::vector<std::shared_ptr<void>> m_endpoint_allocs;
	std::unordered_map<const void *, std::vector<BufferRecord>> m_buffer_state;
	std::unordered_map<const ValidationFilter *, std::vector<FilterHistoryRecord>> m_filter_history;
	std::unordered_map<const ValidationFilter *, std::vector<std::pair<const ValidationFilter *, unsigned>>> m_filter_parents;

	graphengine::BufferDescriptor m_endpoint_buffers[graphengine::GRAPH_MAX_ENDPOINTS][graphengine::NODE_MAX_PLANES] = {};
	graphengine::Graph::Endpoint m_endpoints[graphengine::GRAPH_MAX_ENDPOINTS] = {};

	void clear_buffer_records()
	{
		for (auto &buffer : m_buffer_state) {
			buffer.second.assign(buffer.second.size(), {});
		}
	}

	BufferRecord &lookup_buffer_record(const graphengine::BufferDescriptor &buffer, unsigned i)
	{
		std::vector<BufferRecord> &records = m_buffer_state[buffer.ptr];
		i &= buffer.mask;

		if (records.size() < i)
			records.resize(i);

		return records[i];
	}

	void verify_buffer_contents(const graphengine::BufferDescriptor &buffer, const ValidationFilter *writer, unsigned p, unsigned i, unsigned left, unsigned right)
	{
		BufferRecord &record = lookup_buffer_record(buffer, i);
		ASSERT_NE(record.writer, nullptr);
		ASSERT_EQ(record.writer, writer) << record.writer->name << " " << writer->name;
		ASSERT_EQ(record.plane, p);
		ASSERT_LE(left, record.left);
		ASSERT_GE(record.right, right);
	}
public:
	ValidationState(
		const std::unordered_map<node_id, const ValidationFilter *> &filter_table,
		const std::unordered_map<node_id, std::vector<PlaneDescriptor>> &endpoints,
		const std::unordered_map<const ValidationFilter *, std::vector<std::pair<const ValidationFilter *, unsigned>>> &filter_parents
	) :
		m_filter_parents(filter_parents)
	{
		auto buffer_iter = m_endpoint_buffers;
		auto iter = m_endpoints;

		for (const auto &endpoint : endpoints) {
			iter->id = endpoint.first;

			for (size_t p = 0; p < endpoint.second.size(); ++p) {
				size_t rowsize = (endpoint.second[p].width * endpoint.second[p].bytes_per_sample + 63) & ~static_cast<size_t>(63);
				m_endpoint_allocs.emplace_back(_aligned_malloc(rowsize * endpoint.second[p].height, 64), _aligned_free);
				(*buffer_iter)[p] = { m_endpoint_allocs.back().get(), static_cast<ptrdiff_t>(rowsize), graphengine::BUFFER_MAX };
			}
			iter->buffer = *buffer_iter;

			++buffer_iter;
			++iter;
		}
	}

	const graphengine::Graph::Endpoint *endpoints() { return m_endpoints; }
};

void ValidationFilter::init_context(void *context) const noexcept
{
	state->clear_buffer_records();
	state->m_filter_history[this].clear();
}

void ValidationFilter::process(const graphengine::BufferDescriptor in[], const graphengine::BufferDescriptor out[],
                               unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept
{
	if (::testing::Test::HasFailure())
		return;

	SCOPED_TRACE(name + " " + std::to_string(i) + " " + std::to_string(left) + " " + std::to_string(right));

	// Check bounds.
	ASSERT_LT(i, desc.format.height);
	ASSERT_LE(left, desc.format.width);
	ASSERT_LE(right, desc.format.width);

	// Check conformance with flags.
	ASSERT_EQ(0U, left & desc.alignment_mask);
	if (right < desc.format.width)
		ASSERT_EQ(0U, right & desc.alignment_mask);

	if (desc.flags.stateful && !state->m_filter_history[this].empty())
		ASSERT_EQ(state->m_filter_history[this].back().row + desc.step, i);

	if (!desc.flags.in_place) {
		for (unsigned p = 0; p < std::min(desc.num_deps, desc.num_planes); ++p) {
			ASSERT_NE(in[p].ptr, out[p].ptr);
		}
	}

	if (desc.flags.entire_row) {
		ASSERT_EQ(0U, left);
		ASSERT_EQ(desc.format.width, right);
	}

	if (desc.flags.entire_col) {
		ASSERT_EQ(0U, i);
		ASSERT_TRUE(state->m_filter_history[this].empty());
	}

	// Check for redundant calls.
	{
		auto it = std::find_if(state->m_filter_history[this].begin(), state->m_filter_history[this].end(),
			[&](const ValidationState::FilterHistoryRecord &record) {
			return i > record.row && i < record.row + desc.step;
		});
		if (it != state->m_filter_history[this].end()) {
			ADD_FAILURE() << "already generated by previous call: " << it->row << " " << it->left << " " << it->right;
			return;
		}
	}

	// Check presence of dependencies.
	pair_unsigned row_deps = get_row_deps(i);
	pair_unsigned col_deps = get_col_deps(left, right);
	for (unsigned p = 0; p < desc.num_deps; ++p) {
		const auto &parent = state->m_filter_parents[this][p];

		// Source buffers are unverifiable.
		if (dynamic_cast<const FakeFilter *>(parent.first))
			continue;

		for (unsigned i = row_deps.first; i < row_deps.second; ++i) {
			state->verify_buffer_contents(in[p], parent.first, parent.second, i, col_deps.first, col_deps.second);
			if (::testing::Test::HasFailure())
				return;
		}
	}

	// Update state and history.
	for (unsigned p = 0; p < desc.num_planes; ++p) {
		auto &buffer = state->m_buffer_state[out[p].ptr];

		if (buffer.size() < std::min(out[p].mask, desc.format.height - 1) + 1)
			buffer.resize(std::min(out[p].mask, desc.format.height - 1) + 1);

		for (unsigned ii = i; ii < std::min(i + desc.step, desc.format.height); ++ii) {
			unsigned idx = ii & out[p].mask;
			buffer[idx] = { this, p, ii, left, right };
		}
	}
	state->m_filter_history[this].push_back({ i, left, right });
}


class GraphValidator::Script {
	struct Variable {
		node_id id = graphengine::null_node;
		std::vector<PlaneDescriptor> planes;
	};

	std::unordered_map<std::string, Variable> m_vars;
	std::unordered_map<node_id, std::vector<PlaneDescriptor>> m_endpoints;

	const std::unordered_map<std::string, filter_factory> *m_factories;
	std::vector<std::unique_ptr<ValidationFilter>> *m_filters;

	std::unordered_map<node_id, const ValidationFilter *> m_filter_table;
	std::unordered_map<const ValidationFilter *, std::vector<std::pair<const ValidationFilter *, unsigned>>> m_filter_parents;

	graphengine::Graph *m_graph;

	void eval_source(const ScriptStatement &statement)
	{
		if (statement.args.size() < 2)
			throw std::invalid_argument{ "bad script" };

		PlaneDescriptor main_desc;
		main_desc.width = statement.args[0];
		main_desc.height = statement.args[1];
		main_desc.bytes_per_sample = statement.args.size() >= 3 ? statement.args[2] : 1;

		unsigned num_planes = statement.args.size() >= 4 ? statement.args[3] : 1;
		unsigned subsample_w = statement.args.size() >= 5 ? statement.args[4] : 0;
		unsigned subsample_h = statement.args.size() >= 6 ? statement.args[5] : 0;

		std::vector<PlaneDescriptor> desc(num_planes, main_desc);
		for (unsigned p = 1; p < std::min(num_planes, 3U); ++p) {
			desc[p].width >>= subsample_w;
			desc[p].height >>= subsample_h;
		}

		node_id id = m_graph->add_source(num_planes, desc.data());
		m_vars[statement.result] = { id, desc };
		m_endpoints[id] = std::move(desc);

		auto filter = std::make_unique<FakeFilter>(statement.result.c_str(), 1U << subsample_h);
		m_filter_table[id] = filter.get();
		m_filters->push_back(std::move(filter));
	}

	void eval_sink(const ScriptStatement &statement)
	{
		std::vector<graphengine::node_dep_desc> deps;
		std::vector<PlaneDescriptor> desc;

		for (const auto &parent : statement.parents) {
			const auto &var = m_vars.at(parent.name);
			deps.push_back({ var.id, parent.plane });
			desc.push_back(var.planes[parent.plane]);
		}

		node_id id = m_graph->add_sink(static_cast<unsigned>(deps.size()), deps.data());
		m_endpoints[id] = std::move(desc);

		auto filter = std::make_unique<FakeFilter>(statement.result.c_str(), 0);
		m_filter_table[id] = filter.get();
		m_filters->push_back(std::move(filter));
	}

	void eval_transform(const ScriptStatement &statement)
	{
		std::vector<graphengine::node_dep_desc> deps;
		PlaneDescriptor parent_desc = {};

		for (const auto &parent : statement.parents) {
			const auto &var = m_vars.at(parent.name);
			deps.push_back({ var.id, parent.plane });

			if (!parent_desc.width)
				parent_desc = var.planes[parent.plane];
		}

		std::unique_ptr<ValidationFilter> filter = m_factories->at(statement.filter)(
			statement.result.c_str(), parent_desc, statement.args);
		if (deps.size() != filter->descriptor().num_deps)
			throw std::invalid_argument{ "bad script" };

		node_id id = m_graph->add_transform(filter.get(), deps.data());
		std::vector<PlaneDescriptor> desc(filter->descriptor().num_planes, filter->descriptor().format);
		m_vars[statement.result] = { id, desc };

		m_filter_table[id] = filter.get();
		for (const auto &dep : deps) {
			m_filter_parents[filter.get()].push_back({ m_filter_table.at(dep.id), dep.plane });
		}
		m_filters->push_back(std::move(filter));
	}
public:
	Script(
		const std::unordered_map<std::string, filter_factory> *factory,
		std::vector<std::unique_ptr<ValidationFilter>> *filters,
		graphengine::Graph *graph
	) :
		m_factories{ factory }, m_filters { filters }, m_graph{ graph }
	{}

	void eval(const ScriptStatement &statement)
	{
		if (statement.filter == SOURCE)
			eval_source(statement);
		else if (statement.filter == SINK)
			eval_sink(statement);
		else
			eval_transform(statement);
	}

	const std::unordered_map<node_id, const ValidationFilter *> &filter_table() const { return m_filter_table; }

	const std::unordered_map<
		const ValidationFilter *,
		std::vector<std::pair<const ValidationFilter *, unsigned>>
	> &filter_parents() const { return m_filter_parents; }

	const std::unordered_map<node_id, std::vector<PlaneDescriptor>> &endpoints() const { return m_endpoints; }
};

GraphValidator::GraphValidator()
{
	m_factories[POINT_FILTER] = PointFilter::create;
	m_factories[CONVOLUTION_FILTER] = ConvolutionFilter::create;
	m_factories[COLORSPACE_FILTER] = ColorspaceFilter::create;
	m_factories[MERGE_FILTER] = MergeFilter::create;
	m_factories[SPLIT2_FILTER] = Split2Filter::create;
	m_factories[SCALEV_FILTER] = ScaleVFilter::create;
	m_factories[SCALEH_FILTER] = ScaleHFilter::create;
	m_factories[RECURSIVE_FILTER] = WholeLineFilter::create;
	m_factories[FRAME_FILTER] = WholePlaneFilter::create;
}

void GraphValidator::register_factory(const char *filter_name, filter_factory func)
{
	m_factories[filter_name] = func;
}

void GraphValidator::validate(const ScriptStatement *statements, size_t num_statements)
{
	auto do_validate = [&](auto hooks)
	{
		std::vector<std::unique_ptr<ValidationFilter>> filters;
		graphengine::GraphImpl graph;
		hooks(&graph);

		Script script{ &m_factories, &filters, &graph };
		for (size_t i = 0; i < num_statements; ++i) {
			script.eval(statements[i]);
		}

		std::shared_ptr<void> tmp{ _aligned_malloc(graph.get_tmp_size(), 64), _aligned_free };

		ValidationFilter::ValidationState validation_state{ script.filter_table(), script.endpoints(), script.filter_parents() };
		for (const auto &f : filters) {
			f->state = &validation_state;
		}
		ASSERT_NO_THROW(graph.run(validation_state.endpoints(), tmp.get()));
	};

	{
		SCOPED_TRACE("nonplanar+untiled");
		do_validate([](graphengine::Graph *graph)
		{
			graphengine::GraphImpl::from(graph)->set_planar_enabled(false);
			graphengine::GraphImpl::from(graph)->set_tiling_enabled(false);
		});
	}

	{
		SCOPED_TRACE("nonplanar+tiled");
		do_validate([](graphengine::Graph *graph)
		{
			graphengine::GraphImpl::from(graph)->set_planar_enabled(false);
			graphengine::GraphImpl::from(graph)->set_tiling_enabled(true);
			graphengine::GraphImpl::from(graph)->set_tile_width(128);
		});
	}

	{
		SCOPED_TRACE("planar+untiled");
		do_validate([](graphengine::Graph *graph)
		{
			graphengine::GraphImpl::from(graph)->set_planar_enabled(true);
			graphengine::GraphImpl::from(graph)->set_tiling_enabled(false);
		});
	}

	{
		SCOPED_TRACE("planar+tiled");
		do_validate([](graphengine::Graph *graph)
		{
			graphengine::GraphImpl::from(graph)->set_planar_enabled(true);
			graphengine::GraphImpl::from(graph)->set_tiling_enabled(true);
			graphengine::GraphImpl::from(graph)->set_tile_width(128);
		});
	}
}
