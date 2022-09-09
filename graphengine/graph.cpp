#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <climits>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>
#include "graphengine/filter.h"
#include "graphengine/graph.h"
#include "cpuinfo.h"
#include "node.h"
#include "state.h"

#define TRY try
#define CATCH \
  catch (const std::bad_alloc &) { throw Exception{ Exception::OUT_OF_MEMORY, "out of memory" }; } \
  catch (const std::exception &) { throw Exception{ Exception::UNKNOWN, "unknown C++ exception" }; } \
  catch (const graphengine::Exception &) { throw; } \
  catch (...) { throw Exception{ Exception::UNKNOWN, "unknown exception" }; }

namespace graphengine {
namespace GRAPHENGINE_IMPL_NAMESPACE {

namespace {

constexpr unsigned ALIGNMENT = 64;
constexpr unsigned ALIGNMENT_MASK = ALIGNMENT - 1;

constexpr unsigned TILE_WIDTH_MIN = 128;

unsigned ceil_log2(unsigned count) noexcept
{
	unsigned long lzcnt;

	if (count <= 1)
		return 0;

#if defined(_MSC_VER)
	unsigned long msb;
	_BitScanReverse(&msb, count - 1);
	lzcnt = std::numeric_limits<unsigned>::digits - 1 - msb;
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

	return std::numeric_limits<unsigned>::digits - lzcnt;
}

void validate_plane_desc(const PlaneDescriptor &desc)
{
	if (!desc.width || !desc.height)
		throw Exception{ Exception::INVALID_DESCRIPTOR, "must have non-zero plane dimensions" };
	if (desc.bytes_per_sample != 1 && desc.bytes_per_sample != 2 && desc.bytes_per_sample != 4)
		throw Exception{ Exception::INVALID_DESCRIPTOR, "bytes_per_sample must be 1, 2, or 4" };

	constexpr unsigned max_width = static_cast<unsigned>(UINT_MAX) & ~(ALIGNMENT_MASK);
	if (max_width < desc.width)
		throw Exception{ Exception::INVALID_DIMENSIONS, "frame dimensions too large" };

	constexpr size_t max_plane_size = static_cast<size_t>(PTRDIFF_MAX) & ~static_cast<size_t>(ALIGNMENT_MASK);
	if (max_plane_size / desc.bytes_per_sample < desc.width)
		throw Exception{ Exception::INVALID_DIMENSIONS, "frame dimensions too large" };

	size_t rowsize = ((static_cast<size_t>(desc.bytes_per_sample) * desc.width) + ALIGNMENT_MASK) & ~static_cast<size_t>(ALIGNMENT_MASK);
	if (max_plane_size / rowsize < desc.height)
		throw Exception{ Exception::INVALID_DIMENSIONS, "frame dimensions too large" };
}

unsigned auto_tile_width(size_t cache_size_hint, unsigned width, size_t cache_footprint)
{
	size_t cache_size = cache_size_hint ? cache_size_hint : cpu_cache_per_thread();
	unsigned tile = static_cast<unsigned>(std::lrint(width * std::min(static_cast<double>(cache_size) / cache_footprint, 1.0)));

	// Try 1, 2, and 3 tiles.
	if (tile > (width / 5) * 4)
		return width;
	if (tile > width / 2)
		return ((width / 2) + ALIGNMENT_MASK) & ~(ALIGNMENT_MASK);
	if (tile > width / 3)
		return ((width / 3) + ALIGNMENT_MASK) & ~(ALIGNMENT_MASK);

	// Classify graph as uncacheable if minimum tile exceeds cache by 10%.
	tile = std::max(tile & ~(ALIGNMENT_MASK), TILE_WIDTH_MIN);
	if (tile == TILE_WIDTH_MIN && static_cast<double>(tile) / width * cache_footprint > cache_size * 1.1)
		return width;

	return tile;
}


class CopyFilter final : public Filter {
	FilterDescriptor m_desc;
public:
	explicit CopyFilter(const PlaneDescriptor &desc) : m_desc{}
	{
		m_desc.format = desc;
		m_desc.num_deps = 1;
		m_desc.num_planes = 1;
		m_desc.step = 1;
		m_desc.flags.in_place = true;
	}

	int version() const noexcept override { return VERSION; }

	const FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned i) const noexcept override { return{ i, i + 1 }; }

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override { return{ left, right }; }

	void init_context(void *) const noexcept override {}

	void process(const BufferDescriptor *in, const BufferDescriptor *out, unsigned i, unsigned left, unsigned right, void *, void *) const noexcept override
	{
		const unsigned char *src_ptr = in->get_line<unsigned char>(i);
		unsigned char *dst_ptr = out->get_line<unsigned char>(i);
		src_ptr += static_cast<size_t>(left) * m_desc.format.bytes_per_sample;
		dst_ptr += static_cast<size_t>(left) * m_desc.format.bytes_per_sample;
		std::memcpy(dst_ptr, src_ptr, static_cast<size_t>(right - left) * m_desc.format.bytes_per_sample);
	}
};


class PipelineDisableFilter final : public Filter {
	const Filter *m_delegate;
	FilterDescriptor m_desc;
public:
	explicit PipelineDisableFilter(const Filter *filter) : m_delegate{ filter }, m_desc(filter->descriptor())
	{
		m_desc.step = m_desc.format.height;
		m_desc.flags.entire_col = 1;
	}

	int version() const noexcept override { return VERSION; }

	const FilterDescriptor &descriptor() const noexcept override { return m_desc; }

	pair_unsigned get_row_deps(unsigned) const noexcept override
	{
		unsigned last_call = m_desc.format.height - 1;
		last_call = last_call - last_call % m_delegate->descriptor().step;

		auto first_range = m_delegate->get_row_deps(0);
		auto second_range = m_delegate->get_row_deps(last_call);
		return{ first_range.first, second_range.second };
	}

	pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept override { return m_delegate->get_col_deps(left, right); }

	void init_context(void *context) const noexcept override { m_delegate->init_context(context); }

	void process(const BufferDescriptor in[], const BufferDescriptor out[], unsigned, unsigned left, unsigned right, void *context, void *tmp) const noexcept override
	{
		unsigned step = m_delegate->descriptor().step;

		for (unsigned i = 0; i < m_desc.format.height; i += step) {
			m_delegate->process(in, out, i, left, right, context, tmp);
		}
	}
};

} // namespace


class GraphImpl::impl {
	struct SimulationResult {
		struct node_result {
			size_t cache_size_bytes[NODE_MAX_PLANES];
			ptrdiff_t cache_stride[NODE_MAX_PLANES];
			size_t context_size;
			unsigned cache_mask[NODE_MAX_PLANES];
			unsigned initial_cursor;
		};

		std::vector<node_result> nodes;
		size_t cache_footprint = {};
		size_t tmp_size = {};
		size_t scratchpad_size = {};
		unsigned step = {};
		bool no_tiling = {};

		explicit SimulationResult(size_t num_nodes) : nodes(num_nodes) {}
	};

	std::vector<std::unique_ptr<Filter>> m_pipeline_wrappers;
	std::unique_ptr<Filter> m_copy_filters[NODE_MAX_PLANES];
	std::vector<std::unique_ptr<Node>> m_nodes;
	std::vector<node_id> m_source_ids;
	std::unique_ptr<SimulationResult> m_simulation_result;
	std::unique_ptr<SimulationResult> m_planar_simulation_result[NODE_MAX_PLANES];
	node_dep_desc m_planar_deps[NODE_MAX_PLANES];
	node_id m_sink_id = null_node;
	size_t m_cache_size = 0;
	unsigned m_tile_width = 0;

	struct {
		unsigned pipelining_disabled : 1;
		unsigned buffer_sizing_disabled : 1;
		unsigned fusion_disabled : 1;
		unsigned planar_disabled : 1;
		unsigned tiling_disabled : 1;
	} m_flags = {};

	node_id next_node_id() const { return static_cast<node_id>(m_nodes.size()); }

	Node *node(node_id id) const { return m_nodes[id].get(); }

	Node *lookup_node(node_id id) const
	{
		if (id < 0)
			throw Exception{ Exception::INVALID_NODE, "null node" };
		if (static_cast<size_t>(id) >= m_nodes.size())
			throw Exception{ Exception::INVALID_NODE, "id out of range" };
		return node(id);
	}

	std::array<node_dep, NODE_MAX_PLANES> resolve_node_deps(unsigned num_deps, const node_dep_desc deps[]) const
	{
		assert(num_deps <= NODE_MAX_PLANES);

		std::array<node_dep, NODE_MAX_PLANES> resolved_deps;

		for (unsigned i = 0; i < num_deps; ++i) {
			Node *node = lookup_node(deps[i].id);
			if (deps[i].plane >= node->num_planes())
				throw Exception{ Exception::INVALID_NODE, "plane number out of range" };

			resolved_deps[i] = { node, deps[i].plane };
		}

		return resolved_deps;
	}

	void reserve_next_node()
	{
		if (m_nodes.size() > node_id_max)
			throw Exception{ Exception::LIMIT_EXCEEDED, "maximum number of nodes exceeded" };

		m_nodes.reserve(m_nodes.size() + 1);
	}

	void add_node(std::unique_ptr<Node> node)
	{
		if (m_nodes.size() > node_id_max)
			throw Exception{ Exception::LIMIT_EXCEEDED, "maximum number of nodes exceeded" };

		assert(static_cast<size_t>(node->id()) == m_nodes.size());
		m_nodes.push_back(std::move(node));
	}

	node_id add_transform_internal(const Filter *filter, const node_dep_desc deps[])
	{
		const FilterDescriptor &desc = filter->descriptor();
		if (desc.num_deps > FILTER_MAX_DEPS)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "maximum number of filter dependencies exceeded" };
		if (!desc.num_planes)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "filter must have non-zero plane count" };
		if (desc.num_planes > FILTER_MAX_PLANES)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "maximum number of filter outputs exceeded" };

		validate_plane_desc(desc.format);

		auto resolved_deps = resolve_node_deps(desc.num_deps, deps);

		for (unsigned p = 1; p < desc.num_deps; ++p) {
			const PlaneDescriptor luma_desc = resolved_deps[0].first->format(resolved_deps[0].second);
			const PlaneDescriptor desc = resolved_deps[p].first->format(resolved_deps[p].second);

			if (luma_desc.width != desc.width || luma_desc.height != desc.height)
				throw Exception{ Exception::INVALID_DIMENSIONS, "must have identical dimensions across all dependencies" };
		}

		std::unique_ptr<Node> node = make_transform_node(next_node_id(), filter, resolved_deps.data());
		node_id id = node->id();
		add_node(std::move(node));

		for (unsigned p = 0; p < desc.num_deps; ++p) {
			resolved_deps[p].first->add_ref(resolved_deps[p].second);
		}

		return id;
	}

	std::unique_ptr<Simulation> begin_compile(unsigned num_planes)
	{
		m_simulation_result = std::make_unique<SimulationResult>(m_nodes.size());
		return std::make_unique<Simulation>(m_nodes.size());
	}

	void compile_plane(Simulation *sim, const node_dep &dep) noexcept
	{
		Node *node = dep.first;
		unsigned plane = dep.second;
		assert(!node->sourcesink());

		node->trace_working_memory(sim);

		unsigned height = node->format(plane).height;
		for (unsigned i = 0; i < height; ++i) {
			node->trace_access_pattern(sim, i, i + 1, plane);
		}
	}

	void compile_simulation_result(SimulationResult &result, const Simulation &sim, bool skip_endpoints)
	{
		assert(result.nodes.size() == m_nodes.size());

		result.step = sim.step();
		result.no_tiling = sim.no_tiling();

		for (const auto &node : m_nodes) {
			node_id id = node->id();
			SimulationResult::node_result &node_result = result.nodes[id];

			if (!sim.is_live_node(id))
				continue;
			if (skip_endpoints && node->sourcesink())
				continue;

			for (unsigned p = 0; p < node->num_planes(); ++p) {
				PlaneDescriptor desc = node->format(p);
				unsigned subsample_h = node->subsample_h(p);
				unsigned live_rows = sim.live_range(id);
				if (id != m_sink_id)
					assert(live_rows % (1U << subsample_h) == 0);

				unsigned mask = BUFFER_MAX;
				if (!m_flags.buffer_sizing_disabled) {
					unsigned lines = (live_rows + ((1U << subsample_h) - 1)) >> subsample_h;
					unsigned lines_ceil_log2 = ceil_log2(lines);
					if (lines_ceil_log2 < std::numeric_limits<unsigned>::digits)
						mask = (1U << lines_ceil_log2) - 1;
				}

				if (mask >= desc.height - 1)
					mask = BUFFER_MAX;

				// External nodes are allocated by the caller.
				if (!node->sourcesink()) {
					unsigned buffer_lines = mask == BUFFER_MAX ? desc.height : mask + 1;
					size_t rowsize = (static_cast<size_t>(desc.width) * desc.bytes_per_sample + ALIGNMENT_MASK) & ~static_cast<size_t>(ALIGNMENT_MASK);
					node_result.cache_size_bytes[p] = static_cast<size_t>(buffer_lines) * rowsize;
					node_result.cache_stride[p] = rowsize;
					result.tmp_size += node_result.cache_size_bytes[p];
				}
				node_result.cache_mask[p] = mask;
			}

			node_result.context_size = (sim.context_size(id) + ALIGNMENT_MASK) & ~static_cast<size_t>(ALIGNMENT_MASK);
			node_result.initial_cursor = sim.cursor_min(id);
			result.tmp_size += node_result.context_size;
		}
		result.tmp_size += sim.scratchpad_size();
		result.scratchpad_size = sim.scratchpad_size();

		// Cache footprint also includes the endpoints.
		const auto node_footprint = [&](node_id id)
		{
			const auto &node = m_nodes[id];
			size_t footprint = 0;

			for (unsigned p = 0; p < node->num_planes(); ++p) {
				PlaneDescriptor desc = node->format(p);
				unsigned mask = result.nodes[id].cache_mask[p];
				size_t rowsize = static_cast<size_t>(desc.width) * desc.bytes_per_sample;
				footprint += rowsize * (mask == BUFFER_MAX ? desc.height : mask + 1);
			}
			return footprint;
		};

		result.cache_footprint = result.tmp_size;
		for (node_id id : m_source_ids) {
			result.cache_footprint += node_footprint(id);
		}
		result.cache_footprint += node_footprint(m_sink_id);
	}

	void compile(Simulation *sim, unsigned num_planes, node_dep deps[]) noexcept
	{
		assert(m_simulation_result);

		Node *sink = node(m_sink_id);

		if (m_flags.fusion_disabled) {
			// Even with fusion disabled, the output nodes need to be redirected to the sink.
			for (unsigned p = 0; p < num_planes; ++p) {
				deps[p].first->set_cache_location(deps[p].second, FrameState::cache_descriptor_offset(m_sink_id, p));
				for (unsigned q = 0; q < p; ++q) {
					assert(deps[p].first->cache_location(deps[p].second) != deps[q].first->cache_location(deps[q].second));
				}
			}
		} else {
			sink->apply_node_fusion();
		}

		// Lookup the per-filter memory requirements.
		sink->trace_working_memory(sim);

		// Calculate the subsampling factor.
		unsigned sink_planes = sink->num_planes();
		unsigned height = 0;
		for (unsigned p = 0; p < sink_planes; ++p) {
			height = std::max(height, sink->format(p).height);
		}
		for (unsigned p = 0; p < sink_planes; ++p) {
			sim->update_step(height / sink->format(p).height);
		}

		// Trace the scanline dependency pattern.
		for (unsigned i = 0; i < height; i += sim->step()) {
			unsigned next = height - i < sim->step() ? height : i + sim->step();
			sink->trace_access_pattern(sim, i, next, 0);
		}

		compile_simulation_result(*m_simulation_result, *sim, false);

		// Determine if the graph can be traversed in planar order.
		bool endpoint_visited[(GRAPH_MAX_ENDPOINTS - 1) * NODE_MAX_PLANES] = {};
		bool planar_compatible = !m_flags.planar_disabled;

		for (unsigned p = 0; p < num_planes; ++p) {
			Node *node = deps[p].first;

			for (size_t source_idx = 0; source_idx < m_source_ids.size(); ++source_idx) {
				for (unsigned source_plane = 0; source_plane < NODE_MAX_PLANES; ++source_plane) {
					bool reachable = node->reachable(m_source_ids[source_idx], source_plane);
					bool &flag = endpoint_visited[source_idx * NODE_MAX_PLANES + source_plane];

					if (reachable && flag) {
						planar_compatible = false;
						goto out;
					}
					if (reachable)
						flag = true;
				}
			}
		}
	out:
		if (!planar_compatible)
			return;

		for (unsigned p = 0; p < num_planes; ++p) {
			sim->reset();
			compile_plane(sim, deps[p]);
			m_planar_simulation_result[p] = std::make_unique<SimulationResult>(m_nodes.size());
			compile_simulation_result(*m_planar_simulation_result[p], *sim, true);
			m_planar_deps[p] = { deps[p].first->id(), deps[p].second };
		}
	}

	bool can_run_planar() const { return !m_flags.planar_disabled && m_planar_simulation_result[0] != nullptr; }

	void prepare_frame_state(FrameState *state, const SimulationResult &sim, const Endpoint endpoints[], void *tmp) const
	{
		unsigned char *head = static_cast<unsigned char *>(tmp);
		auto allocate = [&](auto *&ptr, size_t count) { ptr = reinterpret_cast<decltype(ptr)>(head); head += sizeof(*ptr) * count; };

		new (state) FrameState{ head, m_nodes.size() };

		// Realign pointer.
		size_t offset = head - static_cast<unsigned char *>(tmp);
		head = static_cast<unsigned char *>(tmp) + ((offset + ALIGNMENT_MASK) & ~static_cast<size_t>(ALIGNMENT_MASK));

#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
		size_t guard_page_idx = 0;
		auto allocate_guard_page = [&]()
		{
			unsigned char *page = nullptr;
			allocate(page, FrameState::guard_page_size());
			state->set_guard_page(guard_page_idx++, page);
		};
#else
		auto allocate_guard_page = []() {};
#endif

		// Allocate caches.
		for (size_t i = 0; i < m_nodes.size(); ++i) {
			const SimulationResult::node_result &node_sim = sim.nodes[i];
			unsigned num_planes = node(static_cast<node_id>(i))->num_planes();

			allocate_guard_page();
			for (unsigned p = 0; p < num_planes; ++p) {
				BufferDescriptor &cache = state->buffer(FrameState::cache_descriptor_offset(static_cast<node_id>(i), p));

				unsigned char *cache_data = nullptr;
				allocate(cache_data, node_sim.cache_size_bytes[p]);

				cache.ptr = cache_data;
				cache.stride = node_sim.cache_stride[p];
				cache.mask = node_sim.cache_mask[p];
			}
		}

		// Allocate filter contexts.
		for (size_t i = 0; i < m_nodes.size(); ++i) {
			unsigned char *context_data = nullptr;
			allocate_guard_page();
			allocate(context_data, sim.nodes[i].context_size);

			state->set_context(static_cast<node_id>(i), context_data);
		}

		// Allocate scratchpad
		allocate_guard_page();
		state->set_scratchpad(head);
		head += sim.scratchpad_size;
		allocate_guard_page();

#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
		assert(guard_page_idx == FrameState::num_guard_pages(m_nodes.size()));
		state->set_guard_page(guard_page_idx++, nullptr);
#endif

		// Setup endpoints.
		for (size_t i = 0; i < m_source_ids.size() + 1; ++i) {
			assert(endpoints[i].id != null_node);

			std::copy_n(endpoints[i].buffer, node(endpoints[i].id)->num_planes(), &state->buffer(FrameState::cache_descriptor_offset(endpoints[i].id, 0)));
			state->set_callback(i, endpoints[i].id, endpoints[i].callback);
		}
	}

	unsigned calculate_tile_width(const SimulationResult &sim, unsigned width) const
	{
		unsigned tile_width;

		if (sim.no_tiling || m_flags.buffer_sizing_disabled || m_flags.tiling_disabled)
			tile_width = width;
		else if (m_tile_width)
			tile_width = m_tile_width;
		else
			tile_width = auto_tile_width(m_cache_size, width, sim.cache_footprint);

		assert(tile_width == width || tile_width % ALIGNMENT == 0);
		return tile_width;
	}

	void run_node(Node *node, const SimulationResult &sim, const Endpoint endpoints[], unsigned plane, void *tmp) const
	{
		std::aligned_union_t<0, FrameState> _;
		static_assert(std::is_trivially_destructible<FrameState>::value, "destructor not allowed");

		FrameState *state = reinterpret_cast<FrameState *>(&_);
		prepare_frame_state(state, sim, endpoints, tmp);

		const PlaneDescriptor &format = node->format(plane);
		unsigned tile_width = calculate_tile_width(sim, format.width);

		for (unsigned j = 0; j < format.width;) {
			unsigned j_end = std::min(j + tile_width, format.width);
			if (format.width - j_end < TILE_WIDTH_MIN)
				j_end = format.width;

			// Initialize cursors.
			for (size_t i = 0; i < m_nodes.size(); ++i) {
				state->set_cursor(static_cast<node_id>(i), sim.nodes[i].initial_cursor);
			}
			state->reset_initialized(m_nodes.size());

			node->begin_frame(state, j, j_end, plane);
			node->process(state, format.height, plane);

			j = j_end;
		}
	}
public:
	// Optimization toggles.
	void set_pipelining_enabled(bool enabled) { m_flags.pipelining_disabled = !enabled; }
	void set_buffer_sizing_enabled(bool enabled) { m_flags.buffer_sizing_disabled = !enabled; }
	void set_fusion_enabled(bool enabled) { m_flags.fusion_disabled = !enabled; }
	void set_planar_enabled(bool enabled) { m_flags.planar_disabled = !enabled; }
	void set_tiling_enabled(bool enabled) { m_flags.tiling_disabled = !enabled; }

	void set_cache_size(size_t cache_size) { m_cache_size = cache_size; }
	void set_tile_width(unsigned tile_width) { m_tile_width = (tile_width + ALIGNMENT_MASK) & ~ALIGNMENT_MASK; }

	unsigned get_tile_width(bool with_callbacks) const
	{
		if (m_sink_id < 0)
			throw Exception{ Exception::ILLEGAL_STATE, "sink not set" };

		Node *sink = node(m_sink_id);

		if (with_callbacks || !can_run_planar())
			return calculate_tile_width(*m_simulation_result, sink->format(0).width);

		unsigned num_planes = node(m_sink_id)->num_planes();
		unsigned tile_width = 0;

		for (unsigned p = 0; p < num_planes; ++p) {
			unsigned tmp = calculate_tile_width(*m_planar_simulation_result[p], node(m_planar_deps[p].id)->format(m_planar_deps[p].plane).width);
			tile_width = std::max(tile_width, tmp);
		}
		return tile_width;
	}

	// Graph construction methods. Strong exception safety. Graphs have up to 7 sources and 1 sink.
	// Graphs are final once a sink has been defined. No additional nodes may be inserted.
	node_id add_source(unsigned num_planes, const PlaneDescriptor desc[])
	{
		if (!num_planes)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "endpoint must have non-zero plane count" };
		if (num_planes > NODE_MAX_PLANES)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "maximum number of endpoint planes exceeded" };
		if (m_source_ids.size() >= GRAPH_MAX_ENDPOINTS - 1)
			throw Exception{ Exception::LIMIT_EXCEEDED, "maximum number of sources exceeded" };

		for (unsigned p = 0; p < num_planes; ++p) {
			validate_plane_desc(desc[p]);
		}

		m_source_ids.reserve(GRAPH_MAX_ENDPOINTS - 1);

		std::unique_ptr<Node> node = make_source_node(next_node_id(), num_planes, desc);
		node_id id = node->id();
		add_node(std::move(node));
		m_source_ids.push_back(id);
		return id;
	}

	node_id add_transform(const Filter *filter, const node_dep_desc deps[])
	{
		if (filter->version() != Filter::VERSION)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "incompatible filter version" };

		if (m_flags.pipelining_disabled) {
			m_pipeline_wrappers.push_back(std::make_unique<PipelineDisableFilter>(filter));
			filter = m_pipeline_wrappers.back().get();
		}

		try {
			return add_transform_internal(filter, deps);
		} catch (...) {
			if (m_flags.pipelining_disabled)
				m_pipeline_wrappers.pop_back();
			throw;
		}
	}

	node_id add_sink(unsigned num_planes, const node_dep_desc deps[])
	{
		if (!num_planes)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "endpoint must have non-zero plane count" };
		if (num_planes > NODE_MAX_PLANES)
			throw Exception{ Exception::INVALID_DESCRIPTOR, "maximum number of endpoint planes exceeded" };
		if (m_sink_id >= 0)
			throw Exception{ Exception::ILLEGAL_STATE, "sink already set" };

		auto original_resolved_deps = resolve_node_deps(num_planes, deps);
		size_t original_node_count = m_nodes.size();

		auto resolved_deps = original_resolved_deps;
		std::unique_ptr<Simulation> sim;

		try {
			for (unsigned p = 0; p < num_planes; ++p) {
				bool duplicate = std::find(resolved_deps.begin(), resolved_deps.begin() + p, resolved_deps[p]) != resolved_deps.begin() + p;

				// Sink node does not copy anything. Insert copy filters where needed.
				if (!resolved_deps[p].first->sourcesink() && !duplicate)
					continue;

				node_dep_desc copy_deps[FILTER_MAX_DEPS] = { deps[p] };
				auto filter = std::make_unique<CopyFilter>(resolved_deps[p].first->format(resolved_deps[p].second));

				node_id copy_id = add_transform_internal(filter.get(), copy_deps);
				m_copy_filters[p] = std::move(filter);
				resolved_deps[p].first = node(copy_id);
				resolved_deps[p].second = 0;
			}

			for (unsigned p = 0; p < num_planes; ++p) {
				resolved_deps[p].first->add_ref(resolved_deps[p].second);
			}

			std::unique_ptr<Node> node = make_sink_node(next_node_id(), num_planes, resolved_deps.data());
			node_id id = node->id();
			add_node(std::move(node));
			m_sink_id = id;

			// Compilation is irreversible. Run any steps that could fail upfront.
			sim = begin_compile(num_planes);
		} catch (...) {
			// Delete invalid simulation results.
			for (unsigned p = 0; p < num_planes; ++p) {
				m_planar_simulation_result[p].reset();
			}
			m_simulation_result.reset();

			// Unreference the output nodes.
			for (unsigned p = 0; p < num_planes; ++p) {
				resolved_deps[p].first->dec_ref(resolved_deps[p].second);
				if (resolved_deps[p] != original_resolved_deps[p])
					original_resolved_deps[p].first->dec_ref(original_resolved_deps[p].second);
			}

			// Remove inserted copy and sink nodes.
			m_nodes.resize(original_node_count);
			m_sink_id = null_node;

			// Destroy all copy filters.
			for (unsigned p = 0; p < NODE_MAX_PLANES; ++p) {
				m_copy_filters[p].reset();
			}
			throw;
		}

		compile(sim.get(), num_planes, resolved_deps.data());
		return m_sink_id;
	}

	// Runtime execution methods. The sink node must be defined.
	size_t get_cache_footprint(bool with_callbacks) const
	{
		if (m_sink_id < 0)
			throw Exception{ Exception::ILLEGAL_STATE, "sink not set" };

		size_t interleaved_footprint = FrameState::metadata_size(m_nodes.size()) + m_simulation_result->cache_footprint;
		if (with_callbacks || !can_run_planar())
			return interleaved_footprint;

		size_t size = 0;
		for (const auto &result : m_planar_simulation_result) {
			if (!result)
				break;
			size = std::max(size, result->cache_footprint);
		}
		return FrameState::metadata_size(m_nodes.size()) + size;
	}

	size_t get_tmp_size(bool with_callbacks) const
	{
		if (m_sink_id < 0)
			throw Exception{ Exception::ILLEGAL_STATE, "sink not set" };

#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
		size_t guard_page_size = FrameState::guard_page_size() * FrameState::num_guard_pages(m_nodes.size());
#else
		size_t guard_page_size = 0;
#endif

		size_t interleaved_size = FrameState::metadata_size(m_nodes.size()) + m_simulation_result->tmp_size + guard_page_size;
		if (with_callbacks || !can_run_planar())
			return interleaved_size;

		size_t size = 0;
		for (const auto &result : m_planar_simulation_result) {
			if (!result)
				break;
			size = std::max(size, result->tmp_size);
		}
		return FrameState::metadata_size(m_nodes.size()) + size + guard_page_size;
	}

	BufferingRequirement get_buffering_requirement() const
	{
		if (m_sink_id < 0)
			throw Exception{ Exception::ILLEGAL_STATE, "sink not set" };

		assert(m_source_ids.size() < GRAPH_MAX_ENDPOINTS);

		BufferingRequirement buffering{};

		auto max_buffering = [=](node_id id)
		{
			Node *node = this->node(id);
			unsigned planes = node->num_planes();
			unsigned buffering = 0;

			for (unsigned p = 0; p < planes; ++p) {
				buffering = std::max(buffering, m_simulation_result->nodes[id].cache_mask[p]);
			}
			return buffering;
		};

		for (size_t i = 0; i < m_source_ids.size(); ++i) {
			buffering[i] = { m_source_ids[i], max_buffering(m_source_ids[i]) };
		}
		buffering[m_source_ids.size()] = { m_sink_id, max_buffering(m_sink_id) };

		return buffering;
	}

	void run(const Endpoint endpoints[], void *tmp) const
	{
		if (m_sink_id < 0)
			throw Exception{ Exception::ILLEGAL_STATE, "sink not set" };

		bool planar = can_run_planar();
		if (planar) {
			const Endpoint *endpoints_begin = endpoints;
			const Endpoint *endpoints_end = endpoints + m_source_ids.size() + 1;
			planar = std::find_if(endpoints_begin, endpoints_end, [](const Endpoint &e) { return !!e.callback; }) == endpoints_end;
		}

		Node *sink = node(m_sink_id);

		if (planar) {
			unsigned num_planes = sink->num_planes();

			for (unsigned p = 0; p < num_planes; ++p) {
				run_node(node(m_planar_deps[p].id), *m_planar_simulation_result[p], endpoints, m_planar_deps[p].plane, tmp);
			}
		} else {
			run_node(sink, *m_simulation_result, endpoints, 0, tmp);
		}
	}
};


GraphImpl *GraphImpl::from(Graph *graph) noexcept { return dynamic_cast<GraphImpl *>(graph); }

GraphImpl::GraphImpl() TRY : m_impl{ new impl{} } {} CATCH
GraphImpl::GraphImpl(GraphImpl &&other) noexcept = default;
GraphImpl::~GraphImpl() = default;
GraphImpl &GraphImpl::operator=(GraphImpl &&other) noexcept = default;

void GraphImpl::set_pipelining_enabled(bool enabled) { m_impl->set_pipelining_enabled(enabled); }
void GraphImpl::set_buffer_sizing_enabled(bool enabled) { m_impl->set_buffer_sizing_enabled(enabled); }
void GraphImpl::set_fusion_enabled(bool enabled) { m_impl->set_fusion_enabled(enabled); }
void GraphImpl::set_planar_enabled(bool enabled) { m_impl->set_planar_enabled(enabled); }
void GraphImpl::set_tiling_enabled(bool enabled) { m_impl->set_tiling_enabled(enabled); }

void GraphImpl::set_cache_size(size_t cache_size) { m_impl->set_cache_size(cache_size); }
void GraphImpl::set_tile_width(unsigned tile_width) { m_impl->set_tile_width(tile_width); }

unsigned GraphImpl::get_tile_width(bool with_callbacks) const TRY
{
	return m_impl->get_tile_width(with_callbacks);
} CATCH

node_id GraphImpl::add_source(unsigned num_planes, const PlaneDescriptor desc[]) TRY
{
	return m_impl->add_source(num_planes, desc);
} CATCH

node_id GraphImpl::add_transform(const Filter *filter, const node_dep_desc deps[]) TRY
{
	return m_impl->add_transform(filter, deps);
} CATCH

node_id GraphImpl::add_sink(unsigned num_planes, const node_dep_desc deps[]) TRY
{
	return m_impl->add_sink(num_planes, deps);
} CATCH

size_t GraphImpl::get_cache_footprint(bool with_callbacks) const TRY
{
	return m_impl->get_cache_footprint(with_callbacks);
} CATCH

size_t GraphImpl::get_tmp_size(bool with_callbacks) const TRY
{
	return m_impl->get_tmp_size(with_callbacks);
} CATCH

Graph::BufferingRequirement GraphImpl::get_buffering_requirement() const TRY
{
	return m_impl->get_buffering_requirement();
} CATCH

void GraphImpl::run(const Endpoint *endpoints, void *tmp) const TRY
{
	return m_impl->run(endpoints, tmp);
} CATCH


class SubGraphImpl::impl {
	struct NodeEntry {
		const Filter *filter;
		node_dep_desc deps[FILTER_MAX_DEPS];

		explicit NodeEntry(std::nullptr_t) : filter{}, deps{}
		{
			std::fill_n(deps, FILTER_MAX_DEPS, null_dep);
		}

		NodeEntry(const Filter *filter, const node_dep_desc deps[]) : NodeEntry{ nullptr }
		{
			this->filter = filter;
			std::copy_n(deps, filter->descriptor().num_deps, this->deps);
		}
	};

	std::vector<NodeEntry> m_nodes;
	std::vector<node_id> m_source_ids;
	std::vector<std::pair<node_id, node_dep_desc>> m_sinks;

	void check_deps(unsigned num_deps, const node_dep_desc deps[]) const
	{
		for (unsigned p = 0; p < num_deps; ++p) {
			if (deps[p].id >= static_cast<node_id>(m_nodes.size()))
				throw Exception{ Exception::INVALID_NODE, "id out of range" };
		}
	}

	bool is_source(node_id id) const
	{
		return std::find(m_source_ids.begin(), m_source_ids.end(), id) != m_source_ids.end();
	}

	bool is_sink(node_id id) const
	{
		return std::find_if(m_sinks.begin(), m_sinks.end(), [=](const std::pair<node_id, node_dep_desc> &m) { return m.first == id; }) != m_sinks.end();
	}
public:
	node_id add_source()
	{
		if (m_nodes.size() > node_id_max)
			throw Exception{ Exception::LIMIT_EXCEEDED, "maximum number of nodes exceeded" };

		m_source_ids.reserve(m_source_ids.size() + 1);
		m_nodes.reserve(m_nodes.size() + 1);

		m_nodes.push_back(NodeEntry{ nullptr });
		m_source_ids.push_back(static_cast<node_id>(m_nodes.size() - 1));
		return m_source_ids.back();
	}

	node_id add_transform(const Filter *filter, const node_dep_desc deps[])
	{
		if (m_nodes.size() > node_id_max)
			throw Exception{ Exception::LIMIT_EXCEEDED, "maximum number of nodes exceeded" };

		check_deps(filter->descriptor().num_deps, deps);
		m_nodes.push_back({ filter, deps });
		return static_cast<node_id>(m_nodes.size() - 1);
	}

	node_id add_sink(const node_dep_desc &dep)
	{
		if (m_nodes.size() > node_id_max)
			throw Exception{ Exception::LIMIT_EXCEEDED, "maximum number of nodes exceeded" };

		check_deps(1, &dep);

		m_sinks.reserve(m_sinks.size() + 1);
		m_nodes.reserve(m_nodes.size() + 1);

		m_nodes.push_back(NodeEntry{ nullptr });
		m_sinks.push_back({ static_cast<node_id>(m_nodes.size() - 1), dep });
		return m_sinks.back().first;
	}

	void connect(Graph *graph, size_t num_sources, const Mapping sources[], Mapping sinks[]) const
	{
		std::vector<node_id> node_mapping(m_nodes.size(), null_node);

		auto resolve = [&](node_dep_desc dep) -> node_dep_desc
		{
			if (is_source(dep.id)) {
				auto it = std::find_if(sources, sources + num_sources, [&](const Mapping &s) { return s.internal_id == dep.id; });
				if (it == sources + num_sources)
					throw Exception{ Exception::INVALID_NODE, "endpoint not defined" };
				return it->external_dep;
			} else if (is_sink(dep.id)) {
				throw Exception{ Exception::INVALID_NODE, "invalid dependency on sink" };
			} else {
				assert(dep.id >= 0);
				assert(static_cast<size_t>(dep.id) < node_mapping.size());
				assert(node_mapping[dep.id] >= 0);
				return{ node_mapping[dep.id], dep.plane };
			}
		};

		for (node_id id = 0; id < static_cast<node_id>(m_nodes.size()); ++id) {
			const NodeEntry &entry = m_nodes[id];
			if (!entry.filter)
				continue;

			node_dep_desc resolved_deps[FILTER_MAX_DEPS];
			unsigned num_deps = entry.filter->descriptor().num_deps;

			for (unsigned p = 0; p < num_deps; ++p) {
				resolved_deps[p] = resolve(entry.deps[p]);
			}

			node_id resolved_id = graph->add_transform(entry.filter, resolved_deps);
			node_mapping[id] = resolved_id;
		}

		for (size_t i = 0; i < m_sinks.size(); ++i) {
			sinks[i].internal_id = m_sinks[i].first;
			sinks[i].external_dep = resolve(m_sinks[i].second);
		}
	}
};


SubGraphImpl::SubGraphImpl() TRY : m_impl{ new impl{} } {} CATCH
SubGraphImpl::SubGraphImpl(SubGraphImpl &&other) noexcept = default;
SubGraphImpl::~SubGraphImpl() = default;
SubGraphImpl &SubGraphImpl::operator=(SubGraphImpl &&other) noexcept = default;

node_id SubGraphImpl::add_source() TRY
{
	return m_impl->add_source();
} CATCH

node_id SubGraphImpl::add_sink(const node_dep_desc &dep) TRY
{
	return m_impl->add_sink(dep);
} CATCH

node_id SubGraphImpl::add_transform(const Filter *filter, const node_dep_desc deps[]) TRY
{
	return m_impl->add_transform(filter, deps);
} CATCH

void SubGraphImpl::connect(Graph *graph, size_t num_sources, const Mapping sources[], Mapping sinks[]) const TRY
{
	return m_impl->connect(graph, num_sources, sources, sinks);
} CATCH

} // namespace impl
} // namespace graphengine
