#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <stdexcept>
#include <vector>
#include "filter.h"
#include "graph.h"
#include "node.h"
#include "state.h"

namespace graphengine {

namespace {

void validate_plane_desc(const PlaneDescriptor &desc)
{
	if (!desc.width || !desc.height)
		throw std::invalid_argument{ "must have non-zero plane dimensions" };
	if (desc.bytes_per_sample != 1 && desc.bytes_per_sample != 2 && desc.bytes_per_sample != 4)
		throw std::invalid_argument{ "bytes_per_sample must be 1, 2, or 4" };

	constexpr size_t max_plane_size = static_cast<size_t>(PTRDIFF_MAX) & ~static_cast<size_t>(63);
	if (max_plane_size / desc.bytes_per_sample < desc.width)
		throw std::range_error{ "frame dimensions too large" };

	size_t rowsize = ((static_cast<size_t>(desc.bytes_per_sample) * desc.width) + 63) & ~static_cast<size_t>(63);
	if (max_plane_size / rowsize < desc.height)
		throw std::range_error{ "frame dimensions too large" };
}

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

} // namespace


struct Graph::SimulationResult {
	struct node_result {
		size_t cache_size_bytes[NODE_MAX_PLANES];
		ptrdiff_t cache_stride[NODE_MAX_PLANES];
		size_t context_size;
		unsigned cache_mask[NODE_MAX_PLANES];
		unsigned initial_cursor;
	};

	std::vector<node_result> nodes;
	size_t tmp_size;
	unsigned step;

	explicit SimulationResult(const Graph &graph, const Simulation &sim) :
		nodes(graph.m_nodes.size()),
		tmp_size{},
		step{ sim.step() }
	{
		for (const auto &node : graph.m_nodes) {
			node_id id = node->id();
			node_result &result = nodes[id];

			unsigned live_rows = sim.live_range(id);
			if (!live_rows)
				continue;

			for (unsigned p = 0; p < node->num_planes(); ++p) {
				const PlaneDescriptor &desc = node->format(p);
				unsigned subsample_h = node->subsample_h(p);
				assert(live_rows % (1U << subsample_h) == 0);

				unsigned lines = live_rows >> subsample_h;
				unsigned lines_ceil_log2 = ceil_log2(lines);
				unsigned mask = lines_ceil_log2 >= std::numeric_limits<unsigned>::digits ? BUFFER_MAX : (1U << lines_ceil_log2) - 1;

				// External nodes are allocated by the caller.
				if (!node->sourcesink()) {
					result.cache_size_bytes[p] = static_cast<size_t>(mask == BUFFER_MAX ? desc.height : mask + 1) * desc.width * desc.bytes_per_sample;
					result.cache_stride[p] = static_cast<size_t>(desc.width) * desc.bytes_per_sample;
					tmp_size += result.cache_size_bytes[p];
				}
				result.cache_mask[p] = mask;
			}

			result.context_size = (sim.context_size(id) + 63) & ~static_cast<size_t>(63);
			result.initial_cursor = sim.cursor_min(id);
			tmp_size += result.context_size;
		}
		tmp_size += sim.scratchpad_size();
	}
};


void Graph::Callback::operator()(unsigned i, unsigned left, unsigned right) const
{
	if (int ret = m_func(m_user, i, left, right))
		throw CallbackError{ ret };
}


Graph::Graph() = default;

Graph::~Graph() = default;

node_id Graph::next_node_id() const { return static_cast<node_id>(m_nodes.size()); }

Node *Graph::lookup_node(node_id id) const
{
	if (id < 0)
		throw std::range_error{ "null node" };
	if (static_cast<size_t>(id) >= m_nodes.size())
		throw std::range_error{ "id out of range" };
	return m_nodes[id].get();
}

std::array<node_dep, NODE_MAX_PLANES> Graph::resolve_node_deps(unsigned num_deps, const node_dep_desc deps[]) const
{
	assert(num_deps <= NODE_MAX_PLANES);

	std::array<node_dep, NODE_MAX_PLANES> resolved_deps;

	for (unsigned i = 0; i < num_deps; ++i) {
		Node *node = lookup_node(deps[i].first);
		if (deps[i].second >= node->num_planes())
			throw std::range_error{ "plane number out of range" };

		resolved_deps[i] = { node, deps[i].second };
	}

	return resolved_deps;
}

void Graph::reserve_next_node()
{
	if (m_nodes.size() > node_id_max)
		throw std::bad_alloc{};

	m_nodes.reserve(m_nodes.size() + 1);
}

void Graph::add_node(std::unique_ptr<Node> node)
{
	if (m_nodes.size() > node_id_max)
		throw std::bad_alloc{};

	assert(node->id() == m_nodes.size());
	m_nodes.push_back(std::move(node));
}

void Graph::compile()
{
	std::unique_ptr<Simulation> sim = std::make_unique<Simulation>(m_nodes.size());

	Node *sink = lookup_node(m_sink_id);
	assert(sink);

	// Lookup the per-filter memory requirements.
	sink->trace_working_memory(sim.get());

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
		sink->trace_access_pattern(sim.get(), i, next, 0);
	}

	m_simulation = std::make_unique<SimulationResult>(*this, *sim);
}

FrameState Graph::prepare_frame_state(const EndpointConfiguration &endpoints, void *tmp) const
{
	unsigned char *head = static_cast<unsigned char *>(tmp);
	auto allocate = [&](auto *&ptr, size_t count) { ptr = reinterpret_cast<decltype(ptr)>(head); head += sizeof(*ptr) * count; };

	FrameState state{ head, m_nodes.size() };

	// Realign pointer.
	ptrdiff_t offset = head - static_cast<unsigned char *>(tmp);
	head += 64 - offset % 64;

	// Initialize cursors.
	for (size_t i = 0; i < m_nodes.size(); ++i) {
		state.set_cursor(static_cast<node_id>(i), m_simulation->nodes[i].initial_cursor);
	}

	// Allocate caches.
	for (size_t i = 0; i < m_nodes.size(); ++i) {
		const SimulationResult::node_result &node_sim = m_simulation->nodes[i];
		unsigned num_planes = lookup_node(static_cast<node_id>(i))->num_planes();

		for (unsigned p = 0; p < num_planes; ++p) {
			BufferDescriptor &cache = state.buffer(static_cast<node_id>(i), p);

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
		allocate(context_data, m_simulation->nodes[i].context_size);

		state.set_context(static_cast<node_id>(i), context_data);
	}

	// Allocate scratchpad.
	state.set_scratchpad(head);

	// Setup endpoints.
	for (size_t i = 0; i < m_source_ids.size() + 1; ++i) {
		assert(endpoints[i].id != null_node);

		std::copy_n(endpoints[i].buffer, lookup_node(endpoints[i].id)->num_planes(), &state.buffer(endpoints[i].id, 0));
		state.set_callback(i, endpoints[i].id, endpoints[i].callback);
	}

	return state;
}

node_id Graph::add_source(unsigned num_planes, const PlaneDescriptor desc[])
{
	if (!num_planes)
		throw std::invalid_argument{ "endpoint must have non-zero plane count" };
	if (num_planes > NODE_MAX_PLANES)
		throw std::invalid_argument{ "maximum number of endpoint planes exceeded" };
	if (m_source_ids.size() >= GRAPH_MAX_ENDPOINTS - 1)
		throw std::invalid_argument{ "maximum number of sources exceeded" };

	for (unsigned p = 0; p < num_planes; ++p) {
		validate_plane_desc(desc[p]);
	}

	reserve_next_node();
	m_source_ids.reserve(m_source_ids.size() + 1);

	std::unique_ptr<Node> node = make_source_node(next_node_id(), num_planes, desc);
	node_id id = node->id();
	add_node(std::move(node));
	m_source_ids.push_back(id);
	return id;
}

node_id Graph::add_transform(const Filter *filter, const node_dep_desc deps[])
{
	const FilterDescriptor &desc = filter->descriptor();
	if (desc.num_deps > FILTER_MAX_DEPS)
		throw std::invalid_argument{ "maximum number of filter dependencies exceeded" };
	if (!desc.num_planes)
		throw std::invalid_argument{ "filter must have non-zero plane count" };
	if (desc.num_planes > FILTER_MAX_PLANES)
		throw std::invalid_argument{ "maximum number of filter outputs exceeded" };

	validate_plane_desc(desc.format);

	auto resolved_deps = resolve_node_deps(desc.num_deps, deps);

	for (unsigned p = 1; p < desc.num_deps; ++p) {
		const PlaneDescriptor &luma_desc = resolved_deps[0].first->format(resolved_deps[0].second);
		const PlaneDescriptor &desc = resolved_deps[p].first->format(resolved_deps[p].second);

		if (luma_desc.width != desc.width || luma_desc.height != desc.height)
			throw std::runtime_error{ "must have identical dimensions across all dependencies" };
	}

	reserve_next_node();
	std::unique_ptr<Node> node = make_transform_node(next_node_id(), filter, resolved_deps.data());
	node_id id = node->id();
	add_node(std::move(node));
	return id;
}

node_id Graph::add_sink(unsigned num_planes, const node_dep_desc deps[])
{
	if (!num_planes)
		throw std::invalid_argument{ "endpoint must have non-zero plane count" };
	if (num_planes > NODE_MAX_PLANES)
		throw std::invalid_argument{ "maximum number of endpoint planes exceeded" };
	if (m_sink_id >= 0)
		throw std::invalid_argument{ "sink already set" };

	auto resolved_deps = resolve_node_deps(num_planes, deps);

	reserve_next_node();

	std::unique_ptr<Node> node = make_sink_node(next_node_id(), num_planes, resolved_deps.data());
	node_id id = node->id();
	add_node(std::move(node));
	m_sink_id = id;

	try {
		compile();
	} catch (...) {
		m_nodes.pop_back();
		m_sink_id = null_node;
		throw;
	}

	return id;
}

size_t Graph::get_tmp_size() const
{
	return FrameState::metadata_size(m_nodes.size()) + m_simulation->tmp_size;
}

Graph::BufferingRequirement Graph::get_buffering_requirement() const
{
	if (m_sink_id < 0)
		throw std::invalid_argument{ "sink not set" };

	assert(m_source_ids.size() < GRAPH_MAX_ENDPOINTS);

	BufferingRequirement buffering{};
	size_t idx = 0;

	auto max_buffering = [=](node_id id)
	{
		Node *node = lookup_node(id);
		unsigned planes = node->num_planes();
		unsigned buffering = 0;

		for (unsigned p = 0; p < planes; ++p) {
			buffering = std::max(buffering, m_simulation->nodes[id].cache_mask[p]);
		}
		return buffering;
	};

	for (node_id id : m_source_ids) {
		buffering[idx++] = { id, max_buffering(id) };
	}
	buffering[idx++] = { m_sink_id, max_buffering(m_sink_id) };

	return buffering;
}

void Graph::run(const EndpointConfiguration &endpoints, void *tmp) const
{
	if (m_sink_id < 0)
		throw std::invalid_argument{ "sink not set" };

	FrameState state = prepare_frame_state(endpoints, tmp);

	// Traversal.
	Node *sink = lookup_node(m_sink_id);
	state.reset_initialized(m_nodes.size());
	sink->begin_frame(&state, 0);
	sink->process(&state, sink->format(0).height, 0);
}

} // namespace graphengine
