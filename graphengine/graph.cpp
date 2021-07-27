#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
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

}


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

void Graph::compile() {}

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
	return 0;
}

Graph::BufferingRequirement Graph::get_buffering_requirement() const
{
	if (m_sink_id < 0)
		throw std::invalid_argument{ "sink not set" };

	assert(m_source_ids.size() < GRAPH_MAX_ENDPOINTS);

	BufferingRequirement buffering{};
	size_t idx = 0;

	for (node_id id : m_source_ids) {
		buffering[idx++] = { id, BUFFER_MAX };
	}
	buffering[idx++] = { m_sink_id, BUFFER_MAX };

	return buffering;
}

void Graph::run(const EndpointConfiguration &endpoints, void *tmp) const
{
	if (m_sink_id < 0)
		throw std::invalid_argument{ "sink not set" };

	FrameState state{ m_nodes.size() };

	// Set the endpoints.
	for (unsigned n = 0; n < m_source_ids.size() + 1; ++n) {
		if (endpoints[n].id < 0)
			throw std::runtime_error{ "missing buffer for endpoint" };

		unsigned num_planes = lookup_node(endpoints[n].id)->num_planes();
		state.set_external(endpoints[n].id, num_planes, endpoints[n].buffer, endpoints[n].callback);
	}

	// Traversal.
	Node *sink = lookup_node(m_sink_id);
	sink->initialize_frame_state(&state);
	sink->process(&state, sink->format(0).height, 0);
}

} // namespace graphengine
