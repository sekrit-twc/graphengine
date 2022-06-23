#pragma once

#ifndef GRAPHENGINE_NODE_H_
#define GRAPHENGINE_NODE_H_

#include <memory>
#include <utility>
#include "graphengine/namespace.h"
#include "graphengine/types.h"

namespace graphengine {

class Filter;

namespace GRAPHENGINE_IMPL_NAMESPACE {

class Simulation;
class FrameState;

class Node {
	node_id m_id;
	ptrdiff_t m_cache_location[NODE_MAX_PLANES];
protected:
	Node(node_id id);
public:
	Node(const Node &) = delete;

	virtual ~Node() = default;

	Node &operator=(const Node &) = delete;

	node_id id() const { return m_id; }

	ptrdiff_t cache_location(unsigned plane) const { return m_cache_location[plane]; }

	void set_cache_location(unsigned plane, ptrdiff_t offset) { m_cache_location[plane] = offset; }

	// Reference counting.
	virtual unsigned ref_count(unsigned plane) const noexcept = 0;

	virtual void add_ref(unsigned plane) noexcept = 0;

	virtual void dec_ref(unsigned plane) noexcept = 0;

	// Informational methods.
	virtual bool sourcesink() const noexcept = 0;

	virtual unsigned subsample_w(unsigned plane) const noexcept = 0;

	virtual unsigned subsample_h(unsigned plane) const noexcept = 0;

	virtual unsigned num_planes() const noexcept = 0;

	virtual PlaneDescriptor format(unsigned plane) const noexcept = 0;

	// Node-fusion methods.
	virtual void apply_node_fusion() noexcept = 0;

	// Analysis methods used internally by Graph.
	virtual bool reachable(node_id id, unsigned plane) const noexcept = 0;

	virtual void trace_working_memory(Simulation *sim) const noexcept = 0;

	virtual void trace_access_pattern(Simulation *sim, unsigned first_row, unsigned last_row, unsigned plane) const noexcept = 0;

	// Setup method called before Node::process.
	virtual void begin_frame(FrameState *state, unsigned left, unsigned right, unsigned plane) const noexcept = 0;

	// |plane| is used to translate row offsets of subsampled sources.
	// All node planes are processed on every call. Transform nodes are never subsampled.
	virtual void process(FrameState *state, unsigned last_row, unsigned plane) const = 0;
};

typedef std::pair<Node *, unsigned> node_dep;


std::unique_ptr<Node> make_source_node(node_id id, unsigned num_planes, const PlaneDescriptor desc[]);
std::unique_ptr<Node> make_sink_node(node_id id, unsigned num_planes, const node_dep parents[]);
std::unique_ptr<Node> make_transform_node(node_id id, const Filter *filter, const node_dep deps[]);

} // namespace impl
} // namespace graphengine

#endif // GRAPHENGINE_NODE_H_
