#pragma once

#ifndef GRAPHENGINE_NODE_H_
#define GRAPHENGINE_NODE_H_

#include <memory>
#include "types.h"

namespace graphengine {

class FrameState;

class Filter;


class Node {
	node_id m_id;
protected:
	Node(node_id id) : m_id{ id } {}
public:
	Node(const Node &) = delete;

	virtual ~Node() = default;

	Node &operator=(const Node &) = delete;

	node_id id() const { return m_id; }

	virtual bool sourcesink() const noexcept = 0;

	virtual unsigned num_planes() const noexcept = 0;

	virtual PlaneDescriptor format(unsigned plane) const noexcept = 0;

	virtual void initialize_frame_state(FrameState *state) const noexcept = 0;

	// |plane| is used to translate row offsets of subsampled sources.
	// All node planes are processed on every call. Transform nodes are never subsampled.
	virtual void process(FrameState *state, unsigned last, unsigned plane) const = 0;
};


std::unique_ptr<Node> make_source_node(node_id id, unsigned num_planes, const PlaneDescriptor desc[]);
std::unique_ptr<Node> make_sink_node(node_id id, unsigned num_planes, const std::pair<Node *, unsigned> parents[]);
std::unique_ptr<Node> make_transform_node(node_id id, const Filter *filter, const node_dep deps[]);

} // namespace graphengine

#endif // GRAPHENGINE_NODE_H_
