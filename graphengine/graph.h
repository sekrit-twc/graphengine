#pragma once

#ifndef GRAPHENGINE_GRAPH_H_
#define GRAPHENGINE_GRAPH_H_

#include <array>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>
#include "types.h"

namespace graphengine {

class Filter;
class Node;
class Simulation;
class FrameState;

class Graph {
public:
	struct CallbackError { int code; };

	/**
	 * User-defined I/O callback functor.
	 */
	class Callback {
		typedef int (*func_type)(void *user, unsigned i, unsigned left, unsigned right);

		func_type m_func;
		void *m_user;
	public:
		/**
		 * Default construct callback, creating a null callback.
		 */
		Callback(std::nullptr_t x = nullptr) : m_func{}, m_user{} {}

		/**
		 * Construct a callback from user-defined function.
		 *
		 * @param func function pointer
		 * @param user user private data
		 */
		Callback(func_type func, void *user) : m_func{ func }, m_user{ user } {}

		/**
		 * Check if callback is set.
		 *
		 * @return true if callback is not null, else false
		 */
		explicit operator bool() const { return !!m_func; }

		/**
		 * Invoke user-defined callback.
		 *
		 * @param i row index of line to read/write
		 * @param left left column index
		 * @param right right column index, plus one
		 */
		void operator()(unsigned i, unsigned left, unsigned right) const;
	};

	struct Endpoint {
		node_id id;
		BufferDescriptor buffer[NODE_MAX_PLANES];
		Callback callback;
	};
	typedef std::array<Endpoint, GRAPH_MAX_ENDPOINTS> EndpointConfiguration;
	typedef std::array<std::pair<node_id, unsigned>, GRAPH_MAX_ENDPOINTS> BufferingRequirement;
private:
	struct SimulationResult;

	std::vector<std::unique_ptr<Node>> m_nodes;
	std::vector<node_id> m_source_ids;
	std::unique_ptr<SimulationResult> m_simulation_result;

	node_id m_sink_id = null_node;

	node_id next_node_id() const;

	Node *lookup_node(node_id id) const;

	std::array<node_dep, NODE_MAX_PLANES> resolve_node_deps(unsigned num_deps, const node_dep_desc deps[]) const;

	void reserve_next_node();

	void add_node(std::unique_ptr<Node> node);

	std::unique_ptr<Simulation> begin_compile();

	void compile(Simulation *sim) noexcept;

	FrameState prepare_frame_state(const EndpointConfiguration &endpoints, void *tmp) const;
public:
	Graph();

	~Graph();

	node_id add_source(unsigned num_planes, const PlaneDescriptor desc[]);

	node_id add_transform(const Filter *filter, const node_dep_desc deps[]);

	node_id add_sink(unsigned num_planes, const node_dep_desc deps[]);

	size_t get_tmp_size() const;

	BufferingRequirement get_buffering_requirement() const;

	void run(const EndpointConfiguration &endpoints, void *tmp) const;
};

} // namespace graphengine

#endif // GRAPHENGINE_GRAPH_H_
