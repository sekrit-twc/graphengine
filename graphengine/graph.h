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

	std::vector<std::unique_ptr<Filter>> m_pipeline_wrappers;
	std::unique_ptr<Filter> m_copy_filters[NODE_MAX_PLANES];
	std::vector<std::unique_ptr<Node>> m_nodes;
	std::vector<node_id> m_source_ids;
	std::unique_ptr<SimulationResult> m_simulation_result;
	std::unique_ptr<SimulationResult> m_planar_simulation_result[NODE_MAX_PLANES];
	node_dep_desc m_planar_deps[NODE_MAX_PLANES];
	node_id m_sink_id = null_node;
	unsigned m_tile_width = 0;

	struct {
		unsigned pipelining_disabled : 1;
		unsigned buffer_sizing_disabled : 1;
		unsigned fusion_disabled : 1;
		unsigned planar_disabled : 1;
		unsigned tiling_disabled : 1;
	} m_flags = {};

	node_id next_node_id() const;

	Node *node(node_id id) const;
	Node *lookup_node(node_id id) const;

	std::array<node_dep, NODE_MAX_PLANES> resolve_node_deps(unsigned num_deps, const node_dep_desc deps[]) const;

	void reserve_next_node();

	void add_node(std::unique_ptr<Node> node);

	node_id add_transform_internal(const Filter *filter, const node_dep_desc deps[]);

	std::unique_ptr<Simulation> begin_compile(unsigned num_planes);

	void compile_plane(Simulation *sim, const node_dep &dep) noexcept;

	void compile(Simulation *sim, unsigned num_planes, node_dep deps[]) noexcept;

	bool can_run_planar() const;

	FrameState prepare_frame_state(const SimulationResult &sim, const EndpointConfiguration &endpoints, void *tmp) const;

	void run_node(Node *node, const SimulationResult &sim, const EndpointConfiguration &endpoints, unsigned tile_width, unsigned plane, void *tmp) const;
public:
	Graph();

	~Graph();

	// Optimization toggles.
	void set_pipelining_enabled(bool enabled) { m_flags.pipelining_disabled = !enabled; }
	void set_buffer_sizing_enabled(bool enabled) { m_flags.buffer_sizing_disabled = !enabled; }
	void set_fusion_enabled(bool enabled) { m_flags.fusion_disabled = !enabled; }
	void set_planar_enabled(bool enabled) { m_flags.planar_disabled = !enabled; }
	void set_tiling_enabled(bool enabled) { m_flags.tiling_disabled = !enabled; }

	void set_tile_width(unsigned tile_width) { m_tile_width = tile_width; }

	// Graph construction methods. Strong exception safety. Graphs have up to 7 sources and 1 sink.
	// Graphs are final once a sink has been defined. No additional nodes may be inserted.
	node_id add_source(unsigned num_planes, const PlaneDescriptor desc[]);

	node_id add_transform(const Filter *filter, const node_dep_desc deps[]);

	node_id add_sink(unsigned num_planes, const node_dep_desc deps[]);

	// Runtime execution methods. The sink node must be defined.
	size_t get_cache_footprint(bool with_callbacks = true) const;

	size_t get_tmp_size(bool with_callbacks = true) const;

	unsigned get_tile_width(bool with_callbacks = true) const;

	BufferingRequirement get_buffering_requirement() const;

	void run(const EndpointConfiguration &endpoints, void *tmp) const;
};

} // namespace graphengine

#endif // GRAPHENGINE_GRAPH_H_
