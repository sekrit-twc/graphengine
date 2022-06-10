#pragma once

#ifndef GRAPHENGINE_GRAPH_H_
#define GRAPHENGINE_GRAPH_H_

#include <array>
#include <cstddef>
#include <memory>
#include "graphengine/types.h"

namespace graphengine {

class Filter;
class Node;
class Simulation;
class FrameState;

class Graph {
public:
	// Thrown on non-zero callback return value.
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

	// Sinks and sources.
	struct Endpoint {
		node_id id;
		BufferDescriptor buffer[NODE_MAX_PLANES];
		Callback callback;
	};
	typedef std::array<Endpoint, GRAPH_MAX_ENDPOINTS> EndpointConfiguration;
	typedef std::array<std::pair<node_id, unsigned>, GRAPH_MAX_ENDPOINTS> BufferingRequirement;

	virtual ~Graph() = default;

	// Graph construction methods. Strong exception safety. Graphs have up to 7 sources and 1 sink.
	// Graphs are final once a sink has been defined. No additional nodes may be inserted.
	virtual node_id add_source(unsigned num_planes, const PlaneDescriptor desc[]) = 0;

	virtual node_id add_transform(const Filter *filter, const node_dep_desc deps[]) = 0;

	virtual node_id add_sink(unsigned num_planes, const node_dep_desc deps[]) = 0;

	// Runtime execution methods. The sink node must be defined.
	virtual size_t get_cache_footprint(bool with_callbacks = true) const = 0;

	virtual size_t get_tmp_size(bool with_callbacks = true) const = 0;

	virtual BufferingRequirement get_buffering_requirement() const = 0;

	virtual void run(const EndpointConfiguration &endpoints, void *tmp) const = 0;
};


class GraphImpl : public Graph {
	class impl;

	std::unique_ptr<impl> m_impl;
public:
	static GraphImpl *from(Graph *graph) noexcept;
	static const GraphImpl *from(const Graph *graph) noexcept { return from(const_cast<Graph *>(graph)); }

	GraphImpl();

	GraphImpl(GraphImpl &&other) noexcept;

	~GraphImpl();

	GraphImpl &operator=(GraphImpl &&other) noexcept;

	// Optimization toggles.
	void set_pipelining_enabled(bool enabled);
	void set_buffer_sizing_enabled(bool enabled);
	void set_fusion_enabled(bool enabled);
	void set_planar_enabled(bool enabled);
	void set_tiling_enabled(bool enabled);

	void set_cache_size(size_t cache_size);
	void set_tile_width(unsigned tile_width);

	unsigned get_tile_width(bool with_callbacks = true) const;

	// Graph construction methods. Strong exception safety. Graphs have up to 7 sources and 1 sink.
	// Graphs are final once a sink has been defined. No additional nodes may be inserted.
	node_id add_source(unsigned num_planes, const PlaneDescriptor desc[]) override;

	node_id add_transform(const Filter *filter, const node_dep_desc deps[]) override;

	node_id add_sink(unsigned num_planes, const node_dep_desc deps[]) override;

	// Runtime execution methods. The sink node must be defined.
	size_t get_cache_footprint(bool with_callbacks = true) const override;

	size_t get_tmp_size(bool with_callbacks = true) const override;

	BufferingRequirement get_buffering_requirement() const override;

	void run(const EndpointConfiguration &endpoints, void *tmp) const override;
};

} // namespace graphengine

#endif // GRAPHENGINE_GRAPH_H_
