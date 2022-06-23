#pragma once

#ifndef GRAPHENGINE_GRAPH_H_
#define GRAPHENGINE_GRAPH_H_

#include <cstddef>
#include "graphengine/namespace.h"
#include "graphengine/types.h"

namespace graphengine {

class Filter;

class Graph {
public:
	struct Callback {
		int (*func)(void *user, unsigned i, unsigned left, unsigned right);
		void *user;

		Callback(std::nullptr_t x = nullptr) : func{}, user{} {}
		Callback(int (*func)(void *, unsigned, unsigned, unsigned), void *user) : func{func}, user{user} {}

		explicit operator bool() const { return !!func; }
	};

	// Sinks and sources.
	struct Endpoint {
		node_id id = null_node;
		BufferDescriptor buffer[NODE_MAX_PLANES];
		Callback callback;
	};

	struct Buffering {
		node_id id = null_node;
		unsigned mask = 0;
	};

	typedef detail::array<Endpoint, GRAPH_MAX_ENDPOINTS> EndpointConfiguration;
	typedef detail::array<Buffering, GRAPH_MAX_ENDPOINTS> BufferingRequirement;
protected:
	Graph &operator=(const Graph &) = default;
public:
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


class SubGraph {
public:
	struct Mapping {
		node_id internal_id = null_node;
		node_dep_desc external_dep;
	};
protected:
	SubGraph &operator=(const SubGraph &) = default;
public:
	virtual ~SubGraph() = default;

	// Subgraph construction methods. Strong exception safety. Unlike main graphs, subgraphs
	// can have an arbitrary number of sources and sinks, which represent boundary nodes
	// in the main graph. Subgraph sources/sinks have only one plane.
	virtual node_id add_source() = 0;

	virtual node_id add_sink(const node_dep_desc &dep) = 0;

	virtual node_id add_transform(const Filter *filter, const node_dep_desc deps[]) = 0;

	// Connect subgraph to main graph.
	virtual void connect(Graph *graph, size_t num_sources, const Mapping sources[], Mapping sinks[]) const = 0;
};


namespace GRAPHENGINE_IMPL_NAMESPACE {

class GraphImpl : public Graph {
	class impl;

	detail::unique_ptr<impl> m_impl;
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


class SubGraphImpl : public SubGraph {
	class impl;

	detail::unique_ptr<impl> m_impl;
public:
	SubGraphImpl();

	SubGraphImpl(SubGraphImpl &&other) noexcept;

	~SubGraphImpl();

	SubGraphImpl &operator=(SubGraphImpl &&other) noexcept;

	// Subgraph construction methods. Strong exception safety. Unlike main graphs, subgraphs
	// can have an arbitrary number of sources and sinks, which represent boundary nodes
	// in the main graph. Subgraph sources/sinks have only one plane.
	node_id add_source() override;

	node_id add_sink(const node_dep_desc &dep) override;

	node_id add_transform(const Filter *filter, const node_dep_desc deps[]) override;

	// Connect subgraph to main graph.
	void connect(Graph *graph, size_t num_sources, const Mapping sources[], Mapping sinks[]) const override;
};

} // namespace impl

using GRAPHENGINE_IMPL_NAMESPACE::GraphImpl;
using GRAPHENGINE_IMPL_NAMESPACE::SubGraphImpl;

} // namespace graphengine

#endif // GRAPHENGINE_GRAPH_H_
