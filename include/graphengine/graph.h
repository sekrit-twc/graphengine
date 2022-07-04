#ifndef GRAPHENGINE_GRAPH_H_
#define GRAPHENGINE_GRAPH_H_

#include <cstddef>
#include "graphengine/namespace.h"
#include "graphengine/types.h"

struct graphengine_graph {
protected:
	~graphengine_graph() = default;
};

namespace graphengine {

class Filter;

/**
 * Filter graph.
 *
 * A graph is a collection of nodes, which may represent either external images
 * (endpoints) or the outputs of filters. The graph tracks the dependencies
 * between nodes and invokes filters in the appropriate order. Graphs are
 * insert-only: nodes can not be queried or removed.
 */
class Graph : public graphengine_graph {
public:
	/**
	 * Endpoint node callback.
	 *
	 * Just as the inputs and outputs of filters are buffered, the endpoint
	 * planes can also be produced on-demand. The graph will invoke the callback
	 * when a row from an input plane is required or a row from an output plane
	 * is ready.
	 */
	struct Callback {
		/**
		 * User-provided callback function.
		 *
		 * For input planes, the callback ensures the requested tile is present
		 * in the endpoint buffer upon return. For output planes, the callback
		 * is invoked when the graph has produced the notified tile in the
		 * endpoint buffer.
		 *
		 * In case of a subsampled image, the callback handles multiple rows
		 * (e.g. rows [i, i + 2) if the image is 4:2:0 subsampled).
		 *
		 * @param user opaque pointer
		 * @param i row
		 * @param left,right span
		 * @return zero on success, non-zero on error
		 */
		int (*func)(void *user, unsigned i, unsigned left, unsigned right);

		/** Passed to callback. */
		void *user;

		/** Default constructor. */
		constexpr Callback(std::nullptr_t = nullptr) : func{}, user{} {}

		/** Construct with user callback. */
		constexpr Callback(int (*func)(void *, unsigned, unsigned, unsigned), void *user) : func{func}, user{user} {}

		/** True if non-null. */
		constexpr explicit operator bool() const { return !!func; }
	};

	/** Endpoint descriptor. */
	struct Endpoint {
		node_id id = null_node;         /**< Node id. */
		const BufferDescriptor *buffer; /**< Planes. */
		Callback callback;              /**< Callback. */
	};

	/** Buffering requirements for endpoint. */
	struct Buffering {
		node_id id = null_node; /**< Node id. */
		unsigned mask = 0;      /**< Minimum buffer mask. */
	};

	typedef detail::array<Buffering, GRAPH_MAX_ENDPOINTS> BufferingRequirement;
protected:
	Graph &operator=(const Graph &) = default;
public:
	virtual ~Graph() = default;

	/**
	 * Define a source node.
	 *
	 * Up to {@p GRAPH_MAX_ENDPOINTS - 1} sources may be defined. The plane
	 * dimensions must be related to each other by a power-of-2 factor
	 * (subsampling), and the first plane must have the greatest dimensions
	 * (i.e. unsubsampled).
	 *
	 * @param num_planes number of input planes
	 * @param[in] desc array of plane descriptors
	 * @return node id
	 * @throw Exception on precondition violated or internal error
	 */
	virtual node_id add_source(unsigned num_planes, const PlaneDescriptor desc[]) = 0;

	/**
	 * Define a transform/filter node.
	 *
	 * Filters accept up to {@p FILTER_MAX_DEPS} inputs and produce up to
	 * {@p FILTER_MAX_PLANES} outputs. All input planes are of the same
	 * dimensions, but the bytes per sample may differ. All output planes are
	 * of identical dimensions, including the bytes per sample.
	 *
	 * @param[in] filter filter instance, not owned by Graph
	 * @param[in] deps input plane handles
	 * @return node id
	 * @throw Exception on precondition violated or internal error
	 */
	virtual node_id add_transform(const Filter *filter, const node_dep_desc deps[]) = 0;

	/**
	 * Define the sink node.
	 *
	 * The sink node is the output of the graph and is made up of planes from
	 * other nodes in the graph. As with source nodes, the sink planes may be
	 * subsampled by a power-of-2. Defining a sink finalizes the graph, and no
	 * additional nodes may be created.
	 *
	 * @param num_planes number of output planes
	 * @param[in] deps output plane handles
	 * @return node id
	 * @throw Exception on precondition violated or internal error
	 */
	virtual node_id add_sink(unsigned num_planes, const node_dep_desc deps[]) = 0;

	/**
	 * Query the working set size of the graph.
	 *
	 * Computes the estimated amount of memory accessed while executing the
	 * graph. When endpoint callbacks are present, certain traversal
	 * optimizations are disabled and the working set may increase.
	 *
	 * @param with_callbacks whether endpoint callbacks are present
	 * @return size
	 * @throw Exception on sink not defined
	 */
	virtual size_t get_cache_footprint(bool with_callbacks = true) const = 0;

	/**
	 * Query the amount of additional memory required to execute the graph.
	 *
	 * Graph execution is non-allocating. All non-stack allocated memory used
	 * during processing is provided by the caller.
	 *
	 * @param with_callbacks whether endpoint callbacks are present
	 * @return size
	 * @throw Exception on sink not defined
	 */
	virtual size_t get_tmp_size(bool with_callbacks = true) const = 0;

	/**
	 * Query the buffering required for endpoint nodes.
	 *
	 * When using endpoint callbacks, each node must be buffered according to
	 * the access pattern of the filters in the graph. If not using callbacks,
	 * all endpoints should be allocated with {@p BUFFER_MAX}.
	 *
	 * @return buffering
	 * @throw Exception on sink not defined
	 */
	virtual BufferingRequirement get_buffering_requirement() const = 0;

	/**
	 * Execute the graph.
	 *
	 * Graph execution is non-allocating, non-failing, and thread-safe. Graph
	 * processing may occur in parallel across multiple threads, so long as the
	 * addresses of the temporary buffers and output planes are unique.
	 *
	 * Execution fails if and only if an endpoint callback returns non-zero.
	 *
	 * @param[in] endpoints input/output buffers and callbacks
	 * @param[out] tmp working memory
	 * @throw Exception on callback error
	 * @see get_tmp_size()
	 */
	virtual void run(const Endpoint endpoints[], void *tmp) const = 0;
};

/**
 * Partial graph.
 *
 * As {@p Graph} is an append-only structure, subgraphs can be used to record a
 * reusable sequence of filters that can be inserted into multiple graphs.
 */
class SubGraph {
public:
	/** Mapping between internal id namespace and real graph ids. */
	struct Mapping {
		node_id internal_id = null_node; /**< Internal node id. */
		node_dep_desc external_dep;      /**< External plane corresponding to node. */
	};
protected:
	SubGraph &operator=(const SubGraph &) = default;
public:
	virtual ~SubGraph() = default;

	/**
	 * Define an input plane.
	 *
	 * Sources/sinks in a subgraph are modeled as having only one plane, as the
	 * subgraph endpoints can be any plane in a main graph.
	 *
	 * @return id
	 * @throw Exception on internal error
	 */
	virtual node_id add_source() = 0;

	/**
	 * Define an output plane.
	 *
	 * As with sources, subgraph sinks have only one plane. Unlike main graphs,
	 * defining sinks does not finalize the subgraph.
	 *
	 * @param dep internal plane handle
	 * @return id
	 * @throw Exception on precondition violated or internal error
	 */
	virtual node_id add_sink(const node_dep_desc &dep) = 0;

	/**
	 * Define a transform node.
	 *
	 * @see Graph::add_transform()
	 */
	virtual node_id add_transform(const Filter *filter, const node_dep_desc deps[]) = 0;

	/**
	 * Insert the subgraph into a main graph.
	 *
	 * Mappings are used to translate node ids from the internal namespace to
	 * the ids used by the main graph. Source mappings need not be provided for
	 * source planes that are defined but not referenced.
	 *
	 * @param[out] graph graph
	 * @param num_sources number of source mappings provided
	 * @param[in] sources array of source mappings
	 * @param[out] sinks array of sink mappings
	 * @throw Exception on preconditions violated
	 */
	virtual void connect(Graph *graph, size_t num_sources, const Mapping sources[], Mapping sinks[]) const = 0;
};


/** Non-abstract classes. Do not access across library boundaries. */
namespace GRAPHENGINE_IMPL_NAMESPACE {

/** Graph implementation. */
class GraphImpl : public Graph {
	class impl;

	detail::unique_ptr<impl> m_impl;
public:
	/** Perform a dynamic_cast. */
	static GraphImpl *from(Graph *graph) noexcept;
	static const GraphImpl *from(const Graph *graph) noexcept { return from(const_cast<Graph *>(graph)); }

	/**
	 * Default constructor.
	 *
	 * @throw Exception on out of memory
	 */
	GraphImpl();

	/** Move constructor. */
	GraphImpl(GraphImpl &&other) noexcept;

	/** Destructor. */
	~GraphImpl();

	/** Move assignment. */
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

	// Graph virtual functions.
	node_id add_source(unsigned num_planes, const PlaneDescriptor desc[]) override;
	node_id add_transform(const Filter *filter, const node_dep_desc deps[]) override;
	node_id add_sink(unsigned num_planes, const node_dep_desc deps[]) override;

	size_t get_cache_footprint(bool with_callbacks = true) const override;
	size_t get_tmp_size(bool with_callbacks = true) const override;

	BufferingRequirement get_buffering_requirement() const override;

	void run(const Endpoint endpoints[], void *tmp) const override;
};

/** SubGraph implementation. */
class SubGraphImpl : public SubGraph {
	class impl;

	detail::unique_ptr<impl> m_impl;
public:
	/**
	 * Default constructor.
	 *
	 * @throw Exception on out of memory
	 */
	SubGraphImpl();

	/** Move constructor. */
	SubGraphImpl(SubGraphImpl &&other) noexcept;

	/** Destructor. */
	~SubGraphImpl();

	/** Move assignment. */
	SubGraphImpl &operator=(SubGraphImpl &&other) noexcept;

	// SubGraph virtual functions.
	node_id add_source() override;
	node_id add_sink(const node_dep_desc &dep) override;
	node_id add_transform(const Filter *filter, const node_dep_desc deps[]) override;

	void connect(Graph *graph, size_t num_sources, const Mapping sources[], Mapping sinks[]) const override;
};

} // namespace impl

using GRAPHENGINE_IMPL_NAMESPACE::GraphImpl;
using GRAPHENGINE_IMPL_NAMESPACE::SubGraphImpl;

} // namespace graphengine

#endif // GRAPHENGINE_GRAPH_H_
