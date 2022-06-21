#pragma once

#ifndef GRAPHENGINE_TYPES_H_
#define GRAPHENGINE_TYPES_H_

#include <cstddef>
#include <utility>

namespace graphengine {

constexpr unsigned BUFFER_MAX = ~0U;

constexpr unsigned FILTER_MAX_DEPS = 3;
constexpr unsigned FILTER_MAX_PLANES = 3;
constexpr unsigned NODE_MAX_PLANES = 4;

constexpr unsigned GRAPH_MAX_ENDPOINTS = 8;

typedef int node_id;
typedef std::pair<node_id, unsigned> node_dep_desc;

constexpr node_id null_node = -1;
constexpr node_id node_id_max = 1023;
constexpr node_dep_desc null_dep{ null_node, 0 };


struct PlaneDescriptor {
	unsigned width;
	unsigned height;
	unsigned bytes_per_sample;
};

struct BufferDescriptor {
	void *ptr;
	ptrdiff_t stride;
	unsigned mask;

	template <class T = void>
	T *get_line(unsigned i) const
	{
		return reinterpret_cast<T *>(static_cast<unsigned char *>(ptr) + static_cast<ptrdiff_t>(i & mask) * stride);
	}
};

} // namespace graphengine

#endif // GRAPHENGINE_TYPES_H_
