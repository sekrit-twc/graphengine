#pragma once

#ifndef GRAPHENGINE_STATE_H_
#define GRAPHENGINE_STATE_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>
#include <malloc.h>
#include "graph.h"
#include "types.h"

namespace graphengine {

class FrameState {
	std::vector<std::shared_ptr<void>> m_allocs;
	std::vector<std::array<BufferDescriptor, NODE_MAX_PLANES>> m_buffer_descriptors;
	std::vector<void *> m_filter_contexts;
	std::vector<unsigned> m_cursors;
	std::vector<Graph::Callback> m_callbacks;

	void *allocate(size_t size)
	{
		m_allocs.push_back(std::shared_ptr<void>(_aligned_malloc(size, 64), _aligned_free));
		return m_allocs.back().get();
	}
public:
	explicit FrameState(size_t num_nodes)
	{
		m_buffer_descriptors.resize(num_nodes);
		m_filter_contexts.resize(num_nodes);
		m_cursors.resize(num_nodes);
		m_callbacks.resize(num_nodes);
	}

	void allocate_filter_context(node_id id, size_t size)
	{
		m_filter_contexts[id] = allocate(size);
	}

	void allocate_internal(node_id id, unsigned plane, const PlaneDescriptor &desc)
	{
		size_t rowsize = (static_cast<size_t>(desc.width) * desc.bytes_per_sample + 63) & ~static_cast<size_t>(63);
		size_t framesize = rowsize * desc.height;
		void *ptr = allocate(framesize);

		m_buffer_descriptors[id][plane].ptr = ptr;
		m_buffer_descriptors[id][plane].stride = rowsize;
		m_buffer_descriptors[id][plane].mask = BUFFER_MAX;
	}

	void set_external(node_id id, unsigned num_planes, const BufferDescriptor desc[], Graph::Callback callback)
	{
		std::copy_n(desc, num_planes, m_buffer_descriptors[id].begin());
		m_callbacks[id] = callback;
	}

	const BufferDescriptor &buffer(node_id id, unsigned plane) { return m_buffer_descriptors[id][plane]; }

	void *context(node_id id) { return m_filter_contexts[id]; }

	unsigned &cursor(node_id id) { return m_cursors[id]; }

	const Graph::Callback &callback(node_id id) { return m_callbacks[id]; }
};

} // namespace graphengine

#endif // GRAPHENGINE_STATE_H_
