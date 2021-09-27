#pragma once

#ifndef GRAPHENGINE_STATE_H_
#define GRAPHENGINE_STATE_H_

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include "graph.h"
#include "types.h"

namespace graphengine {

class Simulation {
	struct node_state {
		size_t context_size;
		unsigned cursor;
		unsigned cursor_min;
		unsigned live_range;
		bool cursor_valid;
	};

	std::vector<node_state> m_node_state;
	size_t m_scratchpad_size;
	unsigned m_step;
public:
	explicit Simulation(size_t num_nodes) :
		m_node_state(num_nodes),
		m_scratchpad_size{},
		m_step{ 1 }
	{}

	bool is_live(node_id id, unsigned row) const
	{
		const node_state &s = m_node_state[id];
		return s.cursor_valid && row >= s.cursor - s.live_range;
	}

	unsigned cursor(node_id id, unsigned uninitialized_value) const
	{
		return m_node_state[id].cursor_valid ? m_node_state[id].cursor : uninitialized_value;
	}

	unsigned cursor_min(node_id id) const { return m_node_state[id].cursor_min; }

	unsigned live_range(node_id id) const { return m_node_state[id].live_range; }

	size_t context_size(node_id id) const { return m_node_state[id].context_size; }

	size_t scratchpad_size() const { return m_scratchpad_size; }

	unsigned step() const { return m_step; }

	void update_working_memory(node_id id, size_t context, size_t scratchpad)
	{
		m_node_state[id].context_size = std::max(m_node_state[id].context_size, context);
		m_scratchpad_size = std::max(m_scratchpad_size, scratchpad);
	}

	void update_live_range(node_id id, unsigned first, unsigned last)
	{
		node_state &s = m_node_state[id];

		if (!s.cursor_valid) {
			s.cursor = last;
			s.cursor_min = first;
			s.cursor_valid = true;
		}

		s.cursor = std::max(s.cursor, last);
		s.cursor_min = std::min(s.cursor_min, first);
		s.live_range = std::max(s.live_range, s.cursor - first);
	}

	void update_step(unsigned step) { m_step = std::max(m_step, step); }
};

class FrameState {
	unsigned *m_cursors;
	BufferDescriptor *m_caches;
	void **m_contexts;
	unsigned char *m_init_flags;
	void *m_scratchpad;

	Graph::Callback m_callbacks[GRAPH_MAX_ENDPOINTS];
public:
	// Additional size, not including caches, filter contexts, and scratchpad.
	static size_t metadata_size(size_t num_nodes)
	{
		size_t size = 0;
		size += sizeof(unsigned) * num_nodes; // cursors
		size += +sizeof(BufferDescriptor) * NODE_MAX_PLANES * num_nodes; // caches
		size += sizeof(void *) * num_nodes; // contexts
		size += num_nodes; // init_flags

		size = (size + 63) & ~static_cast<size_t>(63);
		return size;
	}

	// Initialize metadata section.
	explicit FrameState(unsigned char *&ptr, size_t num_nodes)
	{
		auto allocate = [&](auto *&out, size_t count) { out = reinterpret_cast<decltype(out)>(ptr); ptr += sizeof(*out) * count; };
		allocate(m_cursors, num_nodes);
		allocate(m_caches, num_nodes * NODE_MAX_PLANES);
		allocate(m_contexts, num_nodes);
		allocate(m_init_flags, num_nodes);
		m_scratchpad = nullptr;
	}

	void set_context(node_id id, void *ptr) { m_contexts[id] = ptr; }
	void set_scratchpad(void *ptr) { m_scratchpad = ptr; }

	void set_callback(size_t endpoint_id, node_id id, Graph::Callback callback)
	{
		set_context(id, reinterpret_cast<void *>(endpoint_id));
		m_callbacks[endpoint_id] = callback;
	}

	unsigned cursor(node_id id) const { return m_cursors[id]; }
	void set_cursor(node_id id, unsigned cursor) { m_cursors[id] = cursor; }

	BufferDescriptor &buffer(node_id id, unsigned plane) const { return m_caches[static_cast<size_t>(id) * NODE_MAX_PLANES + plane]; }
	void *context(node_id id) const { return m_contexts[id]; }
	void *scratchpad() const { return m_scratchpad; }

	Graph::Callback callback(node_id id) { return m_callbacks[reinterpret_cast<unsigned long long>(context(id))]; }

	bool initialized(node_id id) const { return m_init_flags[id]; }
	void set_initialized(node_id id) { m_init_flags[id] = 1; }
	void reset_initialized(size_t n) { std::fill_n(m_init_flags, n, 0); }
};

} // namespace graphengine

#endif // GRAPHENGINE_STATE_H_
