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

	void reset()
	{
		m_node_state.assign(m_node_state.size(), node_state{});
		m_scratchpad_size = 0;
		m_step = 1;
	}

	bool is_live(node_id id, node_id cache_id, unsigned row) const
	{
		const node_state &state = m_node_state[id];
		const node_state &cache_state = m_node_state[cache_id];
		return state.cursor_valid && row >= state.cursor - cache_state.live_range;
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

	void update_cursor_range(node_id id, unsigned first, unsigned last)
	{
		node_state &s = m_node_state[id];

		if (!s.cursor_valid) {
			s.cursor = last;
			s.cursor_min = first;
			s.cursor_valid = true;
		}

		s.cursor = std::max(s.cursor, last);
		s.cursor_min = std::min(s.cursor_min, first);
	}

	void update_live_range(node_id id, node_id cache_id, unsigned first, unsigned last)
	{
		node_state &state = m_node_state[id];
		node_state &cache_state = m_node_state[cache_id];
		cache_state.live_range = std::max(cache_state.live_range, std::max(state.cursor, last) - first);
	}

	void update_step(unsigned step) { m_step = std::max(m_step, step); }
};

class FrameState {
	BufferDescriptor *m_caches;
	void **m_contexts;
	unsigned *m_cursors;
	unsigned char *m_init_flags;
	void *m_scratchpad;

	static_assert(alignof(decltype(*m_caches)) >= alignof(decltype(*m_contexts)), "wrong alignment");
	static_assert(alignof(decltype(*m_contexts)) >= alignof(decltype(*m_cursors)), "wrong alignment");
	static_assert(alignof(decltype(*m_cursors)) >= alignof(decltype(*m_init_flags)), "wrong alignment");

	Graph::Callback m_callbacks[GRAPH_MAX_ENDPOINTS];
public:
	// Additional size, not including caches, filter contexts, and scratchpad.
	static size_t metadata_size(size_t num_nodes)
	{
		size_t size = 0;
		size += sizeof(BufferDescriptor) * NODE_MAX_PLANES * num_nodes; // caches
		size += sizeof(void *) * num_nodes; // contexts
		size += sizeof(unsigned) * num_nodes; // cursors
		size += num_nodes; // init_flags

		size = (size + 63) & ~static_cast<size_t>(63);
		return size;
	}

	static ptrdiff_t cache_descriptor_offset(node_id id, unsigned plane)
	{
		return static_cast<size_t>(id) * NODE_MAX_PLANES + plane;
	}

	static node_id cache_descriptor_offset_to_node(ptrdiff_t offset)
	{
		return static_cast<node_id>(static_cast<size_t>(offset) / NODE_MAX_PLANES);
	}

	// Initialize metadata section.
	explicit FrameState(unsigned char *&ptr, size_t num_nodes)
	{
		auto allocate = [&](auto *&out, size_t count) { out = reinterpret_cast<decltype(out)>(ptr); ptr += sizeof(*out) * count; };
		allocate(m_caches, num_nodes * NODE_MAX_PLANES);
		allocate(m_contexts, num_nodes);
		allocate(m_cursors, num_nodes);
		allocate(m_init_flags, num_nodes);
		m_scratchpad = nullptr;
	}

	void set_context(node_id id, void *ptr) { m_contexts[id] = ptr; }
	void set_scratchpad(void *ptr) { m_scratchpad = ptr; }

	void set_callback(size_t endpoint_id, node_id id, Graph::Callback callback)
	{
		if (callback) {
			m_callbacks[endpoint_id] = callback;
			m_contexts[id] = &m_callbacks[endpoint_id];
		} else {
			m_contexts[id] = nullptr;
		}
	}

	unsigned cursor(node_id id) const { return m_cursors[id]; }
	void set_cursor(node_id id, unsigned cursor) { m_cursors[id] = cursor; }

	BufferDescriptor &buffer(ptrdiff_t offset) const { return m_caches[offset]; }
	void *context(node_id id) const { return m_contexts[id]; }
	void *scratchpad() const { return m_scratchpad; }

	bool has_callback(node_id id) const { return !!m_contexts[id]; }
	Graph::Callback callback(node_id id) const { return *static_cast<Graph::Callback *>(m_contexts[id]);}

	bool initialized(node_id id) const { return m_init_flags[id]; }
	void set_initialized(node_id id) { m_init_flags[id] = 1; }
	void reset_initialized(size_t n) { std::fill_n(m_init_flags, n, 0); }
};

} // namespace graphengine

#endif // GRAPHENGINE_STATE_H_
