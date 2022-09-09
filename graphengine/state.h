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
#include "graphengine/graph.h"
#include "graphengine/namespace.h"
#include "graphengine/types.h"

#ifndef GRAPHENGINE_ENABLE_GUARD_PAGE
  #ifndef NDEBUG
    #define GRAPHENGINE_ENABLE_GUARD_PAGE
  #endif
#endif

namespace graphengine {
namespace GRAPHENGINE_IMPL_NAMESPACE {

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
	bool m_no_tiling;
public:
	explicit Simulation(size_t num_nodes) :
		m_node_state(num_nodes),
		m_scratchpad_size{},
		m_step{ 1 },
		m_no_tiling{}
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

	bool is_live_node(node_id id) const { return m_node_state[id].cursor_valid; }

	unsigned cursor(node_id id, unsigned uninitialized_value) const
	{
		return m_node_state[id].cursor_valid ? m_node_state[id].cursor : uninitialized_value;
	}

	unsigned cursor_min(node_id id) const { return m_node_state[id].cursor_min; }

	unsigned live_range(node_id id) const { return m_node_state[id].live_range; }

	size_t context_size(node_id id) const { return m_node_state[id].context_size; }

	size_t scratchpad_size() const { return m_scratchpad_size; }

	unsigned step() const { return m_step; }

	bool no_tiling() const { return m_no_tiling; }

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

	void set_no_tiling() { m_no_tiling = true; }
};

class FrameState {
#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
	struct guard_page {
		unsigned long page[4096 / sizeof(unsigned long)];

		static constexpr unsigned long pattern() { return static_cast<unsigned long>(0xDEADBEEFDEADBEEF); }

		guard_page() { std::fill_n(page, sizeof(page) / sizeof(page[0]), pattern()); }

		void assert_page() const
		{
			for (const auto &word : page) {
				assert(word == pattern());
			}
		}
	};
#endif
	struct node_state {
		void *context;
		unsigned left;
		unsigned right;
	};
#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
	guard_page **m_guard_pages;
#endif

	BufferDescriptor *m_caches;
	node_state *m_nodes;
	unsigned *m_cursors;
	unsigned char *m_init_flags;
	void *m_scratchpad;

	static_assert(alignof(decltype(*m_caches)) >= alignof(decltype(*m_nodes)), "wrong alignment");
	static_assert(alignof(decltype(*m_nodes)) >= alignof(decltype(*m_cursors)), "wrong alignment");
	static_assert(alignof(decltype(*m_cursors)) >= alignof(decltype(*m_init_flags)), "wrong alignment");

	Graph::Callback m_callbacks[GRAPH_MAX_ENDPOINTS];
public:
	// Additional size, not including caches, filter contexts, and scratchpad.
	static size_t metadata_size(size_t num_nodes)
	{
		size_t size = 0;
#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
		static_assert(alignof(decltype(*m_guard_pages)) >= alignof(decltype(*m_caches)), "wrong alignment");
		size += sizeof(guard_page *) * (num_guard_pages(num_nodes) + 1);
#endif
		size += sizeof(BufferDescriptor) * NODE_MAX_PLANES * num_nodes; // caches
		size += sizeof(node_state) * num_nodes; // contexts
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
#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
		allocate(m_guard_pages, num_guard_pages(num_nodes) + 1);
#endif
		allocate(m_caches, num_nodes * NODE_MAX_PLANES);
		allocate(m_nodes, num_nodes);
		allocate(m_cursors, num_nodes);
		allocate(m_init_flags, num_nodes);
		m_scratchpad = nullptr;
	}

	FrameState(const FrameState &) = delete;
	FrameState(FrameState &&) = delete;

	FrameState &operator=(const FrameState &) = delete;
	FrameState &operator=(FrameState &&) = delete;

	void set_cursor(node_id id, unsigned cursor) { m_cursors[id] = cursor; }
	void set_context(node_id id, void *ptr) { m_nodes[id].context = ptr; }
	void set_scratchpad(void *ptr) { m_scratchpad = ptr; }

#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
	static constexpr size_t guard_page_size() { return sizeof(guard_page); }

	// 1 per buffer, 1 per context, 1 before scratchpad, 1 after scratchpad, 1 null termination
	static size_t num_guard_pages(size_t num_nodes) { return num_nodes * 2 + 2; }

	void set_guard_page(size_t idx, void *ptr)
	{
		if (ptr)
			new (ptr) guard_page{};
		m_guard_pages[idx] = static_cast<guard_page *>(ptr);
	}
#endif

	void set_callback(size_t endpoint_id, node_id id, Graph::Callback callback)
	{
		if (callback) {
			m_callbacks[endpoint_id] = callback;
			m_nodes[id].context = &m_callbacks[endpoint_id];
		} else {
			m_nodes[id].context = nullptr;
		}
	}

	bool update_col_bounds(node_id id, unsigned left, unsigned right)
	{
		if (!initialized(id)) {
			m_nodes[id].left = left;
			m_nodes[id].right = right;
			return true;
		}

		bool changed = false;
		if (left < m_nodes[id].left) {
			m_nodes[id].left = left;
			changed = true;
		}
		if (right > m_nodes[id].right) {
			m_nodes[id].right = right;
			changed = true;
		}
		return changed;
	}

	void check_guard_pages() const
	{
#ifdef GRAPHENGINE_ENABLE_GUARD_PAGE
		for (guard_page **page = m_guard_pages; *page; ++page) {
			(*page)->assert_page();
		}
#endif
	}

	unsigned cursor(node_id id) const { return m_cursors[id]; }

	BufferDescriptor &buffer(ptrdiff_t offset) const { return m_caches[offset]; }
	std::pair<unsigned, unsigned> col_bounds(node_id id) const { return{ m_nodes[id].left, m_nodes[id].right }; }
	void *context(node_id id) const { return m_nodes[id].context; }
	void *scratchpad() const { return m_scratchpad; }

	bool has_callback(node_id id) const { return !!m_nodes[id].context; }
	Graph::Callback callback(node_id id) const { return *static_cast<Graph::Callback *>(m_nodes[id].context); }

	bool initialized(node_id id) const { return !!m_init_flags[id]; }
	void set_initialized(node_id id) { m_init_flags[id] = 1; }
	void reset_initialized(size_t n) { std::fill_n(m_init_flags, n, 0); }
};

} // namespace impl
} // namespace graphengine

#endif // GRAPHENGINE_STATE_H_
