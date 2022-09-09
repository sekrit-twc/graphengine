#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include "graphengine/filter.h"
#include "node.h"
#include "state.h"

namespace graphengine {
namespace GRAPHENGINE_IMPL_NAMESPACE {

namespace {

void invoke_callback(const Graph::Callback &cb, unsigned i, unsigned left, unsigned right)
{
	if (cb.func(cb.user, i, left, right))
		throw Exception{ Exception::USER_CALLBACK, "user callback failed" };
}

unsigned calculate_subsampling_ratios(unsigned num_planes, const PlaneDescriptor desc[], unsigned subsample_w[], unsigned subsample_h[])
{
	unsigned step = 1;

	double width = desc[0].width;
	double height = desc[0].height;

	subsample_w[0] = 0;
	subsample_h[0] = 0;

	for (unsigned p = 1; p < num_planes; ++p) {
		double ratio_w = width / desc[p].width;
		double ratio_h = height / desc[p].height;

		if (ratio_w != 1.0 && ratio_w != 2.0 && ratio_w != 4.0)
			throw Exception{ Exception::INVALID_DIMENSIONS, "only 1, 2, and 4x subsampling supported" };
		if (ratio_h != 1.0 && ratio_h != 2.0 && ratio_h != 4.0)
			throw Exception{ Exception::INVALID_DIMENSIONS, "only 1, 2, and 4x subsampling supported" };

		subsample_w[p] = std::lrint(std::log2(ratio_w));
		subsample_h[p] = std::lrint(std::log2(ratio_h));
		step = std::max(step, 1U << subsample_h[p]);
	}

	return step;
}


class SourceNode : public Node {
	PlaneDescriptor m_desc[NODE_MAX_PLANES] = {};
	unsigned m_ref_count[NODE_MAX_PLANES] = {};
	unsigned m_num_planes;
	unsigned m_step = 1;
	unsigned m_subsample_w[NODE_MAX_PLANES] = {};
	unsigned m_subsample_h[NODE_MAX_PLANES] = {};
public:
	SourceNode(node_id id, unsigned num_planes, const PlaneDescriptor desc[]) :
		Node{ id },
		m_num_planes{ num_planes }
	{
		assert(m_num_planes);
		assert(m_num_planes <= NODE_MAX_PLANES);
		std::copy_n(desc, num_planes, m_desc);
		m_step = calculate_subsampling_ratios(m_num_planes, m_desc, m_subsample_w, m_subsample_h);
	}

	unsigned ref_count(unsigned plane) const noexcept override { return m_ref_count[plane]; }

	void add_ref(unsigned plane) noexcept override { assert(plane < m_num_planes); ++m_ref_count[plane]; }

	void dec_ref(unsigned plane) noexcept override { assert(m_ref_count[plane]); --m_ref_count[plane]; }

	bool sourcesink() const noexcept override { return true; }

	unsigned subsample_w(unsigned plane) const noexcept override { return m_subsample_w[plane]; }

	unsigned subsample_h(unsigned plane) const noexcept override { return m_subsample_h[plane]; }

	unsigned num_planes() const noexcept override { return m_num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept override { return m_desc[plane]; }

	void apply_node_fusion() noexcept override {}

	bool reachable(node_id id, unsigned plane) const noexcept override { return false; }

	void trace_working_memory(Simulation *sim) const noexcept override {}

	void trace_access_pattern(Simulation *sim, unsigned first_row, unsigned last_row, unsigned plane) const noexcept override
	{
		// Normalize row addresses.
		first_row <<= m_subsample_h[plane];
		first_row = first_row - first_row % m_step;

		last_row <<= m_subsample_h[plane];
		last_row = last_row % m_step ? last_row + (m_step - last_row % m_step) : last_row;

		sim->update_cursor_range(id(), first_row, last_row);
		sim->update_live_range(id(), id(), first_row, last_row);
	}

	void begin_frame(FrameState *state, unsigned left, unsigned right, unsigned plane) const noexcept override
	{
		state->update_col_bounds(id(), left << m_subsample_w[plane], right << m_subsample_w[plane]);
		state->set_initialized(id());
	}

	void process(FrameState *state, unsigned last_row, unsigned plane) const override
	{
		if (!state->has_callback(id()))
			return;

		unsigned cursor = state->cursor(id());
		last_row = last_row << m_subsample_h[plane];
		if (cursor >= last_row)
			return;

		Graph::Callback callback = state->callback(id());
		std::pair<unsigned, unsigned> cols = state->col_bounds(id());

		for (; cursor < last_row; cursor += m_step) {
			invoke_callback(callback, cursor, cols.first, cols.second);
			state->check_guard_pages();
		}

		state->set_cursor(id(), cursor);
	}
};


class SinkNode : public Node {
	node_dep m_parents[NODE_MAX_PLANES] = {};
	unsigned m_num_planes;
	unsigned m_step = 1;
	unsigned m_subsample_w[NODE_MAX_PLANES] = {};
	unsigned m_subsample_h[NODE_MAX_PLANES] = {};
public:
	SinkNode(node_id id, unsigned num_planes, const std::pair<Node *, unsigned> parents[]) :
		Node{ id },
		m_num_planes{ num_planes }
	{
		assert(m_num_planes);
		assert(m_num_planes <= NODE_MAX_PLANES);
		std::copy_n(parents, num_planes, m_parents);

		PlaneDescriptor desc[NODE_MAX_PLANES];
		for (unsigned p = 0; p < m_num_planes; ++p) {
			desc[p] = format(p);
		}
		m_step = calculate_subsampling_ratios(m_num_planes, desc, m_subsample_w, m_subsample_h);
	}

	unsigned ref_count(unsigned plane) const noexcept override { return 1; }

	void add_ref(unsigned plane) noexcept override {}

	void dec_ref(unsigned plane) noexcept override {}

	bool sourcesink() const noexcept override { return true; }

	unsigned subsample_w(unsigned plane) const noexcept override { return m_subsample_w[plane]; }

	unsigned subsample_h(unsigned plane) const noexcept override { return m_subsample_h[plane]; }

	unsigned num_planes() const noexcept override { return m_num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept override { return m_parents[plane].first->format(m_parents[plane].second); }

	void apply_node_fusion() noexcept override
	{
		for (unsigned p = 0; p < m_num_planes; ++p) {
			assert(!m_parents[p].first->sourcesink());
			m_parents[p].first->set_cache_location(m_parents[p].second, FrameState::cache_descriptor_offset(id(), p));

			for (unsigned q = 0; q < p; ++q) {
				assert(m_parents[p].first->cache_location(m_parents[p].second) != m_parents[q].first->cache_location(m_parents[q].second));
			}
		}
		for (unsigned p = 0; p < m_num_planes; ++p) {
			m_parents[p].first->apply_node_fusion();
		}
	}

	bool reachable(node_id id, unsigned plane) const noexcept override
	{
		for (unsigned p = 0; p < m_num_planes; ++p) {
			if (m_parents[p].first->id() == id && m_parents[p].second == plane)
				return true;
			if (m_parents[p].first->reachable(id, plane))
				return true;
		}
		return false;
	}

	void trace_working_memory(Simulation *sim) const noexcept override
	{
		for (unsigned p = 0; p < m_num_planes; ++p) {
			m_parents[p].first->trace_working_memory(sim);
		}
	}

	void trace_access_pattern(Simulation *sim, unsigned first_row, unsigned last_row, unsigned plane) const noexcept override
	{
		// Normalize row addresses.
		first_row <<= m_subsample_h[plane];
		first_row = first_row - first_row % m_step;
		last_row <<= m_subsample_h[plane];

		// Walk through parents.
		unsigned cursor = sim->cursor(id(), first_row);
		cursor = sim->is_live(id(), id(), first_row) ? cursor : first_row;

		for (; cursor < last_row; cursor += m_step) {
			for (unsigned p = 0; p < m_num_planes; ++p) {
				m_parents[p].first->trace_access_pattern(sim, cursor >> m_subsample_h[p], (cursor + m_step) >> m_subsample_h[p], m_parents[p].second);
			}
		}
		sim->update_cursor_range(id(), first_row, cursor);
		sim->update_live_range(id(), id(), first_row, cursor);
	}

	void begin_frame(FrameState *state, unsigned left, unsigned right, unsigned plane) const noexcept override
	{
		if (state->initialized(id()))
			return;

		left <<= m_subsample_w[plane];
		right <<= m_subsample_w[plane];
		state->update_col_bounds(id(), left, right);

		for (unsigned p = 0; p < m_num_planes; ++p) {
			m_parents[p].first->begin_frame(state, left >> m_subsample_w[p], right >> m_subsample_w[p], m_parents[p].second);
		}
		state->set_initialized(id());
	}

	void process(FrameState *state, unsigned last_row, unsigned plane) const override
	{
		Graph::Callback callback = state->has_callback(id()) ? state->callback(id()) : nullptr;
		std::pair<unsigned, unsigned> callback_bounds = state->col_bounds(id());
		unsigned cursor = state->cursor(id());

		last_row <<= m_subsample_h[plane];

		for (; cursor < last_row; cursor += m_step) {
			switch (m_num_planes) {
			case 4:
				m_parents[0].first->process(state, (cursor + m_step) >> m_subsample_h[0], 0);
				m_parents[1].first->process(state, (cursor + m_step) >> m_subsample_h[1], 1);
				m_parents[2].first->process(state, (cursor + m_step) >> m_subsample_h[2], 2);
				m_parents[3].first->process(state, (cursor + m_step) >> m_subsample_h[3], 3);
				break;
			case 3:
				m_parents[0].first->process(state, (cursor + m_step) >> m_subsample_h[0], 0);
				m_parents[1].first->process(state, (cursor + m_step) >> m_subsample_h[1], 1);
				m_parents[2].first->process(state, (cursor + m_step) >> m_subsample_h[2], 2);
				break;
			case 2:
				m_parents[0].first->process(state, (cursor + m_step) >> m_subsample_h[0], 0);
				m_parents[1].first->process(state, (cursor + m_step) >> m_subsample_h[1], 1);
				break;
			case 1:
				m_parents[0].first->process(state, (cursor + m_step) >> m_subsample_h[0], 0);
				break;
			default:
				break;
			}

			if (callback) {
				invoke_callback(callback, cursor, callback_bounds.first, callback_bounds.second);
				state->check_guard_pages();
			}
		}

		state->set_cursor(id(), cursor);
	}
};


class TransformNodeBase : public Node {
protected:
	const Filter *m_filter;
	const FilterDescriptor *m_filter_desc;
	node_dep m_parents[FILTER_MAX_DEPS] = {};
	unsigned m_ref_count[FILTER_MAX_PLANES] = {};
public:
	TransformNodeBase(node_id id, const Filter *filter, const node_dep deps[]) :
		Node{ id },
		m_filter{ filter },
		m_filter_desc{ &filter->descriptor() }
	{
		assert(m_filter_desc->num_planes);
		assert(m_filter_desc->num_deps <= FILTER_MAX_DEPS);
		assert(m_filter_desc->num_planes <= FILTER_MAX_DEPS);
		std::copy_n(deps, m_filter_desc->num_deps, m_parents);
	}

	unsigned ref_count(unsigned plane) const noexcept override { return m_ref_count[plane]; }

	void add_ref(unsigned plane) noexcept override { assert(plane < m_filter_desc->num_planes); ++m_ref_count[plane]; }

	void dec_ref(unsigned plane) noexcept override { assert(m_ref_count[plane]); --m_ref_count[plane]; }

	bool sourcesink() const noexcept override { return false; }

	unsigned subsample_w(unsigned) const noexcept override { return 0; }

	unsigned subsample_h(unsigned) const noexcept override { return 0; }

	unsigned num_planes() const noexcept override { return m_filter_desc->num_planes; }

	PlaneDescriptor format(unsigned) const noexcept override { return m_filter_desc->format; }

	void apply_node_fusion() noexcept override
	{
		if (m_filter_desc->flags.in_place) {
			bool parent_visited[FILTER_MAX_DEPS] = {};
			bool plane_used[FILTER_MAX_PLANES] = {};

			size_t self_rowsize = static_cast<size_t>(m_filter_desc->format.width) * m_filter_desc->format.bytes_per_sample;
			unsigned self_height = m_filter_desc->format.height;

			auto try_parent = [&](unsigned parent_num)
			{
				if (parent_visited[parent_num])
					return;
				if (m_filter_desc->inplace_hint.enabled && (m_filter_desc->inplace_hint.disallow_mask & (1U << parent_num)))
					return;

				parent_visited[parent_num] = true;

				node_dep dep = m_parents[parent_num];
				if (dep.first->sourcesink() || dep.first->ref_count(dep.second) > 1)
					return;

				// Already fused.
				if (dep.first->cache_location(dep.second) != FrameState::cache_descriptor_offset(dep.first->id(), dep.second))
					return;

				// Buffer size mismatch.
				PlaneDescriptor dep_format = dep.first->format(dep.second);
				size_t dep_rowsize = static_cast<size_t>(dep_format.width) * dep_format.bytes_per_sample;
				if (dep_rowsize != self_rowsize || dep_format.height != self_height)
					return;

				// Search for an unfused output plane.
				auto try_plane = [&](unsigned plane_num)
				{
					if (plane_used[plane_num])
						return false;

					dep.first->set_cache_location(dep.second, cache_location(plane_num));
					plane_used[plane_num] = true;
					return true;
				};

				// Attempt 1:1 mapping of input to output plane.
				if (m_filter_desc->num_planes >= m_filter_desc->num_deps - parent_num) {
					if (try_plane(m_filter_desc->num_planes - (m_filter_desc->num_deps - parent_num)))
						return;
				}

				for (unsigned plane = m_filter_desc->num_planes; plane != 0; --plane) {
					if (try_plane(plane - 1))
						return;
				}
			};

			if (m_filter_desc->inplace_hint.enabled) {
				try_parent(m_filter_desc->inplace_hint.preferred_index);
			}

			// Scan backwards to optimize for LRU.
			for (unsigned parent = m_filter_desc->num_deps; parent != 0; --parent) {
				try_parent(parent - 1);
			}
		}

		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			m_parents[p].first->apply_node_fusion();
		}
	}

	bool reachable(node_id id, unsigned plane) const noexcept override
	{
		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			if (m_parents[p].first->id() == id && m_parents[p].second == plane)
				return true;
			if (m_parents[p].first->reachable(id, plane))
				return true;
		}
		return false;
	}

	void trace_working_memory(Simulation *sim) const noexcept override
	{
		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			m_parents[p].first->trace_working_memory(sim);
		}
		sim->update_working_memory(id(), m_filter_desc->context_size, m_filter_desc->scratchpad_size);
	}

	void trace_access_pattern(Simulation *sim, unsigned first_row, unsigned last_row, unsigned) const noexcept override
	{
		unsigned uninitialized_first_row = first_row;

		if (m_filter_desc->flags.stateful || m_filter_desc->flags.entire_col)
			uninitialized_first_row = 0;
		if (m_filter_desc->flags.entire_row || m_filter_desc->flags.entire_col)
			sim->set_no_tiling();

		unsigned cursor = sim->cursor(id(), uninitialized_first_row);
		for (unsigned p = 0; p < m_filter_desc->num_planes; ++p) {
			if (!sim->is_live(id(), FrameState::cache_descriptor_offset_to_node(cache_location(p)), first_row) &&
				first_row < sim->cursor_min(id()))
			{
				cursor = first_row;
				break;
			}
		}

		for (; cursor < last_row; cursor += std::min(m_filter_desc->format.height - cursor, m_filter_desc->step)) {
			auto range = m_filter->get_row_deps(cursor);

			for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
				m_parents[p].first->trace_access_pattern(sim, range.first, range.second, m_parents[p].second);
			}
		}

		sim->update_cursor_range(id(), first_row, cursor);
		for (unsigned p = 0; p < m_filter_desc->num_planes; ++p) {
			sim->update_live_range(id(), FrameState::cache_descriptor_offset_to_node(cache_location(p)), first_row, cursor);
		}
	}

	void begin_frame(FrameState *state, unsigned left, unsigned right, unsigned) const noexcept override
	{
		if (m_filter_desc->flags.entire_row) {
			left = 0;
			right = m_filter_desc->format.width;
		}

		left = left & ~m_filter_desc->alignment_mask;
		right = std::min((right + m_filter_desc->alignment_mask) & ~m_filter_desc->alignment_mask, m_filter_desc->format.width);

		if (!state->update_col_bounds(id(), left, right))
			return;

		Filter::pair_unsigned col_deps = m_filter->get_col_deps(left, right);
		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			m_parents[p].first->begin_frame(state, col_deps.first, col_deps.second, m_parents[p].second);
		}

		if (!state->initialized(id()))
			m_filter->init_context(state->context(id()));
		state->set_initialized(id());
	}
};

class TransformNode final : public TransformNodeBase {
public:
	using TransformNodeBase::TransformNodeBase;

	void process(FrameState *state, unsigned last_row, unsigned) const override
	{
		unsigned cursor = state->cursor(id());
		if (cursor >= last_row)
			return;

		// Gather dependencies.
		BufferDescriptor inputs[FILTER_MAX_DEPS];
		BufferDescriptor outputs[FILTER_MAX_PLANES];

		switch (m_filter_desc->num_deps) {
		case 3: inputs[2] = state->buffer(m_parents[2].first->cache_location(m_parents[2].second)); // fallthrough
		case 2: inputs[1] = state->buffer(m_parents[1].first->cache_location(m_parents[1].second)); // fallthrough
		case 1: inputs[0] = state->buffer(m_parents[0].first->cache_location(m_parents[0].second)); // fallthrough
		}

		switch (m_filter_desc->num_planes) {
		case 3: outputs[2] = state->buffer(cache_location(2)); // fallthrough
		case 2: outputs[1] = state->buffer(cache_location(1)); // fallthrough
		case 1: outputs[0] = state->buffer(cache_location(0)); // fallthrough
		}

		for (; cursor < last_row; cursor += m_filter_desc->step) {
			Filter::pair_unsigned parent_range = m_filter->get_row_deps(cursor);

			// Invoke parents.
			switch (m_filter_desc->num_deps) {
			case 3:
				m_parents[0].first->process(state, parent_range.second, m_parents[0].second);
				m_parents[1].first->process(state, parent_range.second, m_parents[1].second);
				m_parents[2].first->process(state, parent_range.second, m_parents[2].second);
				break;
			case 2:
				m_parents[0].first->process(state, parent_range.second, m_parents[0].second);
				m_parents[1].first->process(state, parent_range.second, m_parents[1].second);
				break;
			case 1:
				m_parents[0].first->process(state, parent_range.second, m_parents[0].second);
				break;
			default:
				break;
			}

			// Invoke filter.
			std::pair<unsigned, unsigned> col_bounds = state->col_bounds(id());
			m_filter->process(inputs, outputs, cursor, col_bounds.first, col_bounds.second, state->context(id()), state->scratchpad());
			state->check_guard_pages();
		}

		state->set_cursor(id(), cursor);
	}
};

class TransformNodeOneInput final : public TransformNodeBase {
public:
	TransformNodeOneInput(node_id id, const Filter *filter, const node_dep deps[]) : TransformNodeBase(id, filter, deps)
	{
		assert(m_filter_desc->num_deps == 1);
	}

	void process(FrameState *state, unsigned last_row, unsigned) const override
	{
		unsigned cursor = state->cursor(id());
		if (cursor >= last_row)
			return;

		// Gather dependencies.
		BufferDescriptor *input = &state->buffer(m_parents[0].first->cache_location(m_parents[0].second));
		BufferDescriptor outputs[FILTER_MAX_PLANES];

		switch (m_filter_desc->num_planes) {
		case 3: outputs[2] = state->buffer(cache_location(2)); // fallthrough
		case 2: outputs[1] = state->buffer(cache_location(1)); // fallthrough
		case 1: outputs[0] = state->buffer(cache_location(0)); // fallthrough
		}

		for (; cursor < last_row; cursor += m_filter_desc->step) {
			Filter::pair_unsigned parent_range = m_filter->get_row_deps(cursor);

			// Invoke parent.
			m_parents[0].first->process(state, parent_range.second, m_parents[0].second);

			// Invoke filter.
			std::pair<unsigned, unsigned> col_bounds = state->col_bounds(id());
			m_filter->process(input, outputs, cursor, col_bounds.first, col_bounds.second, state->context(id()), state->scratchpad());
			state->check_guard_pages();
		}

		state->set_cursor(id(), cursor);
	}
};

class TransformNodeOneOutput final : public TransformNodeBase {
public:
	TransformNodeOneOutput(node_id id, const Filter *filter, const node_dep deps[]) : TransformNodeBase(id, filter, deps)
	{
		assert(m_filter_desc->num_planes == 1);
	}

	void process(FrameState *state, unsigned last_row, unsigned) const override
	{
		unsigned cursor = state->cursor(id());
		if (cursor >= last_row)
			return;

		// Gather dependencies.
		BufferDescriptor inputs[FILTER_MAX_DEPS];
		BufferDescriptor *output = &state->buffer(cache_location(0));

		switch (m_filter_desc->num_deps) {
		case 3: inputs[2] = state->buffer(m_parents[2].first->cache_location(m_parents[2].second)); // fallthrough
		case 2: inputs[1] = state->buffer(m_parents[1].first->cache_location(m_parents[1].second)); // fallthrough
		case 1: inputs[0] = state->buffer(m_parents[0].first->cache_location(m_parents[0].second)); // fallthrough
		}

		for (; cursor < last_row; cursor += m_filter_desc->step) {
			Filter::pair_unsigned parent_range = m_filter->get_row_deps(cursor);

			// Invoke parents.
			switch (m_filter_desc->num_deps) {
			case 3:
				m_parents[0].first->process(state, parent_range.second, m_parents[0].second);
				m_parents[1].first->process(state, parent_range.second, m_parents[1].second);
				m_parents[2].first->process(state, parent_range.second, m_parents[2].second);
				break;
			case 2:
				m_parents[0].first->process(state, parent_range.second, m_parents[0].second);
				m_parents[1].first->process(state, parent_range.second, m_parents[1].second);
				break;
			case 1:
				m_parents[0].first->process(state, parent_range.second, m_parents[0].second);
				break;
			default:
				break;
			}

			// Invoke filter.
			std::pair<unsigned, unsigned> col_bounds = state->col_bounds(id());
			m_filter->process(inputs, output, cursor, col_bounds.first, col_bounds.second, state->context(id()), state->scratchpad());
			state->check_guard_pages();
		}

		state->set_cursor(id(), cursor);
	}
};

class TransformNodeSimple final : public TransformNodeBase {
public:
	TransformNodeSimple(node_id id, const Filter *filter, const node_dep deps[]) : TransformNodeBase(id, filter, deps)
	{
		assert(m_filter_desc->num_deps == 1);
		assert(m_filter_desc->num_planes == 1);
	}

	void process(FrameState *state, unsigned last_row, unsigned) const override
	{
		unsigned cursor = state->cursor(id());
		if (cursor >= last_row)
			return;

		// Gather dependencies.
		BufferDescriptor *input = &state->buffer(m_parents[0].first->cache_location(m_parents[0].second));
		BufferDescriptor *output = &state->buffer(cache_location(0));

		for (; cursor < last_row; cursor += m_filter_desc->step) {
			Filter::pair_unsigned parent_range = m_filter->get_row_deps(cursor);

			// Invoke parent.
			m_parents[0].first->process(state, parent_range.second, m_parents[0].second);

			// Invoke filter.
			std::pair<unsigned, unsigned> col_bounds = state->col_bounds(id());
			m_filter->process(input, output, cursor, col_bounds.first, col_bounds.second, state->context(id()), state->scratchpad());
			state->check_guard_pages();
		}

		state->set_cursor(id(), cursor);
	}
};

} // namespace


Node::Node(node_id id) : m_id{ id }, m_cache_location{}
{
	for (unsigned p = 0; p < NODE_MAX_PLANES; ++p) {
		m_cache_location[p] = FrameState::cache_descriptor_offset(id, p);
	}
}


std::unique_ptr<Node> make_source_node(node_id id, unsigned num_planes, const PlaneDescriptor desc[])
{
	return std::make_unique<SourceNode>(id, num_planes, desc);
}

std::unique_ptr<Node> make_sink_node(node_id id, unsigned num_planes, const std::pair<Node *, unsigned> parents[])
{
	return std::make_unique<SinkNode>(id, num_planes, parents);
}

std::unique_ptr<Node> make_transform_node(node_id id, const Filter *filter, const node_dep deps[])
{
	const auto &desc = filter->descriptor();
	if (desc.num_deps == 1 && desc.num_planes == 1)
		return std::make_unique<TransformNodeSimple>(id, filter, deps);
	if (desc.num_deps == 1)
		return std::make_unique<TransformNodeOneInput>(id, filter, deps);
	if (desc.num_planes == 1)
		return std::make_unique<TransformNodeOneOutput>(id, filter, deps);

	return std::make_unique<TransformNode>(id, filter, deps);
}

} // namespace impl
} // namespace graphengine
