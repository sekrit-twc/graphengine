#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include "filter.h"
#include "node.h"
#include "state.h"

namespace graphengine {

namespace {

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
			throw std::runtime_error{ "only 1, 2, and 4x subsampling supported" };
		if (ratio_h != 1.0 && ratio_h != 2.0 && ratio_h != 4.0)
			throw std::runtime_error{ "only 1, 2, and 4x subsampling supported" };

		subsample_w[p] = std::lrint(std::log2(ratio_w));
		subsample_h[p] = std::lrint(std::log2(ratio_h));
		step = std::max(step, 1U << subsample_h[p]);
	}

	return step;

}

} // namespace


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

	virtual unsigned subsample_w(unsigned plane) const noexcept { return m_subsample_w[plane]; }

	virtual unsigned subsample_h(unsigned plane) const noexcept { return m_subsample_h[plane]; }

	unsigned num_planes() const noexcept override { return m_num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept { return m_desc[plane]; }

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

	void begin_frame(FrameState *state, unsigned plane) const noexcept override {}

	void process(FrameState *state, unsigned last_row, unsigned plane) const override
	{
		Graph::Callback callback = state->callback(id());
		if (!callback)
			return;

		unsigned cursor = state->cursor(id());
		last_row = last_row << m_subsample_h[plane];
		if (cursor >= last_row)
			return;

		for (; cursor < last_row; cursor += m_step) {
			callback(cursor, 0, m_desc[0].width);
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

	virtual unsigned subsample_w(unsigned plane) const noexcept { return m_subsample_w[plane]; }

	virtual unsigned subsample_h(unsigned plane) const noexcept { return m_subsample_h[plane]; }

	unsigned num_planes() const noexcept override { return m_num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept { return m_parents[plane].first->format(m_parents[plane].second); }

	void apply_node_fusion() noexcept override
	{
		for (unsigned p = 0; p < m_num_planes; ++p) {
			assert(m_parents[p].first->ref_count(m_parents[p].second) == 1);
			m_parents[p].first->set_cache_location(m_parents[p].second, id(), p);
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

	void begin_frame(FrameState *state, unsigned plane) const noexcept override
	{
		for (unsigned p = 0; p < m_num_planes; ++p) {
			m_parents[p].first->begin_frame(state, m_parents[p].second);
		}
	}

	void process(FrameState *state, unsigned last_row, unsigned) const override
	{
		unsigned cursor = state->cursor(id());

		for (; cursor < last_row; cursor += m_step) {
			for (unsigned p = 0; p < m_num_planes; ++p) {
				m_parents[p].first->process(state, (cursor + m_step) >> m_subsample_h[p], p);
			}

			if (Graph::Callback callback = state->callback(id()))
				callback(cursor, 0, format(0).width);
		}

		state->set_cursor(id(), cursor);
	}
};


class TransformNode : public Node {
	const Filter *m_filter;
	const FilterDescriptor *m_filter_desc;
	node_dep m_parents[FILTER_MAX_DEPS] = {};
	unsigned m_ref_count[FILTER_MAX_PLANES] = {};
public:
	TransformNode(node_id id, const Filter *filter, const node_dep deps[]) :
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

	virtual unsigned subsample_w(unsigned) const noexcept { return 0; }

	virtual unsigned subsample_h(unsigned) const noexcept { return 0; }

	unsigned num_planes() const noexcept override { return m_filter_desc->num_planes; }

	PlaneDescriptor format(unsigned) const noexcept { return m_filter_desc->format; }

	void apply_node_fusion() noexcept override
	{
		if (m_filter_desc->flags.in_place) {
			bool plane_used[FILTER_MAX_PLANES] = {};

			size_t self_rowsize = static_cast<size_t>(m_filter_desc->format.width) * m_filter_desc->format.bytes_per_sample;
			unsigned self_height = m_filter_desc->format.height;

			// Scan backwards to optimize for LRU.
			for (unsigned parent = m_filter_desc->num_deps; parent != 0; --parent) {
				node_dep dep = m_parents[parent - 1];
				if (dep.first->sourcesink() || dep.first->ref_count(dep.second) > 1)
					continue;

				// Already fused.
				if (dep.first->cache_location(dep.second) != std::make_pair(dep.first->id(), dep.second))
					continue;

				// Buffer size mismatch.
				PlaneDescriptor dep_format = dep.first->format(dep.second);
				size_t dep_rowsize = static_cast<size_t>(dep_format.width) * dep_format.bytes_per_sample;
				if (dep_rowsize != self_rowsize || dep_format.height != self_height)
					continue;

				// Search for an unfused output plane.
				for (unsigned plane = 0; plane < m_filter_desc->num_planes; ++plane) {
					if (!plane_used[plane]) {
						auto location = cache_location(plane);
						dep.first->set_cache_location(dep.second, location.first, location.second);
						plane_used[plane] = true;
						break;
					}
				}
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
		if (m_filter_desc->flags.entire_col)
			first_row = 0;

		unsigned cursor = sim->cursor(id(), first_row);
		cursor = sim->is_live(id(), cache_location(0).first, first_row) ? cursor : first_row;

		for (; cursor < last_row; cursor += std::min(m_filter_desc->format.height - cursor, m_filter_desc->step)) {
			auto range = m_filter->get_row_deps(cursor);

			for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
				m_parents[p].first->trace_access_pattern(sim, range.first, range.second, m_parents[p].second);
			}
		}

		sim->update_cursor_range(id(), first_row, cursor);
		for (unsigned p = 0; p < m_filter_desc->num_planes; ++p) {
			sim->update_live_range(id(), cache_location(p).first, first_row, cursor);
		}
	}

	void begin_frame(FrameState *state, unsigned) const noexcept override
	{
		if (state->initialized(id()))
			return;

		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			m_parents[p].first->begin_frame(state, m_parents[p].second);
		}
		m_filter->init_context(state->context(id()));
		state->set_initialized(id());
	}

	void process(FrameState *state, unsigned last_row, unsigned) const override
	{
		unsigned cursor = state->cursor(id());
		if (cursor >= last_row)
			return;

		// Gather dependencies.
		BufferDescriptor inputs[FILTER_MAX_DEPS];
		BufferDescriptor outputs[FILTER_MAX_PLANES];

		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			auto parent_cache = m_parents[p].first->cache_location(m_parents[p].second);
			inputs[p] = state->buffer(parent_cache.first, parent_cache.second);
		}
		for (unsigned p = 0; p < m_filter_desc->num_planes; ++p) {
			auto self_cache = cache_location(p);
			outputs[p] = state->buffer(self_cache.first, self_cache.second);
		}

		for (; cursor < last_row; cursor += m_filter_desc->step) {
			std::pair<unsigned, unsigned> parent_range = m_filter->get_row_deps(cursor);

			// Invoke parents.
			for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
				m_parents[p].first->process(state, parent_range.second, m_parents[p].second);
			}

			// Invoke filter.
			m_filter->process(inputs, outputs, cursor, 0, m_filter_desc->format.width, state->context(id()), state->scratchpad());
		}

		state->set_cursor(id(), cursor);
	}
};


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
	return std::make_unique<TransformNode>(id, filter, deps);
}

} // namespace graphengine
