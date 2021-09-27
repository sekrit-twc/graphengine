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

	bool sourcesink() const noexcept override { return true; }

	virtual unsigned subsample_w(unsigned plane) const noexcept { return m_subsample_w[plane]; }

	virtual unsigned subsample_h(unsigned plane) const noexcept { return m_subsample_h[plane]; }

	unsigned num_planes() const noexcept override { return m_num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept { return m_desc[plane]; }

	void trace_working_memory(Simulation *sim) const noexcept override {}

	void trace_access_pattern(Simulation *sim, unsigned first_row, unsigned last_row, unsigned plane) const noexcept override
	{
		// Normalize row addresses.
		first_row <<= m_subsample_h[plane];
		first_row = first_row - first_row % m_step;

		last_row <<= m_subsample_h[plane];
		last_row = last_row % m_step ? last_row + (m_step - last_row % m_step) : last_row;

		sim->update_live_range(id(), first_row, last_row);
	}

	void begin_frame(FrameState *state, unsigned plane) const noexcept override {}

	void process(FrameState *state, unsigned last_row, unsigned plane) const override
	{
		if (!state->callback(id()))
			return;

		unsigned cursor = state->cursor(id());
		last_row = last_row << m_subsample_h[plane];
		if (cursor >= last_row)
			return;

		for (; cursor < last_row; cursor += m_step) {
			state->callback(id())(cursor, 0, m_desc[0].width);
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

	bool sourcesink() const noexcept override { return true; }

	virtual unsigned subsample_w(unsigned plane) const noexcept { return m_subsample_w[plane]; }

	virtual unsigned subsample_h(unsigned plane) const noexcept { return m_subsample_h[plane]; }

	unsigned num_planes() const noexcept override { return m_num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept { return m_parents[plane].first->format(m_parents[plane].second); }

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
		cursor = sim->is_live(id(), first_row) ? cursor : first_row;

		for (; cursor < last_row; cursor += m_step) {
			for (unsigned p = 0; p < m_num_planes; ++p) {
				m_parents[p].first->trace_access_pattern(sim, cursor >> m_subsample_h[p], (cursor + m_step) >> m_subsample_h[p], m_parents[p].second);
			}
		}
		sim->update_live_range(id(), first_row, cursor);
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

				BufferDescriptor in = state->buffer(m_parents[p].first->id(), m_parents[p].second);
				BufferDescriptor out = state->buffer(id(), p);
				size_t rowsize = static_cast<size_t>(format(p).width) * format(p).bytes_per_sample;

				for (unsigned i = cursor >> m_subsample_h[p]; i < (cursor + m_step) >> m_subsample_h[p]; ++i) {
					std::copy_n(static_cast<const uint8_t *>(in.get_line(i)), rowsize, static_cast<uint8_t *>(out.get_line(i)));
				}
			}

			if (state->callback(id()))
				state->callback(id())(cursor, 0, format(0).width);
		}

		state->set_cursor(id(), cursor);
	}
};


class TransformNode : public Node {
	const Filter *m_filter;
	const FilterDescriptor *m_filter_desc;
	node_dep m_parents[FILTER_MAX_DEPS] = {};
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

	bool sourcesink() const noexcept override { return false; }

	virtual unsigned subsample_w(unsigned) const noexcept { return 0; }

	virtual unsigned subsample_h(unsigned) const noexcept { return 0; }

	unsigned num_planes() const noexcept override { return m_filter_desc->num_planes; }

	PlaneDescriptor format(unsigned plane) const noexcept { return m_filter_desc->format; }

	void trace_working_memory(Simulation *sim) const noexcept override
	{
		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			m_parents[p].first->trace_working_memory(sim);
		}
		sim->update_working_memory(id(), m_filter_desc->context_size, m_filter_desc->scratchpad_size);
	}

	void trace_access_pattern(Simulation *sim, unsigned first_row, unsigned last_row, unsigned) const noexcept override
	{
		unsigned cursor = sim->cursor(id(), first_row);
		cursor = sim->is_live(id(), first_row) ? cursor : first_row;

		for (; cursor < last_row; cursor += std::min(m_filter_desc->format.height - cursor, m_filter_desc->step)) {
			auto range = m_filter->get_row_deps(cursor);

			for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
				m_parents[p].first->trace_access_pattern(sim, range.first, range.second, m_parents[p].second);
			}
		}
		sim->update_live_range(id(), first_row, cursor);
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
		for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
			inputs[p] = state->buffer(m_parents[p].first->id(), m_parents[p].second);
		}

		for (; cursor < last_row; cursor += m_filter_desc->step) {
			std::pair<unsigned, unsigned> parent_range = m_filter->get_row_deps(cursor);

			// Invoke parents.
			for (unsigned p = 0; p < m_filter_desc->num_deps; ++p) {
				m_parents[p].first->process(state, parent_range.second, m_parents[p].second);
			}

			// Invoke filter.
			m_filter->process(inputs, &state->buffer(id(), 0), cursor, 0, m_filter_desc->format.width, state->context(id()), state->scratchpad());
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
