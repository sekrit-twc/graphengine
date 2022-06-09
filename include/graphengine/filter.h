#pragma once

#ifndef GRAPHENGINE_FILTER_H_
#define GRAPHENGINE_FILTER_H_

#include <cstddef>
#include <utility>
#include "graphengine/types.h"

namespace graphengine {

struct FilterDescriptor {
	PlaneDescriptor format;
	unsigned num_deps;
	unsigned num_planes;
	unsigned step;
	unsigned alignment_mask;
	struct {
		unsigned char enabled : 1;
		unsigned char preferred_index : 2;
		unsigned char disallow_mask : 3;
	} inplace_hint;

	size_t context_size;
	size_t scratchpad_size;

	struct flags {
		unsigned char stateful : 1;
		unsigned char in_place : 1;
		unsigned char entire_row : 1;
		unsigned char entire_col : 1;
	} flags;
};


class Filter {
protected:
	Filter() = default;
public:
	Filter(const Filter &) = delete;

	virtual ~Filter() = default;

	Filter &operator=(const Filter &) = delete;

	virtual const FilterDescriptor &descriptor() const noexcept = 0;

	virtual std::pair<unsigned, unsigned> get_row_deps(unsigned i) const noexcept = 0;

	virtual std::pair<unsigned, unsigned> get_col_deps(unsigned left, unsigned right) const noexcept = 0;

	virtual void init_context(void *context) const noexcept {}

	virtual void process(const BufferDescriptor in[], const BufferDescriptor out[],
	                     unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept = 0;
};

} // namespace graphengine

#endif // GRAPHENGINE_FILTER_H_
