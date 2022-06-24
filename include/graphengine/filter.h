#ifndef GRAPHENGINE_FILTER_H_
#define GRAPHENGINE_FILTER_H_

#include <cstddef>
#include "graphengine/types.h"

struct graphengine_filter {
protected:
	~graphengine_filter() = default;
};

namespace graphengine {

/**
 * Filter descriptor.
 * Contains metadata describing how the filter is to be invoked.
 */
struct FilterDescriptor {
	PlaneDescriptor format;  /**< Output image format. */
	unsigned num_deps;       /**< Number of input planes. */
	unsigned num_planes;     /**< Number of output planes. */
	unsigned step;           /**< Rows processed per call, or height if {@p flags.entire_col}. */
	unsigned alignment_mask; /**< Alignment constraint on left/right span in pixels, e.g. 0x3 for mod4. */

	/** Flags bitfield. */
	struct {
		unsigned char stateful : 1;   /**< Processing starts from row 0 and must not skip any rows. */
		unsigned char in_place : 1;   /**< Input and output buffers may alias. */
		unsigned char entire_row : 1; /**< Processes an entire row in each call. */
		unsigned char entire_col : 1; /**< Processes all rows in one call. */
		unsigned short : 0;
	} flags;

	/** Detailed control for in-place processing. */
	struct {
		unsigned char enabled : 1;         /**< Subsequent fields are ignored if unset. */
		unsigned char preferred_index : 2; /**< Preferred input plane to overwrite. */
		unsigned char disallow_mask : 3;   /**< Bitmask of input planes that may not be overwritten. */
		unsigned short : 0;
	} inplace_hint;

	size_t context_size;    /**< Memory required for state carried across calls. */
	size_t scratchpad_size; /**< Temporary memory not retained across calls. */

	static_assert(sizeof(flags) == sizeof(unsigned short), "alignment error");
	static_assert(sizeof(inplace_hint) == sizeof(unsigned short), "alignment error");
};

/**
 * Interface class for filters.
 *
 * Filters apply transformations to images, defined as a collection of 1-3
 * planes of the same dimensions. Execution of a filter chain or graph is
 * buffered, with a handful of rows processed on each call.
 */
class Filter : public graphengine_filter {
public:
	/** Version number to be reported by {@p version()}. */
	static constexpr int VERSION = 0;

	typedef detail::pair<unsigned, unsigned> pair_unsigned;
protected:
	Filter() = default;
public:
	Filter(const Filter &) = delete;

	virtual ~Filter() = default;

	Filter &operator=(const Filter &) = delete;

	/**
	 * Report version number.
	 *
	 * @return {@p VERSION}
	 */
	virtual int version() const noexcept = 0;

	/**
	 * Return reference to descriptor.
	 *
	 * @return descriptor
	 * @see FilterDescriptor
	 */
	virtual const FilterDescriptor &descriptor() const noexcept = 0;

	/**
	 * Query the input rows required for a given output row.
	 *
	 * @param i row to be processed
	 * @return input range for [i, i + step)
	 */
	virtual pair_unsigned get_row_deps(unsigned i) const noexcept = 0;

	/**
	 * Query the input span required for a given output span.
	 *
	 * The output span queried will respect {@p desc.alignment_mask}, i.e.
	 *   (left & desc.alignment_mask) == 0
	 *   (right & desc.alignment_mask) == 0 || right == width
	 *
	 * @param left,right span to be processed
	 * @return input span for [left, right)
	 */
	virtual pair_unsigned get_col_deps(unsigned left, unsigned right) const noexcept = 0;

	/**
	 * Initialize internal state.
	 *
	 * Called before processing the first row. {@p context} will be passed to
	 * all subsequent calls to {@p process()}.
	 *
	 * @param context aligned to 64 bytes
	 */
	virtual void init_context(void *context) const noexcept = 0;

	/**
	 * Process rows.
	 *
	 * Apply the filter to row [i, i + step) wtih span [left, right). The span
	 * respects {@p desc.alignment_mask}. Filters must not write outside the
	 * requested range, unless that write is to the 64-byte alignment padding
	 * at the end of the row. For example, if the row is 637 bytes long, the
	 * filter may write to indices 637-639, but if the row is 640 bytes long,
	 * index 640 may not be written.
	 *
	 * The input and output buffers may alias if {@p desc.flags.inplace} is set.
	 * If a special code path is required for this case, it must be detected by
	 * comparing the buffer pointers, not the addresses of the descriptors.
	 *
	 * @param in,out array of buffer descriptors, one per input/output plane
	 * @param i row to be processed
	 * @param left,right span to be processed
	 * @param context filter state, aligned to 64 bytes
	 * @param tmp scratchpad memory, aligned to 64 bytes
	 */
	virtual void process(const BufferDescriptor in[], const BufferDescriptor out[],
	                     unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept = 0;
};

} // namespace graphengine

#endif // GRAPHENGINE_FILTER_H_
