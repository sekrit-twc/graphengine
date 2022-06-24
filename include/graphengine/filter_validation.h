#ifndef GRAPHENGINE_FILTER_VALIDATION_VALIDATE_H_
#define GRAPHENGINE_FILTER_VALIDATION_VALIDATE_H_

#include "graphengine/types.h"

namespace graphengine {

class Filter;
struct PlaneDescriptor;

/**
 * GTest-based validation driver for filter implementations.
 *
 * Validation tool should be embedded into the user test project. No API or ABI
 * stability is provided. All functions may throw std::exception.
 */
class FilterValidation {
public:
	/** Pixel format. */
	struct PixelDescriptor {
		unsigned bits_per_sample; /**< Number of non-padding bits. */
		bool floating_point;      /**< Floating-point samples, must be 16 or 32-bit. */
		bool chroma;              /**< Samples with range [-0.5, 0.5] for floats. */
	};

private:
	class impl;

	detail::unique_ptr<impl> m_impl;
public:
	/**
	 * Initialize test with no input planes.
	 *
	 * @param[in] filter filter
	 */
	explicit FilterValidation(const Filter *filter);

	/**
	 * Initialize test.
	 *
	 * @param[in] filter filter
	 * @param[in] dep_format input dimensions
	 * @param[in] bytes_per_sample override sample size per-plane
	 */
	FilterValidation(const Filter *filter, const PlaneDescriptor &dep_format, const unsigned bytes_per_sample[] = nullptr);

	/** Move constructor. */
	FilterValidation(FilterValidation &&other) noexcept;

	/** Destructor. */
	~FilterValidation();

	/** Move assignment. */
	FilterValidation &operator=(FilterValidation &&other) noexcept;

	/**
	 * Set reference implementation of filter.
	 *
	 * If provided, the filter output will be compared to the reference output.
	 * {@p snr_thresh} can be provided for fuzzy matches. Otherwise, the test
	 * will check for bit-exactness.
	 *
	 * @param[in] ref_filter reference
	 * @param snr_thresh SNR tolerance
	 */
	FilterValidation &set_reference_filter(const Filter *ref_filter, double snr_thresh = 1.0 / 0.0);

	/**
	 * Set input pixel format for planes.
	 *
	 * The filter is exercised against pseudo-random input. If not set, the
	 * input format will be deduced from the input plane descriptor as an
	 * integer format with no padding bits.
	 *
	 * @param pixel_desc format
	 */
	FilterValidation &set_input_pixel_format(const PixelDescriptor &pixel_desc);

	/** Set input pixel format for a specific plane. */
	FilterValidation &set_input_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc);

	/**
	 * Set output pixel format.
	 *
	 * SNR calculations require the output format to be set correctly. If not
	 * set, the output format will be deduced from the filter descriptor.
	 *
	 * @param pixel_desc format
	 */
	FilterValidation &set_output_pixel_format(const PixelDescriptor &pixel_desc);

	/** Set output pixel format for a specific plane. */
	FilterValidation &set_output_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc);

	/**
	 * Set SHA1 hashsum.
	 *
	 * If set, the filter output will be compared against the checksum.
	 *
	 * @param plane planme
	 * @param[in] sha1 hash
	 */
	FilterValidation &set_sha1(unsigned plane, const char sha1[40]);

	/**
	 * Run the test.
	 *
	 * @return true on success, false on failure
	 */
	bool run();
};

} // namespace graphengine

#endif // GRAPHENGINE_FILTER_VALIDATION_VALIDATE_H_
