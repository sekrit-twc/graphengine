#ifndef GRAPHENGINE_FILTER_VALIDATION_VALIDATE_H_
#define GRAPHENGINE_FILTER_VALIDATION_VALIDATE_H_

#include <memory>

namespace graphengine {

class Filter;
struct PlaneDescriptor;

class FilterValidation {
public:
	struct PixelDescriptor {
		unsigned bits_per_sample;
		bool floating_point;
		bool chroma;
	};

private:
	class impl;

	std::unique_ptr<impl> m_impl;
public:
	explicit FilterValidation(const Filter *filter);
	FilterValidation(const Filter *filter, const PlaneDescriptor &dep_format);
	FilterValidation(const Filter *filter, const PlaneDescriptor dep_format[3]);

	~FilterValidation();

	FilterValidation &set_reference_filter(const Filter *ref_filter);
	FilterValidation &set_reference_filter(const Filter *ref_filter, double snr_thresh);

	FilterValidation &set_input_pixel_format(const PixelDescriptor &pixel_desc);
	FilterValidation &set_input_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc);

	FilterValidation &set_output_pixel_format(const PixelDescriptor &pixel_desc);
	FilterValidation &set_output_pixel_format(unsigned plane, const PixelDescriptor &pixel_desc);

	FilterValidation &set_sha1(unsigned plane, const char sha1[40]);

	bool run();
};

} // namespace graphengine

#endif // GRAPHENGINE_FILTER_VALIDATION_VALIDATE_H_
