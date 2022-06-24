#pragma once

#ifndef GRAPHENGINE_UNIT_TEST_VALIDATOR_H_
#define GRAPHENGINE_UNIT_TEST_VALIDATOR_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "graphengine/filter.h"

class ValidationFilter : public graphengine::Filter {
public:
	class ValidationState;

	std::string name;
	graphengine::FilterDescriptor desc = {};
	ValidationFilter *parents[graphengine::FILTER_MAX_DEPS] = {};
	ValidationState *state = nullptr;

	explicit ValidationFilter(const char *name) : name{ name } {}

	int version() const noexcept override { return VERSION; }

	const graphengine::FilterDescriptor &descriptor() const noexcept override final { return desc; }

	void init_context(void *context) const noexcept override final;

	void process(const graphengine::BufferDescriptor in[], const graphengine::BufferDescriptor out[],
	             unsigned i, unsigned left, unsigned right, void *context, void *tmp) const noexcept override final;
};


struct ScriptStatement {
	struct ParentDefinition {
		std::string name;
		unsigned plane;

		ParentDefinition(const std::string &name, unsigned plane = 0) : name{ name }, plane{ plane } {}
	};

	std::string result;
	std::string filter;
	std::vector<ParentDefinition> parents;
	std::vector<unsigned> args;
};

// Special source and sink nodes.
constexpr char SOURCE[] = "Source"; // arg0 = width, arg1 = height, arg2 = bytes_per_sample(1), arg3 = num_planes(1), arg4 = subsample_w(0), arg4 = subsample_h(0)
constexpr char SINK[] = "Sink";

// Built-in access patterns.
constexpr char GENERATE_FILTER[] = "Generate"; // arg0 = width, arg1 = height, arg2 = bytes_per_sample(1)
constexpr char POINT_FILTER[] = "Point"; // arg0 = bytes_per_sample
constexpr char CONVOLUTION_FILTER[] = "Convolution"; // arg0 = N(3)
constexpr char COLORSPACE_FILTER[] = "Colorspace";
constexpr char MERGE_FILTER[] = "Merge";
constexpr char SPLIT2_FILTER[] = "Split2";
constexpr char SCALEV_FILTER[] = "ScaleV"; // arg0 = N
constexpr char SCALEH_FILTER[] = "ScaleH"; // arg0 = N
constexpr char RECURSIVE_FILTER[] = "WholeLine"; // arg0 = width(0)
constexpr char FRAME_FILTER[] = "WholePlane"; // arg0 = width, arg1 = height, arg2 = bytes_per_sample(1)


class GraphValidator {
	class Script;

	typedef std::function<
		std::unique_ptr<ValidationFilter>(
			const char *name,
			const graphengine::PlaneDescriptor &source_format,
			const std::vector<unsigned> &args)
	> filter_factory;

	std::unordered_map<std::string, filter_factory> m_factories;
public:
	GraphValidator();

	void register_factory(const char *filter_name, filter_factory func);

	void validate(const ScriptStatement *statements, size_t num_statements);
};

#endif // GRAPHENGINE_UNIT_TEST_VALIDATOR_H_
