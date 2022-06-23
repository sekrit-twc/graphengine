#pragma once

#ifndef GRAPHENGINE_CPUINFO_H_
#define GRAPHENGINE_CPUINFO_H_

#include "graphengine/namespace.h"

namespace graphengine {
namespace GRAPHENGINE_IMPL_NAMESPACE {

unsigned long cpu_cache_per_thread();

} // namespace impl
} // namespace graphengine

#endif // GRAPHENGINE_CPUINFO_H_
