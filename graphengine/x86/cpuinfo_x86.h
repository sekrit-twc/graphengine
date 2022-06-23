#pragma once

#ifndef GRAPHENGINE_X86_CPUINFO_X86_H_
#define GRAPHENGINE_X86_CPUINFO_X86_H_

#if defined(__i386) || defined(_M_IX86) || defined(_M_X64) || defined(__x86_64__)

#include "graphengine/namespace.h"

namespace graphengine {
namespace GRAPHENGINE_IMPL_NAMESPACE {

unsigned long cpu_cache_per_thread_x86() noexcept;

} // namespace impl
} // namespace graphengine

#endif // defined(__i386) || defined(_M_IX86) || defined(_M_X64) || defined(__x86_64__)

#endif // GRAPHENGINE_X86_CPUINFO_X86_H_
