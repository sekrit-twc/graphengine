#include "cpuinfo.h"

#if defined(__i386) || defined(_M_IX86) || defined(_M_X64) || defined(__x86_64__)
  #include "x86/cpuinfo_x86.h"
#endif

namespace graphengine {
namespace GRAPHENGINE_IMPL_NAMESPACE {

unsigned long cpu_cache_per_thread()
{
	unsigned long cache = 0;

#if defined(__i386) || defined(_M_IX86) || defined(_M_X64) || defined(__x86_64__)
	cache = cpu_cache_per_thread_x86();
#endif
	return cache ? cache : 1UL << 20;
}

} // namespace impl
} // namespace graphengine
