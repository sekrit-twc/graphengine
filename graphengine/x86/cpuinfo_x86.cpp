#if defined(__i386) || defined(_M_IX86) || defined(_M_X64) || defined(__x86_64__)

#if 0
  #include <stdio.h>
  #define TRACE(fmt, ...) fprintf(stderr, "[cpuinfo] " fmt, __VA_ARGS__)
#else
  #define TRACE(fmt, ...) do {} while (0)
#endif

#if defined(_MSC_VER)
  #include <intrin.h>
#elif defined(__GNUC__)
  #include <cpuid.h>
#endif

#include "cpuinfo_x86.h"

namespace graphengine {

namespace {

struct X86CacheHierarchy {
	unsigned long l1d;
	unsigned long l1d_threads;
	unsigned long l2;
	unsigned long l2_threads;
	unsigned long l3;
	unsigned long l3_threads;
	bool l2_inclusive;
	bool l3_inclusive;
	bool valid;
};

/**
 * Execute the CPUID instruction.
 *
 * @param regs array to receive eax, ebx, ecx, edx
 * @param eax argument to instruction
 * @param ecx argument to instruction
 */
void do_cpuid(int regs[4], int eax, int ecx)
{
#if defined(_MSC_VER)
	__cpuidex(regs, eax, ecx);
#elif defined(__GNUC__)
	__cpuid_count(eax, ecx, regs[0], regs[1], regs[2], regs[3]);
#else
	regs[0] = 0;
	regs[1] = 0;
	regs[2] = 0;
	regs[3] = 0;
#endif
}

// Query leaf 4h (Intel) or leaf 8000001Dh (AMD).
void do_query_x86_deterministic_cache_parameters(X86CacheHierarchy &cache, int leaf) noexcept
{
	int regs[4];

	for (int i = 0; i < 8; ++i) {
		unsigned threads;
		unsigned long line_size;
		unsigned long partitions;
		unsigned long ways;
		unsigned long sets;
		unsigned long cache_size;
		int cache_type;
		bool inclusive;

		do_cpuid(regs, leaf, i);
		cache_type = regs[0] & 0x1FU;
		TRACE("L%u cache, type %d\n", (static_cast<unsigned>(regs[0]) >> 5) & 0x07U, cache_type);

		// No more caches.
		if (cache_type == 0)
			break;

		// Not data or unified cache.
		if (cache_type != 1 && cache_type != 3)
			continue;

		threads = ((static_cast<unsigned>(regs[0]) >> 14) & 0x0FFFU) + 1;
		line_size = ((static_cast<unsigned>(regs[1]) >> 0) & 0x0FFFU) + 1;
		partitions = ((static_cast<unsigned>(regs[1]) >> 12) & 0x03FFU) + 1;
		ways = ((static_cast<unsigned>(regs[1]) >> 22) & 0x03FFU) + 1;
		sets = static_cast<unsigned>(regs[2]) + 1;

		cache_size = line_size * partitions * ways * sets;
		inclusive = regs[3] & (1U << 1);
		TRACE("%u threads, %lu bytes, %s\n", threads, cache_size, inclusive ? "inclusive" : "non-inclusive");

		// Cache level.
		switch ((static_cast<unsigned>(regs[0]) >> 5) & 0x07U) {
		case 1:
			cache.l1d = cache_size;
			cache.l1d_threads = threads;
			break;
		case 2:
			cache.l2 = cache_size;
			cache.l2_threads = threads;
			cache.l2_inclusive = inclusive;
			break;
		case 3:
			cache.l3 = cache_size;
			cache.l3_threads = threads;
			cache.l3_inclusive = inclusive;
			break;
		default:
			break;
		}
	}
}

X86CacheHierarchy do_query_x86_cache_hierarchy_intel(int max_feature) noexcept
{
	X86CacheHierarchy cache = { 0 };
	int regs[4];

	if (max_feature < 2)
		return cache;

	// Detect cache size of single-threaded CPU from flags.
	if (max_feature >= 2 && max_feature < 4)
		return cache;

	// Detect cache hierarchy.
	if (max_feature >= 4)
		do_query_x86_deterministic_cache_parameters(cache, 4);

	// Detect logical processor count on x2APIC systems.
	if (max_feature >= 0x0B) {
		unsigned l1d_threads = cache.l1d_threads;
		unsigned l2_threads = cache.l2_threads;
		unsigned l3_threads = cache.l3_threads;

		for (int i = 0; i < 8; ++i) {
			unsigned logical_processors;

			do_cpuid(regs, 0x0B, i);
			TRACE("APIC level %u\n", (static_cast<unsigned>(regs[2]) >> 8) & 0xFFU);

			if (((regs[2] >> 8) & 0xFFU) == 0)
				break;

			logical_processors = regs[1] & 0xFFFFU;
			TRACE("logical processors: %u\n", logical_processors);

			l1d_threads = logical_processors <= cache.l1d_threads ? logical_processors : l1d_threads;
			l2_threads = logical_processors <= cache.l2_threads ? logical_processors : l2_threads;
			l3_threads = logical_processors <= cache.l3_threads ? logical_processors : l3_threads;
			TRACE("updated cache sharing: %u %u %u\n", l1d_threads, l2_threads, l3_threads);
		}

		cache.l1d_threads = l1d_threads;
		cache.l2_threads = l2_threads;
		cache.l3_threads = l3_threads;
	}

	return cache;
}

X86CacheHierarchy do_query_x86_cache_hierarchy_amd(int max_feature) noexcept
{
	X86CacheHierarchy cache = { 0 };
	int regs[4];
	unsigned max_extended_feature;

	do_cpuid(regs, 0x80000000U, 0);
	max_extended_feature = static_cast<unsigned>(regs[0]);
	TRACE("max extended feature #: 0x%x\n", max_extended_feature);

	if (max_extended_feature >= 0x80000005U) {
		do_cpuid(regs, 0x80000005U, 0);
		cache.l1d = ((static_cast<unsigned>(regs[2]) >> 24) & 0xFFU) * 1024U;
		cache.l1d_threads = cache.l1d ? 1 : cache.l1d_threads;
		TRACE("L1d: %lu\n", cache.l1d);
	}

	if (max_extended_feature >= 0x80000006U) {
		do_cpuid(regs, 0x80000006U, 0);
		cache.l2 = ((static_cast<unsigned>(regs[2]) >> 16) & 0xFFFFU) * 1024U;
		cache.l3 = ((static_cast<unsigned>(regs[3]) >> 18) & 0x3FFFU) * 512U * 1024U;
		cache.l2_threads = cache.l2 ? 1 : cache.l2_threads;
		cache.l3_threads = cache.l3 ? 1 : cache.l3_threads;
		TRACE("L2: %lu\n", cache.l2);
		TRACE("L3: %lu\n", cache.l3);
	}

	if (max_extended_feature >= 0x80000008U) {
		unsigned threads;
		unsigned family;

		do_cpuid(regs, 0x80000008U, 0);
		threads = (regs[2] & 0xFFU) + 1;
		cache.l3_threads = cache.l3 ? threads : cache.l3_threads;
		TRACE("package threads: %u\n", threads);

		do_cpuid(regs, 1, 0);
		family = ((static_cast<unsigned>(regs[0]) >> 20) & 0xFFU) + ((static_cast<unsigned>(regs[0]) >> 8) & 0x0FU);
		TRACE("family %xh\n", family);

		if (family == 0x15)
			cache.l2_threads = 2; // Bulldozer shared L2 cache.
		else if (family == 0x16)
			cache.l2_threads = threads; // Jaguar L2 LLC.
	}

	if (max_extended_feature >= 0x8000001DU)
		do_query_x86_deterministic_cache_parameters(cache, 0x8000001DU);

	return cache;
}

X86CacheHierarchy do_query_x86_cache_hierarchy() noexcept
{
	enum { GENUINEINTEL, AUTHENTICAMD, OTHER } vendor;

	X86CacheHierarchy cache = { 0 };
	int regs[4] = { 0 };
	int max_feature;

	do_cpuid(regs, 0, 1);
	max_feature = regs[0] & 0xFFU;
	TRACE("max feature #: %d\n", max_feature);

	if (regs[1] == 0x756E6547U && regs[3] == 0x49656E69U && regs[2] == 0x6C65746EU) {
		vendor = GENUINEINTEL;
		TRACE("%s\n", "GenuineIntel");
	} else if (regs[1] == 0x68747541U && regs[3] == 0x69746E65U && regs[2] == 0x444D4163U) {
		vendor = AUTHENTICAMD;
		TRACE("%s\n", "AuthenticAMD");
	} else {
		vendor = OTHER;
		TRACE("vendor %08x-%08x-%08x\n", regs[0], regs[2], regs[1]);
	}

	if (vendor == GENUINEINTEL)
		cache = do_query_x86_cache_hierarchy_intel(max_feature);
	else if (vendor == AUTHENTICAMD)
		cache = do_query_x86_cache_hierarchy_amd(max_feature);

	TRACE("final hierarchy: L1 %lu / %lu, L2: %lu / %lu, L3: %lu / %lu\n",
		cache.l1d, cache.l1d_threads, cache.l2, cache.l2_threads, cache.l3, cache.l3_threads);

	cache.valid = cache.l1d && cache.l1d_threads && !(cache.l2 && !cache.l2_threads) && !(cache.l3 && !cache.l3_threads);
	return cache;
}

} // namespace


unsigned long cpu_cache_per_thread_x86() noexcept
{
	static const X86CacheHierarchy cache = do_query_x86_cache_hierarchy();

	if (!cache.valid)
		return 0;

	// Detect Skylake-SP cache hierarchy and report L2 size instead of L3.
	if (cache.l3 && !cache.l3_inclusive && cache.l2 >= 1024 * 1024U && cache.l2_threads <= 2)
		return cache.l2 / cache.l2_threads;

	if (cache.l3)
		return cache.l3 / cache.l3_threads;
	else if (cache.l2)
		return cache.l2 / cache.l2_threads;
	else
		return cache.l1d / cache.l1d_threads;
}

} // namespace graphengine

#endif // defined(__i386) || defined(_M_IX86) || defined(_M_X64) || defined(__x86_64__)
