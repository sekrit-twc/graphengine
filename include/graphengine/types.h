#ifndef GRAPHENGINE_TYPES_H_
#define GRAPHENGINE_TYPES_H_

#include <cstddef>

namespace graphengine {

/** All-ones bit pattern representing an unbuffered plane. */
constexpr unsigned BUFFER_MAX = ~0U;

/** Maximum number of filter input planes. */
constexpr unsigned FILTER_MAX_DEPS = 3;
/** Maximum number of filter output planes. */
constexpr unsigned FILTER_MAX_PLANES = 3;

/**
 * Maximum number of planes in a graph node.
 *
 * One greater than {@p FILTER_MAX_PLANES} to account for packed alpha channels
 * in graph endpoints. Filter nodes are limited to FILTER_MAX_PLANES.
 */
constexpr unsigned NODE_MAX_PLANES = 4;

/** Maximum number of external nodes in a graph (7 inputs + 1 output). */
constexpr unsigned GRAPH_MAX_ENDPOINTS = 8;

/** Node handle. */
typedef int node_id;
/** Negative ids correspond to invalid nodes. */
constexpr node_id null_node = -1;
/** Maximum number of nodes per graph. */
constexpr node_id node_id_max = 1023;

/** Graph edges connect to a specific plane in a node. */
struct node_dep_desc {
	node_id id = null_node; /**< Node containing plane. */
	unsigned plane = 0;     /**< Plane index. */
};
constexpr node_dep_desc null_dep{};

/**
 * Plane descriptor. Planes are 2D arrays of samples.
 */
struct PlaneDescriptor {
	unsigned width;            /**< Width, or the inner dimension. */
	unsigned height;           /**< Height, or the outer dimension. */
	unsigned bytes_per_sample; /**< 1, 2, or 4. */
};

/**
 * Buffer descriptor.
 *
 * Processing occurs in circular arrays holding a power-of-2 number of rows.
 * The address of the i-th row is:
 *   ((char *)ptr) + (ptrdiff_t)(i & mask) * stride
 */
struct BufferDescriptor {
	void *ptr;        /** Pointer to first row. */
	ptrdiff_t stride; /** Distance between row in bytes. */
	unsigned mask;    /** Circular array mask, or {@p BUFFER_MAX} for unbuffered. */

	/**
	 * Helper function for calculating row addresses.
	 *
	 * @tparam T cast pointer to type
	 * @param i row index
	 * @return pointer to row
	 */
	template <class T = void>
	T *get_line(unsigned i) const
	{
		return (T *)((unsigned char *)ptr + (ptrdiff_t)(i & mask) * stride);
	}
};

/** Exception type. */
struct Exception {
	enum {
		UNKNOWN = 0,
		OUT_OF_MEMORY = 1,
		USER_CALLBACK = 2,
		ILLEGAL_STATE = 3,
		LIMIT_EXCEEDED = 4,
		INVALID_DESCRIPTOR = 5,
		INVALID_DIMENSIONS = 6,
		INVALID_NODE = 7,
	};

	int code;        /**< Error code, see above. */
	const char *msg; /**< Error message, statically allocated. */
};


// STL replacement types for API/ABI functions.
namespace detail {

template <class T, size_t N>
struct array {
	T _[N];

	T &operator[](size_t idx) { return _[idx]; }
	const T &operator[](size_t idx) const { return _[idx]; }

	T *data() { return _; }
	const T *data() const { return _; }

	T *begin() { return data(); }
	const T *begin() const { return data(); }

	T *end() { return data() + N; }
	const T *end() const { return data() + N; }
};

template <class T, class U>
struct pair {
	T first;
	U second;
};

template <class T>
class unique_ptr {
	T *m_ptr;
public:
	unique_ptr(std::nullptr_t x = nullptr) : m_ptr{} {}
	explicit unique_ptr(T *ptr) : m_ptr{ ptr } {}
	unique_ptr(unique_ptr &&other) noexcept : m_ptr{} { reset(other.release()); }

	~unique_ptr() { reset(); }

	unique_ptr &operator=(unique_ptr &&other) noexcept { reset(other.release()); return *this; }

	void reset(T *ptr = nullptr) { delete m_ptr; m_ptr = ptr; }
	T *release() { T *tmp = m_ptr; m_ptr = nullptr; return tmp; }
	T *get() const { return m_ptr; }

	explicit operator bool() const { return !!m_ptr; }

	T &operator*() const { return *m_ptr; }
	T *operator->() const { return m_ptr; }
};

} // namespace detail
} // namespace graphengine

#endif // GRAPHENGINE_TYPES_H_
