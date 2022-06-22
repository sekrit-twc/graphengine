#pragma once

#ifndef GRAPHENGINE_TYPES_H_
#define GRAPHENGINE_TYPES_H_

#include <cstddef>

namespace graphengine {

constexpr unsigned BUFFER_MAX = ~0U;

constexpr unsigned FILTER_MAX_DEPS = 3;
constexpr unsigned FILTER_MAX_PLANES = 3;
constexpr unsigned NODE_MAX_PLANES = 4;

constexpr unsigned GRAPH_MAX_ENDPOINTS = 8;

typedef int node_id;
constexpr node_id null_node = -1;
constexpr node_id node_id_max = 1023;

struct node_dep_desc {
	node_id id = null_node;
	unsigned plane = 0;
};
constexpr node_dep_desc null_dep{};

struct PlaneDescriptor {
	unsigned width;
	unsigned height;
	unsigned bytes_per_sample;
};

struct BufferDescriptor {
	void *ptr;
	ptrdiff_t stride;
	unsigned mask;

	template <class T = void>
	T *get_line(unsigned i) const
	{
		return reinterpret_cast<T *>(static_cast<unsigned char *>(ptr) + static_cast<ptrdiff_t>(i & mask) * stride);
	}
};


// STL replacement types.
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
