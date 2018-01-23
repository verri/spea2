#ifndef SPEA2_UTILITIES_HPP
#define SPEA2_UTILITIES_HPP

#include <spea2/meta.hpp>

#include <algorithm>
#include <array>
#include <vector>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/range/irange.hpp>

namespace spea2
{
namespace util
{

template <
  typename Integer1, typename Integer2,
  typename = meta::requires<std::is_integral<Integer1>, std::is_integral<Integer2>>>
auto range(Integer1 first, Integer2 last)
{
  return boost::irange<std::common_type_t<Integer1, Integer2>>(first, last);
}

template <typename... Ts, typename = std::enable_if_t<(sizeof...(Ts) > 0ul)>>
auto indexes_of(const Ts&... args)
{
  const auto sizes = std::array<std::size_t, sizeof...(Ts)>{{args.size()...}};

  if (!std::all_of(begin(sizes), end(sizes),
                   [size = sizes[0]](const auto& s) { return s == size; }))
    throw std::runtime_error{"indexes_of: container sizes mismatch"};

  return range(0u, sizes[0]);
}

using boost::make_iterator_range;
using boost::geometry::index::nearest;
using boost::geometry::distance;

template <std::size_t N>
using point = boost::geometry::model::point<double, N, boost::geometry::cs::cartesian>;

template <std::size_t N>
using rtree = boost::geometry::index::rtree<point<N>, boost::geometry::index::rstar<32>>;

namespace detail
{
template <std::size_t N, std::size_t... I>
auto to_point(const std::array<double, N>& arr, std::index_sequence<I...>) -> point<N>
{
  return {arr[I]...};
}
} // namespace detail

template <std::size_t N> auto to_point(const std::array<double, N>& arr)
{
  return detail::to_point(arr, std::make_index_sequence<N>());
}

template <typename T> class setter_iterator
{
public:
  explicit setter_iterator(T& value)
    : ptr_{&value}
  {
  }

  auto operator++() -> setter_iterator& { return *this; }
  auto operator++(int) -> setter_iterator& { return *this; }
  auto operator*() -> T& { return *ptr_; }

private:
  T* ptr_;
};

template <typename T> auto setter(T& value) { return setter_iterator<T>{value}; }

} // namespace util
} // namespace spea2

#endif // SPEA2_UTILITIES_HPP
