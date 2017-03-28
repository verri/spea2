#ifndef SPEA2_UTILITIES_HPP
#define SPEA2_UTILITIES_HPP

#include <spea2/meta.hpp>

#include <algorithm>
#include <array>
#include <vector>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/range/irange.hpp>

namespace spea2
{
namespace util
{

template <typename T, typename F,
          typename R = decltype(std::declval<F>()(std::declval<const T&>()))>
auto vtransform(const std::vector<T>& from, const F& f) -> std::vector<R>
{
  std::vector<R> result;
  result.reserve(from.size());
  std::transform(from.begin(), from.end(), std::back_inserter(result), f);
  return result;
}

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

using boost::geometry::index::query;
using boost::geometry::index::nearest;

namespace detail
{
template <typename T> class setter_iterator
{
public:
  setter_iterator(T* value)
    : ptr_{value}
  {
  }

  auto operator++() { return *this; }
  auto operator++(int) { return *this; }
  auto operator*() -> T& { return *ptr_; }

private:
  T* ptr_;
};
}

template <typename T> auto setter(T& value) -> detail::setter_iterator<T>
{
  return &value;
}

using boost::geometry::distance;

} // namespace util
} // namespace spea2

#endif // SPEA2_UTILITIES_HPP
