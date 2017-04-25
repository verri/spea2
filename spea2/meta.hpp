#ifndef SPEA2_META_HPP
#define SPEA2_META_HPP

#include <array>
#include <type_traits>

namespace spea2
{
namespace meta
{

// TODO: In C++17, use std version.
template <typename...> using void_t = void;

template <typename...> struct conjunction : std::true_type
{
};

template <typename B> struct conjunction<B> : B
{
};

template <typename B, typename... Bs>
struct conjunction<B, Bs...> : std::conditional_t<bool(B::value), conjunction<Bs...>, B>
{
};

template <bool B> using bool_constant = std::integral_constant<bool, B>;

template <typename B> struct negation : bool_constant<!B::value>
{
};

template <typename T> struct always_false : std::false_type
{
};

// Concepts emulation

template <typename T, template <typename> class Expression, typename = void_t<>>
struct compiles : std::false_type
{
};

template <typename T, template <typename> class Expression>
struct compiles<T, Expression, void_t<Expression<T>>> : std::true_type
{
};

template <typename... Checks>
using requires = std::enable_if_t<conjunction<Checks...>::value>;

template <typename... Checks>
using fallback = std::enable_if_t<conjunction<negation<Checks>...>::value>;

template <std::size_t N, typename T> struct is_double_array : std::false_type
{
};

template <std::size_t N> struct is_double_array<N, std::array<double, N>> : std::true_type
{
};

template <typename T>
using mutate_result =
  decltype(std::declval<T&>().mutate(std::declval<typename T::individual_type&>(),
                                     std::declval<typename T::generator_type&>()));

template <typename T> using has_mutate = meta::compiles<T, mutate_result>;

template <typename T>
using recombine_result = decltype(
  std::declval<T&>().recombine(std::declval<const typename T::individual_type&>(),
                               std::declval<const typename T::individual_type&>(),
                               std::declval<typename T::generator_type&>()));

template <typename T> using has_recombine = meta::compiles<T, recombine_result>;

template <typename T>
using evaluate_result =
  decltype(std::declval<T&>().evaluate(std::declval<typename T::individual_type&>(),
                                       std::declval<typename T::generator_type&>()));

template <typename T> using has_evaluate = meta::compiles<T, evaluate_result>;

template <typename T, typename E = void> struct Problem : std::false_type
{
};

template <typename T>
struct Problem<
  T, requires<conjunction<
       has_mutate<T>, std::is_same<mutate_result<T>, void>, has_recombine<T>,
       std::is_same<recombine_result<T>, std::array<typename T::individual_type, 2u>>,
       has_evaluate<T>, is_double_array<T::objective_count, evaluate_result<T>>,
       bool_constant<(T::objective_count > 0)>>>> : std::true_type
{
};

} // namespace meta
} // namespace spea2

#endif // SPEA2_META_HPP
