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
using fallback = std::enable_if_t<conjunction<negation<Checks...>>::value>;

} // namespace meta

namespace detail
{

template <typename T> struct is_double_array : std::false_type
{
};

template <std::size_t N> struct is_double_array<std::array<double, N>> : std::true_type
{
};

template <typename T>
using use_mutate =
  decltype(std::declval<T&>().mutate(std::declval<typename T::individual_type&>(),
                                     std::declval<typename T::generator_type&>()));

template <typename T> using has_mutate = meta::compiles<T, use_mutate>;

template <typename T>
using use_recombine = decltype(
  std::declval<T&>().recombine(std::declval<const typename T::individual_type&>(),
                               std::declval<const typename T::individual_type&>(),
                               std::declval<typename T::generator_type&>()));

template <typename T> using has_recombine = meta::compiles<T, use_recombine>;

template <typename T>
using use_evaluate =
  decltype(std::declval<T&>().evaluate(std::declval<const typename T::individual_type&>(),
                                       std::declval<typename T::generator_type&>()));

template <typename T> using has_evaluate = meta::compiles<T, use_evaluate>;

} // namespace detail

template <typename T, typename E = void> struct Problem : std::false_type
{
};

template <typename T>
struct Problem<
  T,
  meta::requires<meta::conjunction<
    meta::compiles<T, detail::has_mutate>, std::is_same<detail::use_mutate<T>, void>,
    meta::compiles<T, detail::has_recombine>,
    std::is_same<detail::use_recombine<T>, std::array<typename T::individual_type, 2u>>,
    meta::compiles<T, detail::has_evaluate>,
    detail::is_double_array<detail::use_evaluate<T>>,
    std::integral_constant<bool, (T::objective_count > 0 &&
                                  T::objective_count ==
                                    std::tuple_size<detail::use_evaluate<T>>::value)> //
    >>> : std::true_type
{
};
} // namespace spea2

#endif // SPEA2_META_HPP
