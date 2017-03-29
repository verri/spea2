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

template <typename R, typename... Checks>
using requires_t = std::enable_if_t<conjunction<Checks...>::value, R>;

template <typename R, typename... Checks>
using fallback_t = std::enable_if_t<conjunction<negation<Checks...>>::value, R>;

} // namespace meta

namespace detail
{

template <typename T> struct is_double_array : std::false_type
{
};

template <std::size_t N> struct is_double_array<std::array<double, N>> : std::true_type
{
};

template <typename T> using use_mutate = decltype(std::declval<T&>().mutate(0.0));

template <typename T> using has_mutate = meta::compiles<T, use_mutate>;

template <typename T>
using use_recombine =
  decltype(recombine(std::declval<const T&>(), std::declval<const T&>()));

template <typename T> using has_recombine = meta::compiles<T, use_recombine>;

template <typename T> using use_f = decltype(std::declval<const T&>().f());

template <typename T> using has_f = meta::compiles<T, use_f>;

} // namespace detail

template <typename T, typename E = void> struct Individual : std::false_type
{
};

template <typename T>
struct Individual<T,
                  meta::requires<meta::conjunction<                            //
                    meta::compiles<T, detail::has_mutate>,                     //
                    std::is_same<detail::use_mutate<T>, void>,                 //
                    meta::compiles<T, detail::has_recombine>,                  //
                    std::is_same<detail::use_recombine<T>, std::array<T, 2u>>, //
                    meta::compiles<T, detail::has_f>,                          //
                    detail::is_double_array<detail::use_f<T>>                  //
                    >>> : std::true_type
{
};
} // namespace spea2

#endif // SPEA2_META_HPP
