#ifndef SPEA2_FITNESS_HPP
#define SPEA2_FITNESS_HPP

#include <array>

namespace spea2
{

template <std::size_t N, typename V = double> struct fitness : std::array<V, N> {
  static constexpr auto nobjectives = N;

  template <typename... Ts> fitness(Ts&&... values) : std::array<V, N>{{std::forward<Ts>(values)...}} {}

  // Dominates
  auto operator<(const fitness& other) const -> bool
  {
    if (*this == other)
      return false;
    for (auto i = 0ul; i < nobjectives; ++i)
      if ((*this)[i] > other[i])
        return false;
    return true;
  }
};
}

#endif // SPEA2_FITNESS_HPP
