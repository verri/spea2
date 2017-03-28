#ifndef SPEA2_ALGORITHM_HPP
#define SPEA2_ALGORITHM_HPP

#include <spea2/meta.hpp>

#include <array>
#include <type_traits>
#include <vector>

namespace spea2
{

template <typename T, std::size_t N>
static auto dominates(const std::array<T, N>& a, const std::array<T, N>& b)
{
  if (a == b)
    return false;
  for (auto i = 0ul; i < N; ++i)
    if (a[i] > b[i])
      return false;
  return true;
}

template <std::size_t N, typename T, typename E = void> class algorithm
{
  static_assert(meta::always_false<T>::value,
                "Individual type doesn't complies with the required concepts");
};

template <std::size_t N, typename T> class algorithm<N, T, meta::requires<Individual<T>>>
{
  using individual_type = T;
  using fitness_type = decltype(std::declval<const individual_type&>().fitness());

  static_assert(std::tuple_size<fitness_type>::value == N,
                "individual.fitness() should return a std::array with N elements");

public:
  algorithm(std::vector<individual_type> population, std::size_t archive_size,
            double mutation_rate, double crossover_rate)
    : population_(std::move(population))
    , archive_size_(archive_size)
    , mutation_rate_(mutation_rate)
    , crossover_rate_(crossover_rate)
  {
  }

private:
  std::vector<individual_type> population_;
  std::size_t archive_size_;
  double mutation_rate_, crossover_rate_;
};

} // namespace spea2

#endif // SPEA2_ALGORITHM_HPP
