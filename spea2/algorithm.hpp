#ifndef SPEA2_ALGORITHM_HPP
#define SPEA2_ALGORITHM_HPP

#include <spea2/meta.hpp>
#include <spea2/utilities.hpp>

#include <algorithm>
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
  for (const auto i : util::range(0ul, N))
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
    archive_.reserve(population_.size() + archive_size);
    end_nondominated_ = archive_.begin();
  }

  auto iterate() -> void
  {
    archive_.clear();
    std::move(population_.begin(), population_.end(), std::back_inserter(archive_));
    population_.clear();

    auto fitness = util::vtransform(archive_, [](auto&& ind) { return ind.fitness(); });

    auto past_nondominated = 0ul;
    auto first_dominated = archive_.size();

    while (past_nondominated < first_dominated)
    {
      const auto i = past_nondominated;
      auto is_nondominated = true;
      for (const auto j : util::range(0u, first_dominated))
      {
        if (dominates(fitness[j], fitness[i]))
        {
          // Individual i is dominated
          is_nondominated = false;
          --first_dominated;
          std::swap(fitness[i], fitness[first_dominated]);
          std::swap(archive_[i], archive_[first_dominated]);
          break;
        }
      }
      if (is_nondominated)
        ++past_nondominated;
    }

    end_nondominated_ = archive_.begin() + past_nondominated;
  }

  auto nondominated()
  {
    return boost::make_iterator_range(archive_.begin(), end_nondominated_);
  }

  auto archive() const -> const std::vector<individual_type>& { return archive_; }

private:
  std::vector<individual_type> population_, archive_;
  std::size_t archive_size_;
  double mutation_rate_, crossover_rate_;

  typename std::vector<individual_type>::iterator end_nondominated_;
};

template <typename T, typename = meta::requires<Individual<T>>>
auto make_algorithm(std::vector<T> population, std::size_t archive_size,
                    double mutation_rate, double crossover_rate)
  -> algorithm<std::tuple_size<detail::use_fitness<T>>::value, T>
{
  return {std::move(population), archive_size, mutation_rate, crossover_rate};
}

} // namespace spea2

#endif // SPEA2_ALGORITHM_HPP
