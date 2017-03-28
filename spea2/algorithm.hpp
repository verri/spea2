#ifndef SPEA2_ALGORITHM_HPP
#define SPEA2_ALGORITHM_HPP

#include <spea2/meta.hpp>
#include <spea2/utilities.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iosfwd>
#include <type_traits>
#include <vector>

// NOTE: The resulting fitness is not always 100% compatible because when it is not
// necessary to calculate, we don't.

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
  using f_type = detail::use_f<individual_type>;

  static_assert(std::tuple_size<f_type>::value == N,
                "individual.f() should return a std::array with N elements");

public:
  struct solution_type
  {
    individual_type individual;
    double fitness;

    friend auto operator<<(std::ostream& os, const solution_type& s) -> std::ostream&
    {
      return os << s.individual << ",\tfitness = " << s.fitness;
    }
  };

  algorithm(std::vector<individual_type> population, std::size_t archive_size,
            double mutation_rate, double crossover_rate)
    : population_size_(population.size())
    , archive_size_(archive_size)
    , k_(std::sqrt(population_size_ + archive_size_))
    , mutation_rate_(mutation_rate)
    , crossover_rate_(crossover_rate)
  {
    next_population_.reserve(population.size());
    std::transform(population.begin(), population.end(),
                   std::back_inserter(next_population_), [](auto& ind) -> solution_type {
                     return {std::move(ind), 0.0};
                   });

    archive_.reserve(next_population_.size() + archive_size_);
    fx_.reserve(next_population_.size() + archive_size_);
    strength_.reserve(population.size() + archive_size_);
    environmental_selection();
  }

  auto iterate() -> void
  {
    // TODO: From Step 5
    environmental_selection();
  }

  auto nondominated() const
  {
    return util::make_iterator_range(archive_.begin(), end_nondominated_);
  }

  decltype(auto) archive() const { return (archive_); }

private:
  auto environmental_selection() -> void
  {
    std::transform(next_population_.cbegin(), next_population_.cend(),
                   std::back_inserter(fx_),
                   [](const auto& sol) { return sol.individual.f(); });

    std::move(next_population_.begin(), next_population_.end(),
              std::back_inserter(archive_));

    next_population_.clear();

    auto past_nondominated = 0ul;
    auto first_dominated = archive_.size();

    while (past_nondominated < first_dominated)
    {
      const auto i = past_nondominated;
      auto is_nondominated = true;
      for (const auto j : util::range(0u, first_dominated))
      {
        if (dominates(fx_[j], fx_[i]))
        {
          // Individual i is dominated
          is_nondominated = false;
          --first_dominated;
          std::swap(fx_[i], fx_[first_dominated]);
          std::swap(archive_[i], archive_[first_dominated]);
          break;
        }
      }
      if (is_nondominated)
        ++past_nondominated;
    }

    end_nondominated_ = archive_.cbegin() + past_nondominated;

    if (past_nondominated == archive_size_)
      return;

    if (past_nondominated < archive_size_)
    {
      // TODO: fill out with the best dominated individuals

      // Strength:
      strength_.clear();
      std::fill_n(std::back_inserter(strength_), archive_.size(), 0.0);

      const auto all_ix = util::indexes_of(archive_, strength_, fx_);
      const auto dominated_ix = util::range(past_nondominated, archive_.size());
      const auto nondominated_ix = util::range(0u, past_nondominated);

      for (const auto i : all_ix)
        for (const auto j : dominated_ix)
          if (dominates(fx_[i], fx_[j]))
            ++strength_[i];

      // Raw Fitness:
      for (const auto i : nondominated_ix)
        archive_[i].fitness = 0.0;

      for (const auto i : dominated_ix)
        for (const auto j : all_ix)
          if (dominates(fx_[j], fx_[i]))
            archive_[i].fitness += strength_[j];

      //  Distances:
      auto tree = util::rtree<N>{};
      for (const auto i : all_ix)
        tree.insert(std::make_pair(util::to_point(fx_[i]), i));

      // Fitness:
      for (const auto i : dominated_ix)
      {
        const auto current = util::to_point(fx_[i]);
        auto kth_nei = util::indexed_point<N>{};

        tree.query(util::nearest(current, k_), util::setter(kth_nei));
        archive_[i].fitness += 1.0 / (util::distance(current, kth_nei.first) + 2);
      }

      // Partially sort dominated
      std::partial_sort( //
        archive_.begin() + past_nondominated, //
        archive_.begin() + archive_size_, //
        archive_.end(), //
        [](const auto& x, const auto& y) { return x.fitness < y.fitness; });
    }
    else
    {
      // TODO: truncate, i.e., ignore some nondominated individuals
    }

    if (archive_.size() > archive_size_)
      archive_.erase(archive_.begin() + archive_size_, archive_.end());
  }

  std::vector<solution_type> next_population_, archive_;
  const std::size_t population_size_, archive_size_, k_;
  const double mutation_rate_, crossover_rate_;

  std::vector<f_type> fx_;
  std::vector<std::size_t> strength_;

  typename decltype(archive_)::const_iterator end_nondominated_;
};

template <typename T, typename = meta::requires<Individual<T>>>
auto make_algorithm(std::vector<T> population, std::size_t archive_size,
                    double mutation_rate, double crossover_rate)
  -> algorithm<std::tuple_size<detail::use_f<T>>::value, T>
{
  return {std::move(population), archive_size, mutation_rate, crossover_rate};
}

} // namespace spea2

#endif // SPEA2_ALGORITHM_HPP
