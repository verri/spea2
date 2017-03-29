#ifndef SPEA2_ALGORITHM_HPP
#define SPEA2_ALGORITHM_HPP

#include <spea2/meta.hpp>
#include <spea2/utilities.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iosfwd>
#include <random>
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
  using f_type = detail::use_f<individual_type>;

  static_assert(std::tuple_size<f_type>::value == N,
                "individual.f() should return a std::array with N elements");

public:
  struct solution_type
  {
    individual_type x;
    f_type fx;
    double fitness;

    friend auto operator<<(std::ostream& os, const solution_type& s) -> std::ostream&
    {
      return os << s.x << ",\tfitness = " << s.fitness;
    }
  };

  algorithm(std::vector<individual_type> population, std::size_t archive_size,
            double mutation_rate, double recombination_rate, unsigned seed)
    : next_population_(std::move(population))
    , population_size_(next_population_.size())
    , archive_size_(archive_size)
    , k_(std::sqrt(population_size_ + archive_size_))
    , mutation_rate_(mutation_rate)
    , recombination_rate_(recombination_rate)
    , generator_(seed)
  {
    archive_.reserve(next_population_.size() + archive_size_);
    strength_.reserve(population.size() + archive_size_);
    environmental_selection();
  }

  // === Steps 5 and 6 ===
  // Step 3 (Environmental Selection) is performed at the end to produce the
  // archive and nondominated sets.
  auto iterate() -> void
  {
    // == Mating Selection, Recombination and Mutation ==
    // We perform binary tournament selection with replacement on the archive.
    auto dist = std::uniform_int_distribution<std::size_t>(0u, archive_.size() - 1u);
    const auto binary_tournament = [&]() -> const individual_type& {
      const auto i = rand(dist);
      const auto j = rand(dist);
      return archive_[i].fitness < archive_[j].fitness ? archive_[i].x : archive_[j].x;
    };

    while (next_population_.size() < population_size_)
    {
      // Two binary tournament to select the parents.
      const auto& parent1 = binary_tournament();
      const auto& parent2 = binary_tournament();

      // Children are either a recombination or the parents themselves.
      auto children = [&]() -> std::array<individual_type, 2u> {
        if (rand(dist) < recombination_rate_)
          return recombine(parent1, parent2);
        return {{parent1, parent2}};
      }();

      // Mutate and put children in the new population.
      for (auto& child : children)
      {
        child.mutate(mutation_rate_);
        next_population_.push_back(std::move(child));
        if (next_population_.size() == population_size_)
          break;
      }
    }

    // == Environmental Selection ==
    environmental_selection();
  }

  auto nondominated() const
  {
    return util::make_iterator_range(archive_.begin(), end_nondominated_);
  }

  decltype(auto) archive() const { return (archive_); }

private:
  // === Step 3 of the algorithm ===
  // Step 2 - Fitness assignment is called as part of this step.
  auto environmental_selection() -> void
  {
    // First, we move every individual to the archive.  Thus, it will contain
    // both the previous archive and the previous population.  It is necessary
    // to calculate f(x) for the individuals in the previous population.
    // In the next steps, we apply the necessary operations to keep the best solutions.
    std::transform(next_population_.begin(), next_population_.end(),
                   std::back_inserter(archive_), [](auto& x) -> solution_type {
                     auto fx = x.f();
                     return {std::move(x), std::move(fx), 0.0};
                   });

    // Next population gets empty for the Mating Selection, that should be
    // done at the iterate() method.
    next_population_.clear();

    // We assign the fitness of every solution. Even to those that were at
    // the previous archive, since fitness depends on the density of the
    // combined population.
    fitness_assignment();

    // Now, we count how many individuals are nondominated.
    const auto nondominated_count = static_cast<std::size_t>(std::count_if(
      archive_.begin(), archive_.end(), [](const auto& s) { return s.fitness < 1.0; }));

    // Then, we partially sort the individuals guaranteeing that all nondominated
    // individuals are properly sorted.
    std::partial_sort(                                                //
      archive_.begin(),                                               //
      archive_.begin() + std::max(nondominated_count, archive_size_), //
      archive_.end(),                                                 //
      [](const auto& x, const auto& y) { return x.fitness < y.fitness; });

    // If there are too many nondominated individuals,
    // apply the truncation procedure.
    if (nondominated_count > archive_size_)
      truncate(nondominated_count);

    // Keep only the `archive_size_` best individuals.
    if (archive_.size() > archive_size_)
      archive_.erase(archive_.begin() + archive_size_, archive_.end());

    // Updates nondominated end.
    end_nondominated_ = archive_.cbegin() + std::min(nondominated_count, archive_size_);
  }

  // === Step 2 of the algorithm ===
  auto fitness_assignment() -> void
  {
    // == Strength ==
    // We reset strength values, since they depend on the combined population.
    strength_.clear();
    std::fill_n(std::back_inserter(strength_), archive_.size(), 0ul);

    const auto all_ix = util::indexes_of(archive_, strength_);

    for (const auto i : all_ix)
      for (const auto j : all_ix)
        if (dominates(archive_[i].fx, archive_[j].fx))
          ++strength_[i];

    // == Raw Fitness ==
    // Assign raw fitness based on the strength values.
    for (const auto i : all_ix)
      archive_[i].fitness = 0.0;

    for (const auto i : all_ix)
      for (const auto j : all_ix)
        if (dominates(archive_[j].fx, archive_[i].fx))
          archive_[i].fitness += strength_[j];

    // == Tree of distances ==
    auto tree = util::rtree<N>{};
    for (const auto i : all_ix)
      tree.insert(util::to_point(archive_[i].fx));

    // == Density and Fitness ==
    // Update raw fitness adding the density information.
    for (const auto i : all_ix)
    {
      const auto current = util::to_point(archive_[i].fx);
      auto kth_nei = util::point<N>{};

      tree.query(util::nearest(current, k_), util::setter(kth_nei));
      archive_[i].fitness += 1.0 / (util::distance(current, kth_nei) + 2.0);
    }
  }

  // === Truncation procedure ===
  auto truncate(std::size_t nondominated_count) -> void
  {
    // The distances are now calculated only inside the nondominated set.
    auto tree = util::rtree<N>{};
    for (const auto i : util::range(0u, nondominated_count))
      tree.insert(util::to_point(archive_[i].fx));

    // Distances up to the kth neighbor are stored.
    // It may not be required to calculate so many distances, but we are doing
    // so because the truncation procedure is probably rare and we want to
    // comply to the original algorithm.
    auto vdistances = std::vector<std::vector<double>>(nondominated_count);
    for (auto& distances : vdistances)
      distances.reserve(k_);

    // Cache for the neighborhood so we don't keep allocating and deallocating
    // unnecessarily.
    auto neighborhood = std::vector<util::point<N>>();
    neighborhood.reserve(k_);

    // Iteratively "remove" some individuals.  In fact, we just move them outside
    // the range of nondominated.
    while (nondominated_count > archive_size_)
    {
      // Clear old distances.
      for (auto& distances : vdistances)
        distances.clear();

      // Fill all distances up to the kth neighbor.
      for (const auto i : util::range(0u, nondominated_count))
      {
        neighborhood.clear();
        const auto current = util::to_point(archive_[i].fx);

        tree.query(util::nearest(current, k_), std::back_inserter(neighborhood));
        for (const auto& neighbor : neighborhood)
          vdistances[i].push_back(util::distance(current, neighbor));
      }

      // Find the index which have smallest distance, and "remove" it.
      static const auto cmp = //
        +[](const std::vector<double>& a, const std::vector<double>& b) -> bool {
        for (const auto i : util::indexes_of(a, b))
          if (a[i] < b[i])
            return true;
          else if (a[i] > b[i])
            return false;
        return true;
      };

      const auto i = std::distance( //
        vdistances.begin(),         //
        std::min_element(vdistances.begin(), vdistances.end(), cmp));

      --nondominated_count;
      tree.remove(util::to_point(archive_[i].fx));
      std::swap(archive_[i], archive_[nondominated_count]);
      vdistances.pop_back();
    }
  }

  template <typename Distribution> auto rand(Distribution& dist)
  {
    return dist(generator_);
  }

  std::vector<individual_type> next_population_;
  std::vector<solution_type> archive_;
  const std::size_t population_size_, archive_size_, k_;
  const double mutation_rate_, recombination_rate_;

  std::vector<std::size_t> strength_;
  typename decltype(archive_)::const_iterator end_nondominated_;

  std::mt19937 generator_;
};

template <typename T, typename = meta::requires<Individual<T>>>
auto make_algorithm(std::vector<T> population, std::size_t archive_size,
                    double mutation_rate, double recombination_rate, unsigned seed)
  -> algorithm<std::tuple_size<detail::use_f<T>>::value, T>
{
  return {std::move(population), archive_size, mutation_rate, recombination_rate, seed};
}

template <typename T, typename = meta::fallback<Individual<T>>>
auto make_algorithm(std::vector<T>, std::size_t, double, double, unsigned)
{
  static_assert(meta::always_false<T>::value,
                "Individual type doesn't complies with the required concepts");
}

} // namespace spea2

#endif // SPEA2_ALGORITHM_HPP
