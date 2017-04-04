#include "spea2/algorithm.hpp"

#include <catch.hpp>

#include <algorithm>
#include <iostream>
#include <random>
#include <valarray>

class knapsack
{
public:
  static constexpr auto objective_count = 2ul;

  using individual_type = std::valarray<bool>;
  using generator_type = std::mt19937;

  knapsack(std::array<std::valarray<double>, objective_count> values,
           std::valarray<double> weights, double capacity, double mutation_rate,
           double recombination_rate)
    : values{std::move(values)}
    , weights{std::move(weights)}
    , capacity{capacity}
    , mutation_rate{mutation_rate}
    , recombination_rate{recombination_rate}
  {
    const auto size = this->weights.size();
    for (const auto& v : this->values)
      if (v.size() != this->weights.size())
        throw std::runtime_error{"mismatching sizes"};
  }

  auto evaluate(const individual_type& x, generator_type&) const
  {
    auto result = std::array<double, objective_count>{};

    for (auto i = 0ul; i < objective_count; ++i)
      result[i] = weights[x].sum() > capacity ? 0.0 : -values[i][x].sum();

    return result;
  }

  auto mutate(individual_type& x, generator_type& g) const -> void
  {
    for (auto& allele : x)
      if (spea2::draw(mutation_rate, g))
        allele = !allele;
  }

  auto recombine(const individual_type& parent1, const individual_type& parent2,
                 generator_type& g) const -> std::array<individual_type, objective_count>
  {
    if (!spea2::draw(recombination_rate, g))
      return {{parent1, parent2}};

    auto mask = individual_type(parent1.size());
    for (auto& v : mask)
      v = spea2::draw(0.5, g);

    return {{
      (parent1 && mask) || (parent2 && !mask), // first child
      (parent1 && !mask) || (parent2 && mask)  // second child
    }};
  }

private:
  std::array<std::valarray<double>, objective_count> values;
  std::valarray<double> weights;
  double capacity;

  double mutation_rate, recombination_rate;
};

static_assert(spea2::Problem<knapsack>::value,
              "Knapsack problem doesn't comply with spea2::Problem concept");

template <typename F> static auto generate_random_valarray(std::size_t size, F f)
{
  auto result = std::valarray<std::result_of_t<F()>>(size);
  std::generate_n(begin(result), size, std::move(f));
  return result;
}

template <typename F> static auto generate_random_vector(std::size_t size, F f)
{
  auto result = std::vector<std::result_of_t<F()>>();
  result.reserve(size);
  std::generate_n(back_inserter(result), size, std::move(f));
  return result;
}

TEST_CASE("Knapsack", "")
{
  static constexpr auto item_count = 50u;
  static constexpr auto generation_count = 100u;
  static constexpr auto population_size = 100u;
  static constexpr auto archive_size = 20u;

  // Our pseudo-random number generator.
  auto generator = std::mt19937{17};

  // Initial population.
  auto initial_population = generate_random_vector(population_size, [&] {
    auto individual = knapsack::individual_type(item_count);
    if (individual.size() != item_count)
      std::abort();
    for (auto& allele : individual)
      allele = spea2::draw(0.1, generator);
    return individual;
  });

  // Problem.
  const auto random_values = [&]() {
    return generate_random_valarray(
      item_count, [&] { return std::generate_canonical<double, 64>(generator); });
  };

  auto problem = knapsack{/* values = */ {{random_values(), random_values()}},
                          /* weights = */ random_values(),
                          /* capacity = */ 0.3 * item_count,
                          /* mutation = */ 1.0 / item_count,
                          /* crossover = */ 0.4};

  // Algorithm
  auto algorithm =
    spea2::make_algorithm(std::move(problem), std::move(initial_population), archive_size,
                          std::move(generator));

  // Iterate and print best in each iteration.
  for (auto t = 0u; t < generation_count; ++t)
    algorithm.iterate();

  auto solutions = std::vector<typename decltype(algorithm)::solution_type>();
  const auto nondominated = algorithm.nondominated();

  solutions.reserve(nondominated.size());
  std::copy(nondominated.begin(), nondominated.end(), back_inserter(solutions));
  std::sort(solutions.begin(), solutions.end(),
            [](const auto& a, const auto& b) { return a.fx < b.fx; });

  for (const auto& solution : solutions)
  {
    std::cout << std::fixed << std::setprecision(4) << "x = ";
    for (auto allele : solution.x)
      std::cout << (allele ? 1 : 0);
    std::cout << "\tf(x) = [";
    for (auto v : solution.fx)
      std::cout << ' ' << -v;
    std::cout << " ]\n";
  }
}
