#ifndef SPEA2_ALGORITHM_HPP
#define SPEA2_ALGORITHM_HPP

#include <type_traits>
#include <vector>

namespace spea2
{

template <typename Problem> class algorithm
{
  using individual_type = typename Problem::individual_type;
  using fitness_type = typename individual_type::fitness_type;

  static constexpr auto nobjectives = fitness_type::nobjectives;

public:
  algorithm(Problem problem, std::size_t population_size, std::size_t archive_size)
    : problem{std::move(problem)}, population_size{population_size}, archive_size{archive_size}
  {
    initialize();
  }

  auto iterate() -> void {}

private:
  auto initialize() -> void
  {
    population.reserve(population_size);
    archive.reserve(archive_size);
    for (auto i = 0ul; i < population_size; ++i)
      population.push_back(problem.generate_individual());
  }

  Problem problem;
  std::size_t population_size, archive_size;

  std::vector<individual_type> population, archive;
};

template <typename Problem>
auto make_algorithm(Problem&& problem, std::size_t population_size) -> algorithm<std::decay_t<Problem>>
{
  return {std::forward<Problem>(problem), population_size, population_size};
}

template <typename Problem>
auto make_algorithm(Problem&& problem, std::size_t population_size, std::size_t archive_size) -> algorithm<std::decay_t<Problem>>
{
  return {std::forward<Problem>(problem), population_size, archive_size};
}
}

#endif // SPEA2_ALGORITHM_HPP
