#ifndef SPEA2_ALGORITHM_HPP
#define SPEA2_ALGORITHM_HPP

#include <type_traits>
#include <vector>

namespace spea2
{

template <typename Problem> class algorithm
{
public:
  algorithm(Problem problem, std::size_t, std::size_t) : problem{std::move(problem)} {}

private:
  Problem problem;
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
