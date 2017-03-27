#include "spea2/algorithm.hpp"
#include "spea2/fitness.hpp"

#include <catch.hpp>

class Individual
{
public:
  using fitness_type = spea2::fitness<2>;

  Individual(int x) : x{x} {}
  auto fitness() -> fitness_type { return {x * 10.0, 1.0 / (x + 1.0)}; }

private:
  int x;
};

struct Problem {
  using individual_type = Individual;

  auto generate_individual() -> individual_type { return std::rand(); }
};

TEST_CASE("Simple test problem", "[foobar]")
{
  auto model = spea2::make_algorithm(Problem{}, 100u, 100u);
  (void)model;
}
