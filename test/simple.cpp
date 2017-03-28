#include "spea2/algorithm.hpp"

#include <catch.hpp>
#include <cmath>
#include <iostream>

static auto pow6(double x) { return x * x * x * x * x * x; }
static auto square(double x) { return x * x; }
static auto f1(double x)
{
  return 1 - std::exp(-4.0 * x) * pow6(std::sin(6.0 * 3.1415 * x));
}
static auto g(double x) { return 1.0 + 9 * std::pow(x, 0.25); }
static auto f2(double x) { return g(x) * (1.0 - square(f1(x) / g(x))); }

static auto drand() { return static_cast<double>(std::rand()) / RAND_MAX; }

class individual
{
public:
  individual(double x)
    : x{x}
  {
  }

  auto f() const -> std::array<double, 2u> { return {{f1(x), f2(x)}}; }

  auto mutate(double rate) -> void
  {
    if (drand() < rate)
      x = drand();
  }

  auto crossover(individual& other) -> void { std::swap(x, other.x); }

  friend auto operator<<(std::ostream& os, const individual& i) -> std::ostream&
  {
    const auto fx = i.f();
    return os << "x=(" << i.x << "), f(x)=(" << fx[0] << ", " << fx[1] << ')';
  }

private:
  double x;
};

TEST_CASE("Simple test problem", "[foobar]")
{
  auto initial_population = std::vector<individual>{};
  initial_population.reserve(7u);
  std::generate_n(std::back_inserter(initial_population), initial_population.capacity(),
                  drand);

  auto model = spea2::make_algorithm(std::move(initial_population), 5u, 0.01, 0.4);
  // model.iterate();

  std::cout << "Archive: " << std::endl;
  for (const auto& ind : model.archive())
    std::cout << '\t' << ind << std::endl;

  std::cout << "Nondominated: " << std::endl;
  for (const auto& ind : model.nondominated())
    std::cout << '\t' << ind << std::endl;
}
