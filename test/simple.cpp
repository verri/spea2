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

  operator double() const { return x; }

  friend auto operator<<(std::ostream& os, const individual& x) -> std::ostream&
  {
    return os << "x = " << static_cast<double>(x) << ",\tf(x) = [" << f1(x) << ", "
              << f2(x) << ']';
  }

private:
  double x;
};

class problem
{
public:
  static constexpr auto objective_count = 2ul;
  using individual_type = individual;

  auto evaluate(individual_type x) const -> std::array<double, objective_count>
  {
    return {{f1(x), f2(x)}};
  }

  auto mutate(individual_type& x, double rate) const -> void
  {
    if (drand() < rate)
      x = drand();
  }

  auto recombine(const individual_type& a, const individual_type& b) const
    -> std::array<individual_type, 2u>
  {
    return {{b, a}};
  };
};

TEST_CASE("Simple test problem", "[foobar]")
{
  auto initial_population = std::vector<individual>{};
  initial_population.reserve(5u);
  std::generate_n(std::back_inserter(initial_population), initial_population.capacity(),
                  drand);

  const auto seed = std::random_device{}();
  auto model =
    spea2::make_algorithm(problem{}, std::move(initial_population), 5u, 0.1, 0.4, seed);

  std::cout << std::setprecision(6) << std::fixed;
  for (auto t = 0u; t < 3; ++t)
  {
    std::cout << "=== Iteration " << t << " ===" << std::endl;
    std::cout << "Archive: " << std::endl;
    for (const auto& ind : model.archive())
      std::cout << '\t' << ind << std::endl;

    std::cout << "Nondominated: " << std::endl;
    for (const auto& ind : model.nondominated())
      std::cout << '\t' << ind << std::endl;
    std::cout << std::endl;

    model.iterate();
  }
}
