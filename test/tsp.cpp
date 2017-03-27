#include "spea2/algorithm.hpp"

#include <catch.hpp>

struct TSP {
};

TEST_CASE("Simple TSP problem", "[TSP]")
{
  auto model = spea2::make_algorithm(TSP{}, 100u, 100u);
  (void)model;
}
