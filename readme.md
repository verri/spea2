# SPEA2

This is a header-only, generic C++ implementation of the SPEA2 algorithm<sup>[1](#paper)</sup>.

## Requirements

To use the library, you will need

- C++14-compliant compiler
- Boost >=1.63
  - [Boost.Range](http://www.boost.org/doc/libs/1_63_0/libs/range/doc/html/index.html)
  - [Boost.Geometry](http://www.boost.org/doc/libs/1_63_0/libs/geometry/doc/html/index.html)

Older versions of Boost might work, but I haven't tested it.

## Documentation

This library and all its requirements are header-only.  Thus, the only thing you need is to include
the headers in your project.  The core of the library is the class `spea2::algorithm` which expects
a user-defined optimization problem as a template argument.  Such feature enables a generic, optimized
library with no virtual calls.

The user-defined problem must comply with the following class:
```c++
class problem
{
public:
  static constexpr std::size_t objective_count = /* ... */;
  using individual_type = /* ... */;
  using generator_type = /* ... */;

  auto evaluate(const individual_type&, generator_type&)
    -> std::array<double, objective_count>;
  
  auto recombine(const individual_type&, const individual_type&, generator_type&)
    -> std::array<individual_type, objective_count>;
  
  auto mutate(individual_type&, generator_type&) 
    -> void;
    
   /* ... */
};
```

The static member `problem::objective_count` is the number of objectives in the problem.
The aliases (or nested classes) `problem::individual_type` and `problem::generator_type` 
are the type of the individuals and a 
[random number engine](http://en.cppreference.com/w/cpp/numeric/random), respectively.

The function members `problem::evaluate`, `problem::recombine`, and `problem::mutate` 
define the functioning of the algorithm.  

`problem::evaluate` is called *exactly once* per individual during the execution of the algorithm.
It receives the individual to evaluate and a random engine, and it must return one `double`
per objective.  The algorithm will try to minimize such objectives.

`problem::recombine` is called for every two selected (by binary tournament) individuals
from the archive to populate the next generation.  It receives references to the individuals 
and a random engine, and it must return two new individual.  The new individuals (which can
be the same, if the user wants to) go the next population.

`problem::mutate` is called *exactly once* per individual outputed by `problem::recombine`.
It receives the individual and a random engine.  This is the only function that can (and should)
modify the input individual.

The class doesn't necessarily need to have exactly the same function-member signatures,
however the return time must match. Parameters type must be compatible.  If the user-defined
class doesn't comply with the requirement, a `static_assert` is triggered, avoiding nasty
compiler cries.

## Example

To illustrate the usage, let's implement a multi-objective
[Knapsack problem](https://en.wikipedia.org/wiki/Knapsack_problem).

First, we include the required headers.
```c++
#include <spea2/algorithm> // spea2::make_algorithm, spea2::draw
#include <random>          // std::mt19937
#include <valarray>        // std::valarray
```

## Contributing

I've tried to implement it as close as possible to the original algorithm.
Any discrepancy may be reported at the [issue tracker](https://github.com/verri/spea2/issues).

The library should be flexible enough to work in most of the common cases, 
but if it doesn't fit into your problem, please let me know (or send me a pull request).

Pull requests are welcomed, especially for

- performance optimizations 
- C++11 compliance
  - that should be possible with only few tweaks
- dependency reduction
  - Boost.Range could be easily dropped
- improve genericity

## References

<a name="paper">1</a>: Zitzler, E., Laumanns, M., & Thiele, L. (2001). 
"SPEA2: Improving the Strength Pareto Evolutionary Algorithm". 
Evolutionary Methods for Design Optimization and Control with Applications to Industrial Problems, 95–100.
http://doi.org/10.1.1.28.7571
