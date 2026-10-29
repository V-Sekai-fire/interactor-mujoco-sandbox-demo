# repository-witness-cpp

A header-only, exception-free property-testing library for C++17, with an optional doctest adapter.

## What it is for

It checks a property against generated inputs over a ladder of growing search budgets, and reports either the counterexample it found or that every rung passed. A failing input is shrunk, and the run prints the seed that reproduces it. The ladder follows the Lean `plausible-witness-dag` library; this is an independent C++ implementation that builds without exceptions or RTTI.

## Building and running

```sh
cmake -B build -S .
cmake --build build
ctest --test-dir build
```

The library is the headers alone; the build above compiles and runs its tests. A CMake consumer links the `witness::witness` target, through `add_subdirectory` or through `find_package(witness-cpp)` after an install; any other build adds `include/` to its include path.

## Licence

MIT. See [LICENSE](LICENSE).
