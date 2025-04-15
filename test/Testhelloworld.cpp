#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this once per test-runner!

#include "catch.hpp"
#include "helloworld.hpp"

TEST_CASE("Test Hello World.") {
    REQUIRE(argv == 2);
}

