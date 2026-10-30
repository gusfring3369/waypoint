#include <exception>
#include <iostream>

#include "test_framework.hpp"

int main() {
    int failedTests = 0;
    for (const auto& test : testing::registry()) {
        const int failuresBefore = testing::failureCount();
        try {
            test.body();
        } catch (const testing::AbortTestCase&) {
            // failure already recorded by REQUIRE
        } catch (const std::exception& e) {
            ++testing::failureCount();
            std::cerr << "    unexpected exception: " << e.what() << '\n';
        }
        const bool passed = testing::failureCount() == failuresBefore;
        failedTests += passed ? 0 : 1;
        std::cout << (passed ? "[ PASS ] " : "[ FAIL ] ") << test.name << '\n';
    }

    std::cout << '\n'
              << testing::registry().size() - static_cast<std::size_t>(failedTests) << '/'
              << testing::registry().size() << " test cases passed\n";
    return failedTests == 0 ? 0 : 1;
}
