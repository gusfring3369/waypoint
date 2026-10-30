#pragma once

// A deliberately tiny unit test harness so the project builds with nothing but
// a C++20 compiler and CMake. Provides TEST_CASE, CHECK (records and
// continues), REQUIRE (records and aborts the test case) and CHECK_THROWS.

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace testing {

struct TestCase {
    std::string name;
    std::function<void()> body;
};

struct AbortTestCase {};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline int& failureCount() {
    static int failures = 0;
    return failures;
}

struct Registrar {
    Registrar(std::string name, std::function<void()> body) {
        registry().push_back({std::move(name), std::move(body)});
    }
};

inline void reportFailure(const char* expression, const char* file, int line) {
    ++failureCount();
    std::cerr << "    " << file << ':' << line << ": check failed: " << expression << '\n';
}

inline bool approxEqual(double a, double b, double tolerance = 1e-9) {
    return std::fabs(a - b) <= tolerance * std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
}

}  // namespace testing

#define WAYPOINT_CONCAT_INNER(a, b) a##b
#define WAYPOINT_CONCAT(a, b) WAYPOINT_CONCAT_INNER(a, b)

#define TEST_CASE(name)                                                           \
    static void WAYPOINT_CONCAT(test_body_, __LINE__)();                          \
    static const ::testing::Registrar WAYPOINT_CONCAT(test_registrar_, __LINE__)( \
        name, &WAYPOINT_CONCAT(test_body_, __LINE__));                            \
    static void WAYPOINT_CONCAT(test_body_, __LINE__)()

#define CHECK(expr)                                                       \
    do {                                                                  \
        if (!(expr)) ::testing::reportFailure(#expr, __FILE__, __LINE__); \
    } while (false)

#define REQUIRE(expr)                                            \
    do {                                                         \
        if (!(expr)) {                                           \
            ::testing::reportFailure(#expr, __FILE__, __LINE__); \
            throw ::testing::AbortTestCase{};                    \
        }                                                        \
    } while (false)

#define CHECK_APPROX(a, b) CHECK(::testing::approxEqual((a), (b)))

#define CHECK_THROWS_AS(expr, exception_type)                                                       \
    do {                                                                                            \
        bool threw = false;                                                                         \
        try {                                                                                       \
            static_cast<void>(expr);                                                                \
        } catch (const exception_type&) {                                                           \
            threw = true;                                                                           \
        }                                                                                           \
        if (!threw) ::testing::reportFailure(#expr " throws " #exception_type, __FILE__, __LINE__); \
    } while (false)
