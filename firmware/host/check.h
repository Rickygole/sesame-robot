// check.h -- tiny host-side assert harness. Deliberately not a vendored
// unit test framework: this is ~40 lines for six files of math, and
// there's nothing else installed on this machine to depend on.
#pragma once

#include <cmath>
#include <cstdio>

extern int g_fails;

#define CHECK(expr)                                                  \
  do {                                                                \
    if (!(expr)) {                                                    \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
      ++g_fails;                                                      \
    }                                                                 \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                          \
  do {                                                                 \
    const double a_ = (double)(a);                                     \
    const double b_ = (double)(b);                                     \
    const double eps_ = (double)(eps);                                 \
    if (std::fabs(a_ - b_) > eps_) {                                   \
      std::fprintf(stderr,                                             \
                    "FAIL %s:%d: %s ~= %s (got %g vs %g, eps %g)\n",   \
                    __FILE__, __LINE__, #a, #b, a_, b_, eps_);          \
      ++g_fails;                                                       \
    }                                                                  \
  } while (0)
