#pragma once

#include <vector>

using std::vector;
using Int = int_fast64_t;

struct PBclause {
  /* assuming always coefficients * lits >=  or == clauseConsVal */
  /* if ==, set equals bool to true */
  vector<Int> lits;
  vector<Int> coeffs;
  Int clauseConsVal;
  bool equals = false;
  Int clauseId = 0;
};