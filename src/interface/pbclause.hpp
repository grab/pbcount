/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

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
  /* adding an id just for debug print, to see which clauses are compiled in benchmarks*/
  Int clauseId = 0;
};