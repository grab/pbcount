/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#pragma once

/* inclusions *****************************************************************/

#include "util.hpp"

/* uses ***********************************************************************/

using util::printComment;
using util::printThickLine;
using util::printThinLine;
using util::showError;
using util::showWarning;

/* global functions ***********************************************************/

void printDd(const Cudd &mgr, const ADD &dd, int n, int pr);

void writeDd(const Cudd &mgr, const ADD &dd, const string &filePath);

ADD projectDdVar(const Cudd &mgr, const ADD &dd, Int var);

void mainVisual(int argc, char *argv[]);
