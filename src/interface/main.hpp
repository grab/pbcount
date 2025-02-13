/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#pragma once
/* inclusions *****************************************************************/

#include <limits>

#include "../../lib/cxxopts.hpp"

#include "counter.hpp"
#include "pbformula.hpp"
#include "interactive.hpp"

// #include <boost/process.hpp>
// #include <boost/filesystem.hpp>
// namespace bp = boost::process;
/* classes ********************************************************************/

class OptionDict
{
public:
  /* optional: */
  bool helpFlag;
  string cnfFilePath;
  Int weightFormatOption;
  string jtFilePath;
  Float jtWaitSeconds;
  Int outputFormatOption;
  Int clusteringHeuristicOption;
  Int cnfVarOrderingHeuristicOption;
  Int ddVarOrderingHeuristicOption;
  Int clauseCompilationHeuristicOption;
  Int randomSeedOption;
  Int verbosityLevelOption;
  Int preprocessingOption;
  Int operationModeOption;
  Int interactionAdaptiveRestartOption;

  cxxopts::Options *options;

  void printOptionalOptions() const;
  void printHelp() const;
  void printWelcome() const;
  OptionDict(int argc, char *argv[]);
};

/* namespaces *****************************************************************/

namespace pbsolving
{
  void pbsolveInteractive(
      const string &pbFilePath, 
      WeightFormat weightFormat, 
      OutputFormat outputFormat, 
      ClusteringHeuristic clusteringHeuristic,
      VarOrderingHeuristic pbVarOrderingHeuristic,
      bool inversePbVarOrdering,
      VarOrderingHeuristic ddVarOrderingHeuristic,
      bool inverseDdVarOrdering,
      ClauseCompilationHeuristic clauseCompilationHeuristic,
      AdaptiveRestartChoice adaptiveRestartChoice);
  void pbsolveFile(
      const string &pbFilePath,
      WeightFormat weightFormat,
      const string &jtFilePath,
      Float jtWaitSeconds,
      OutputFormat outputFormat,
      ClusteringHeuristic clusteringHeuristic,
      VarOrderingHeuristic pbVarOrderingHeuristic,
      bool inversePbVarOrdering,
      VarOrderingHeuristic ddVarOrderingHeuristic,
      bool inverseDdVarOrdering,
      ClauseCompilationHeuristic clauseCompilationHeuristic,
      PreprocessingConfig preprocessingConfig,
      OperationModeChoice operationModeChoice,
      AdaptiveRestartChoice adaptiveRestartChoice);
  void pbsolveOptions(
      const string &pbFilePath,
      Int weightFormatOption,
      const string &jtFilePath,
      Float jtWaitSeconds,
      Int outputFormatOption,
      Int clusteringHeuristicOption,
      Int pbVarOrderingHeuristicOption,
      Int ddVarOrderingHeuristicOption,
      Int clauseCompilationHeuristicOption,
      Int preprocessingOption,
      Int operationMode,
      Int interactionAdaptiveRestartMode);
  void pbsolveCommand(int argc, char *argv[]);
}
/* global functions ***********************************************************/

int main(int argc, char *argv[]);