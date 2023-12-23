#pragma once
/* inclusions *****************************************************************/

#include <limits>

#include "../../lib/cxxopts.hpp"

#include "counter.hpp"
#include "pbformula.hpp"


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

  cxxopts::Options *options;

  void printOptionalOptions() const;
  void printHelp() const;
  void printWelcome() const;
  OptionDict(int argc, char *argv[]);
};

/* namespaces *****************************************************************/

namespace pbsolving
{
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
      PreprocessingConfig preprocessingConfig);
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
      Int preprocessingOption);
  void pbsolveCommand(int argc, char *argv[]);
}
/* global functions ***********************************************************/

int main(int argc, char *argv[]);