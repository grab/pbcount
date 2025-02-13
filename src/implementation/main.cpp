/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

/* inclusions *****************************************************************/

#include "../interface/main.hpp"

#include <string>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

/* classes ********************************************************************/

/* class OptionDict ***********************************************************/

void OptionDict::printOptionalOptions() const {
  cout << " Optional options:\n";
  util::printHelpOption();
  util::printCnfFileOption();
  util::printWeightFormatOption();
  // util::printJtFileOption();
  // util::printJtWaitOption();
  // util::printOutputFormatOption();
  util::printClusteringHeuristicOption();
  util::printCnfVarOrderingHeuristicOption();
  util::printDdVarOrderingHeuristicOption();
  util::printClauseCompilationHeuristicOption();
  util::printPreprocessingOption();
  util::printOperationModeOption();
  util::printInteractionAdaptiveRestartOption();
  util::printRandomSeedOption();
  util::printVerbosityLevelOption();
}

void OptionDict::printHelp() const {
  cout << options->help({
      REQUIRED_OPTION_GROUP,
      // OPTIONAL_OPTION_GROUP
  });
  printOptionalOptions();
}

void OptionDict::printWelcome() const {
  bool commented = !helpFlag;

  printThickLine(commented);

  printComment(
      "PBCount2: Pseudo Boolean Counter based on decision diagrams (help: "
      "'pbcount -h')",
      0, 1, commented);

  const string &addversion = "0.0";
  const string &version = "research paper code";
  const string &date = "0000/00/00";
  // printComment("Version " + version + ", released on " + date, 0, 1, commented);
  printComment("Version: " + version, 0, 1, commented);

  printThickLine(commented);
}

OptionDict::OptionDict(int argc, char *argv[]) {
  options = new cxxopts::Options("pbcount", "");

  options->add_options(OPTIONAL_OPTION_GROUP)
      (HELP_OPTION, "help")
      
      (PB_FILE_OPTION, "", cxxopts::value<string>()->default_value(STDIN_CONVENTION))
      
      (WEIGHT_FORMAT_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_WEIGHT_FORMAT_CHOICE)))
      
      (JT_FILE_OPTION, "", cxxopts::value<string>()->default_value(DUMMY_STR))
      
      (JT_WAIT_DURAION_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_JT_WAIT_SECONDS)))
      
      (OUTPUT_FORMAT_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_OUTPUT_FORMAT_CHOICE)))
      
      (CLUSTERING_HEURISTIC_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_CLUSTERING_HEURISTIC_CHOICE)))
      
      (CLUSTER_VAR_ORDER_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_CNF_VAR_ORDERING_HEURISTIC_CHOICE)))
      
      (DIAGRAM_VAR_ORDER_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_DD_VAR_ORDERING_HEURISTIC_CHOICE)))
      
      (CLAUSE_COMPILATION_HEURISTIC_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_CLAUSE_COMPILATION_HEURISTIC_CHOICE)))
      
      (PREPROCESSING_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_PREPROCESSING_CHOICE)))

      (OPERATION_MODE_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_OPERATION_MODE_CHOICE)))

      (INTERACTION_ADAPTIVE_RESTART_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_INTERACTIVE_ADAPTIVE_RESTART_CHOICE)))
      
      (RANDOM_SEED_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_RANDOM_SEED)))
      
      (VERBOSITY_LEVEL_OPTION, "", cxxopts::value<string>()->default_value(to_string(DEFAULT_VERBOSITY_LEVEL_CHOICE)));

  cxxopts::ParseResult result = options->parse(argc, argv);

  helpFlag = result["h"].as<bool>();

  printWelcome();

  cnfFilePath = result[PB_FILE_OPTION].as<string>();
  jtFilePath = result[JT_FILE_OPTION].as<string>();
  if (cnfFilePath == jtFilePath) {
    showError("options --" + PB_FILE_OPTION + " and --" + JT_FILE_OPTION +
                  " must have distinct args",
              !helpFlag);
  }

  weightFormatOption = std::stoll(result[WEIGHT_FORMAT_OPTION].as<string>());
  jtWaitSeconds = std::stod(result[JT_WAIT_DURAION_OPTION].as<string>());
  outputFormatOption = std::stoll(result[OUTPUT_FORMAT_OPTION].as<string>());
  clusteringHeuristicOption =
      std::stoll(result[CLUSTERING_HEURISTIC_OPTION].as<string>());
  cnfVarOrderingHeuristicOption =
      std::stoll(result[CLUSTER_VAR_ORDER_OPTION].as<string>());
  ddVarOrderingHeuristicOption =
      std::stoll(result[DIAGRAM_VAR_ORDER_OPTION].as<string>());
  clauseCompilationHeuristicOption = 
      std::stoll(result[CLAUSE_COMPILATION_HEURISTIC_OPTION].as<string>());
  preprocessingOption = 
      std::stoll(result[PREPROCESSING_OPTION].as<string>());
  operationModeOption = 
      std::stoll(result[OPERATION_MODE_OPTION].as<string>());
  interactionAdaptiveRestartOption =
      std::stoll(result[INTERACTION_ADAPTIVE_RESTART_OPTION].as<string>());
  randomSeedOption = std::stoll(result[RANDOM_SEED_OPTION].as<string>());
  verbosityLevelOption =
      std::stoll(result[VERBOSITY_LEVEL_OPTION].as<string>());
}

/* namespaces *****************************************************************/

/* namespace pbsolving
 * **********************************************************/

void pbsolving::pbsolveInteractive(const string &pbFilePath, WeightFormat weightFormat,
                            OutputFormat outputFormat, ClusteringHeuristic clusteringHeuristic,
                            VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering,
                            VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering,
                            ClauseCompilationHeuristic clauseCompilationHeuristic,
                            AdaptiveRestartChoice adaptiveRestartChoice) {
  Interactor userInteractor = Interactor();
  userInteractor.startInteraction(pbFilePath, weightFormat, outputFormat, clusteringHeuristic, pbVarOrderingHeuristic, inversePbVarOrdering, ddVarOrderingHeuristic, inverseDdVarOrdering, clauseCompilationHeuristic, adaptiveRestartChoice);
}

void pbsolving::pbsolveFile(const string &pbFilePath, WeightFormat weightFormat,
                            const string &jtFilePath, Float jtWaitSeconds,
                            OutputFormat outputFormat,
                            ClusteringHeuristic clusteringHeuristic,
                            VarOrderingHeuristic pbVarOrderingHeuristic,
                            bool inversePbVarOrdering,
                            VarOrderingHeuristic ddVarOrderingHeuristic,
                            bool inverseDdVarOrdering,
                            ClauseCompilationHeuristic clauseCompilationHeuristic,
                            PreprocessingConfig preprocessingConfig,
                            OperationModeChoice operationModeChoice,
                            AdaptiveRestartChoice adaptiveRestartChoice) {
  if (verbosityLevel >= 1) {
    printComment("Reading command-line options...", 1);

    /* required: */
    util::printRow("pbFilePath", pbFilePath);

    /* optional: */
    util::printRow("weightFormat", util::getWeightFormatName(weightFormat));
    util::printRow("jtFilePath", jtFilePath);
    util::printRow("jtWaitSeconds", jtWaitSeconds);
    util::printRow("outputFormat", util::getOutputFormatName(outputFormat));
    util::printRow("clustering",
                   util::getClusteringHeuristicName(clusteringHeuristic));
    util::printRow("clusterVarOrder",
                   util::getVarOrderingHeuristicName(pbVarOrderingHeuristic));
    util::printRow("inverseClusterVarOrder", inversePbVarOrdering);
    util::printRow("diagramVarOrder",
                   util::getVarOrderingHeuristicName(ddVarOrderingHeuristic));
    util::printRow("inverseDiagramVarOrder", inverseDdVarOrdering);
    util::printRow("clauseComplilationHeuristic",
                   util::getClauseCompilationHeuristicName(clauseCompilationHeuristic));
    util::printRow("preprocessingConfig", util::getPreprocessingConfigName(preprocessingConfig));
    util::printRow("operationMode", util::getOperationModeChoiceName(operationModeChoice));
    util::printRow("InteractionAdaptiveRestartMode", util::getInteractiveAdaptiveRestartChoiceName(adaptiveRestartChoice));
    util::printRow("randomSeed", randomSeed);
  }

  if (operationModeChoice == OperationModeChoice::Incremental) {
    pbsolveInteractive(pbFilePath, weightFormat, outputFormat, clusteringHeuristic, pbVarOrderingHeuristic, inversePbVarOrdering, ddVarOrderingHeuristic, inverseDdVarOrdering, clauseCompilationHeuristic,
    adaptiveRestartChoice);
    return;
  }

  if (outputFormat == OutputFormat::MODEL_COUNT && jtFilePath != DUMMY_STR) {
    throw MyError("Join tree mode not supported in this implementation.", true);
    return;
  }

  const std::map<WeightFormat, string> weightFormatNames = {
      {WeightFormat::UNWEIGHTED, "unweighted"},
      {WeightFormat::WEIGHTED, "weighted"}
  };
  std::cout << "weight format default: " << weightFormatNames.at(weightFormat)
            << std::endl;
  switch (clusteringHeuristic) {
    case ClusteringHeuristic::MONOLITHIC: {
      PBMonolithicCounter monolithicCounter(ddVarOrderingHeuristic,
                                            inverseDdVarOrdering,
                                            clauseCompilationHeuristic);
      monolithicCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    case ClusteringHeuristic::LINEAR: {
      PBLinearCounter linearCounter(ddVarOrderingHeuristic,
                                    inverseDdVarOrdering,
                                    clauseCompilationHeuristic);
      linearCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    case ClusteringHeuristic::BUCKET_LIST: {
      PBBucketCounter bucketCounter(
          false, pbVarOrderingHeuristic, inversePbVarOrdering,
          ddVarOrderingHeuristic, inverseDdVarOrdering,
          clauseCompilationHeuristic);
      bucketCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    case ClusteringHeuristic::BUCKET_TREE: {
      PBBucketCounter bucketCounter(
          true, pbVarOrderingHeuristic, inversePbVarOrdering,
          ddVarOrderingHeuristic, inverseDdVarOrdering,
          clauseCompilationHeuristic);
      bucketCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    case ClusteringHeuristic::BOUQUET_LIST: {
      PBBouquetCounter bouquetCounter(
          false, pbVarOrderingHeuristic, inversePbVarOrdering,
          ddVarOrderingHeuristic, inverseDdVarOrdering,
          clauseCompilationHeuristic);
      bouquetCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    case ClusteringHeuristic::BOUQUET_TREE: {
      PBBouquetCounter bouquetCounter(
          true, pbVarOrderingHeuristic, inversePbVarOrdering,
          ddVarOrderingHeuristic, inverseDdVarOrdering,
          clauseCompilationHeuristic);
      bouquetCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    case ClusteringHeuristic::COMPUTE_GRAPH_MIN_DEGREE: {
      PBComputeGraphCounter computeGraphCounter(
          pbVarOrderingHeuristic, inversePbVarOrdering,
          ddVarOrderingHeuristic, inverseDdVarOrdering,
          clauseCompilationHeuristic);
      computeGraphCounter.output(pbFilePath, weightFormat, outputFormat, preprocessingConfig);
      break;
    }
    default: {
      showError("no such clusteringHeuristic");
    }
  }
}

void pbsolving::pbsolveOptions(const string &pbFilePath, 
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
                               Int interactionAdaptiveRestartMode) {
  WeightFormat weightFormat;
  try {
    weightFormat = WEIGHT_FORMAT_CHOICES.at(weightFormatOption);
  } catch (const std::out_of_range &) {
    showError("no such weightFormatOption: " + to_string(weightFormatOption));
  }

  OutputFormat outputFormat;
  try {
    outputFormat = OUTPUT_FORMAT_CHOICES.at(outputFormatOption);
  } catch (const std::out_of_range &) {
    showError("no such outputFormatOption: " + to_string(outputFormatOption));
  }

  ClusteringHeuristic clusteringHeuristic;
  try {
    clusteringHeuristic =
        CLUSTERING_HEURISTIC_CHOICES.at(clusteringHeuristicOption);
  } catch (const std::out_of_range &) {
    showError("no such clusteringHeuristicOption: " +
              to_string(clusteringHeuristicOption));
  }

  VarOrderingHeuristic pbVarOrderingHeuristic;
  bool inverseCnfVarOrdering;
  try {
    pbVarOrderingHeuristic = VAR_ORDERING_HEURISTIC_CHOICES.at(
        std::abs(pbVarOrderingHeuristicOption));
    inverseCnfVarOrdering = pbVarOrderingHeuristicOption < 0;
  } catch (const std::out_of_range &) {
    showError("no such pbVarOrderingHeuristicOption: " +
              to_string(pbVarOrderingHeuristicOption));
  }

  VarOrderingHeuristic ddVarOrderingHeuristic;
  bool inverseDdVarOrdering;
  try {
    ddVarOrderingHeuristic = VAR_ORDERING_HEURISTIC_CHOICES.at(
        std::abs(ddVarOrderingHeuristicOption));
    inverseDdVarOrdering = ddVarOrderingHeuristicOption < 0;
  } catch (const std::out_of_range &) {
    showError("no such ddVarOrderingHeuristicOption: " +
              to_string(ddVarOrderingHeuristicOption));
  }

  ClauseCompilationHeuristic clauseCompilationHeuristic;
  try {
    clauseCompilationHeuristic = CLAUSE_COMPILATION_HEURISTIC_CHOICES.at(
        std::abs(clauseCompilationHeuristicOption));
  } catch (const std::out_of_range &) {
    showError("no such clauseCompilationHeuristicOption: " +
              to_string(clauseCompilationHeuristicOption));
  }
  PreprocessingConfig preprocessingConfig;
  try {
    preprocessingConfig =
        PREPROCESSING_CHOICES.at(preprocessingOption);
  } catch (const std::out_of_range &) {
    showError("no such PrepropocessingConfig: " +
              to_string(preprocessingOption));
  }
  OperationModeChoice operationModeChoice;
  try {
    operationModeChoice =
        OPERATION_MODES.at(operationMode);
  } catch (const std::out_of_range &) {
    showError("no such OperationModeChoice: " +
              to_string(operationMode));
  }
  AdaptiveRestartChoice adaptiveRestartChoice;
  try {
    adaptiveRestartChoice =
        INTERACTIVE_ADAPTIVE_RESTART_CHOICES.at(interactionAdaptiveRestartMode);
  } catch (const std::out_of_range &) {
    showError("no such adaptiveRestartChoice: " +
              to_string(interactionAdaptiveRestartMode));
  }

  pbsolveFile(pbFilePath, weightFormat, jtFilePath, jtWaitSeconds, outputFormat,
              clusteringHeuristic, pbVarOrderingHeuristic,
              inverseCnfVarOrdering, ddVarOrderingHeuristic,
              inverseDdVarOrdering, clauseCompilationHeuristic,
              preprocessingConfig, operationModeChoice, 
              adaptiveRestartChoice);
}

void pbsolving::pbsolveCommand(int argc, char *argv[]) {
  OptionDict optionDict(argc, argv);

  randomSeed = optionDict.randomSeedOption;          // global variable
  verbosityLevel = optionDict.verbosityLevelOption;  // global variable
  startTime = util::getTimePoint();                  // global variable

  if (optionDict.helpFlag) {
    optionDict.printHelp();
  } else {
    printComment("Process ID of this main program:", 1);
    printComment("pid " + to_string(getpid()));

    pbsolveOptions(optionDict.cnfFilePath, optionDict.weightFormatOption,
                   optionDict.jtFilePath, optionDict.jtWaitSeconds,
                   optionDict.outputFormatOption,
                   optionDict.clusteringHeuristicOption,
                   optionDict.cnfVarOrderingHeuristicOption,
                   optionDict.ddVarOrderingHeuristicOption,
                   optionDict.clauseCompilationHeuristicOption,
                   optionDict.preprocessingOption,
                   optionDict.operationModeOption,
                   optionDict.interactionAdaptiveRestartOption);
    cout << "\n";

    util::printDuration(startTime);
  }
}

/* global functions ***********************************************************/

int main(int argc, char *argv[]) {
  pbsolving::pbsolveCommand(argc, argv);
  return 0;
}