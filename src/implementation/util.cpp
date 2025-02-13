/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

/* inclusions *****************************************************************/

#include "../interface/util.hpp"

/* global variables ***********************************************************/

Int randomSeed = DEFAULT_RANDOM_SEED;
Int verbosityLevel = DEFAULT_VERBOSITY_LEVEL_CHOICE;
TimePoint startTime;

/* constants ******************************************************************/

const string &COMMENT_WORD = "c";
const string &PROBLEM_WORD = "p";

const string &STDIN_CONVENTION = "-";

const string &REQUIRED_OPTION_GROUP = "Required";
const string &OPTIONAL_OPTION_GROUP = "Optional";

const string &HELP_OPTION = "h, hi";
const string &PB_FILE_OPTION = "cf";
const string &WEIGHT_FORMAT_OPTION = "wf";
const string &JT_FILE_OPTION = "jf";
const string &JT_WAIT_DURAION_OPTION = "jw";
const string &OUTPUT_FORMAT_OPTION = "of";
const string &CLUSTERING_HEURISTIC_OPTION = "ch";
const string &CLUSTER_VAR_ORDER_OPTION = "cv";
const string &DIAGRAM_VAR_ORDER_OPTION = "dv";
const string &RANDOM_SEED_OPTION = "rs";
const string &VERBOSITY_LEVEL_OPTION = "vl";
const string &CLAUSE_COMPILATION_HEURISTIC_OPTION = "cc";
const string &PREPROCESSING_OPTION = "pp";
const string &OPERATION_MODE_OPTION = "om";
const string &INTERACTION_ADAPTIVE_RESTART_OPTION = "re";

const std::map<Int, ClauseCompilationHeuristic> CLAUSE_COMPILATION_HEURISTIC_CHOICES = {
  {1, ClauseCompilationHeuristic::BOTTOMUP},
  {2, ClauseCompilationHeuristic::TOPDOWN},
  {3, ClauseCompilationHeuristic::DYNAMIC},
  {4, ClauseCompilationHeuristic::OPT_BOTTOMUP},
  {5, ClauseCompilationHeuristic::OPT_TOPDOWN}
};
const Int DEFAULT_CLAUSE_COMPILATION_HEURISTIC_CHOICE = 3;

const std::map<Int, PreprocessingConfig> PREPROCESSING_CHOICES = {
  {1, PreprocessingConfig::OFF},
  {2, PreprocessingConfig::ALL}
};
const Int DEFAULT_PREPROCESSING_CHOICE = 2;

const std::map<Int, OperationModeChoice> OPERATION_MODES = {
  {1, OperationModeChoice::Single},
  {2, OperationModeChoice::Incremental}
};
const Int DEFAULT_OPERATION_MODE_CHOICE = 1;

const std::map<Int, AdaptiveRestartChoice> INTERACTIVE_ADAPTIVE_RESTART_CHOICES = {
  {1, AdaptiveRestartChoice::ON},
  {2, AdaptiveRestartChoice::OFF}
};
const Int DEFAULT_INTERACTIVE_ADAPTIVE_RESTART_CHOICE = 2;

const std::map<Int, WeightFormat> WEIGHT_FORMAT_CHOICES = {
  {1, WeightFormat::UNWEIGHTED},
  {2, WeightFormat::WEIGHTED}
};
const Int DEFAULT_WEIGHT_FORMAT_CHOICE = 1;

const Float DEFAULT_JT_WAIT_SECONDS = 10.0;

const std::map<Int, OutputFormat> OUTPUT_FORMAT_CHOICES = {
  {1, OutputFormat::JOIN_TREE},
  {2, OutputFormat::MODEL_COUNT}
};
const Int DEFAULT_OUTPUT_FORMAT_CHOICE = 2;

const std::map<Int, ClusteringHeuristic> CLUSTERING_HEURISTIC_CHOICES = {
  {1, ClusteringHeuristic::MONOLITHIC},
  {2, ClusteringHeuristic::LINEAR},
  {3, ClusteringHeuristic::BUCKET_LIST},
  {4, ClusteringHeuristic::BUCKET_TREE},
  {5, ClusteringHeuristic::BOUQUET_LIST},
  {6, ClusteringHeuristic::BOUQUET_TREE},
  {7, ClusteringHeuristic::COMPUTE_GRAPH_MIN_DEGREE}
};
const Int DEFAULT_CLUSTERING_HEURISTIC_CHOICE = 7;

const std::map<Int, VarOrderingHeuristic> VAR_ORDERING_HEURISTIC_CHOICES = {
  {1, VarOrderingHeuristic::APPEARANCE},
  {2, VarOrderingHeuristic::DECLARATION},
  {3, VarOrderingHeuristic::RANDOM},
  {4, VarOrderingHeuristic::MCS},
  {5, VarOrderingHeuristic::LEXP},
  {6, VarOrderingHeuristic::LEXM},
  {7, VarOrderingHeuristic::MINFILL}
};
const Int DEFAULT_CNF_VAR_ORDERING_HEURISTIC_CHOICE = 5;
const Int DEFAULT_DD_VAR_ORDERING_HEURISTIC_CHOICE = 4;

const Int DEFAULT_RANDOM_SEED = 10;

const vector<Int> VERBOSITY_LEVEL_CHOICES = {0, 1, 2, 3, 4};
const Int DEFAULT_VERBOSITY_LEVEL_CHOICE = 0;

const Float NEGATIVE_INFINITY = -std::numeric_limits<Float>::infinity();

const Int DUMMY_MIN_INT = std::numeric_limits<Int>::min();
const Int DUMMY_MAX_INT = std::numeric_limits<Int>::max();

const string &DUMMY_STR = "";

const string &DOT_DIR = "./";

const string &PROJECTED_COUNTING_UNSUPPORTED_STR = "Projected model counting is unsupported on this clustering heuristic at the moment, please run with monolithic clustering heuristic (--ch 1).";

/* namespaces *****************************************************************/

/* namespace util *************************************************************/

bool util::isInt(Float d) {
  Float intPart;
  Float fractionalPart = modf(d, &intPart);
  return fractionalPart == 0.0;
}

/* functions: printing ********************************************************/

void util::printComment(const string &message, Int preceedingNewLines, Int followingNewLines, bool commented) {
  for (Int i = 0; i < preceedingNewLines; i++) cout << "\n";
  cout << (commented ? COMMENT_WORD + " " : "") << message;
  for (Int i = 0; i < followingNewLines; i++) cout << "\n";
}

void util::printSolutionLine(WeightFormat weightFormat, Float modelCount, Int preceedingThinLines, Int followingThinLines) {
  for (Int i = 0; i < preceedingThinLines; i++) printThinLine();
  cout << "s " << (weightFormat == WeightFormat::UNWEIGHTED ? "mc" : "wmc") << " " << to_string(modelCount) << "\n";
  for (Int i = 0; i < followingThinLines; i++) printThinLine();
}

void util::printBoldLine(bool commented) {
  printComment("******************************************************************", 0, 1, commented);
}

void util::printThickLine(bool commented) {
  printComment("==================================================================", 0, 1, commented);
}

void util::printThinLine() {
  printComment("------------------------------------------------------------------");
}

void util::printHelpOption() {
  cout << "  -h, --hi      help information\n";
}

void util::printCnfFileOption() {
  cout << "      --" << PB_FILE_OPTION << std::left << std::setw(56) << " arg  pb file path (to use stdin, type: '--" + PB_FILE_OPTION + " -')";
  cout << "Default: -\n";
}

void util::printWeightFormatOption() {
  cout << "      --" << WEIGHT_FORMAT_OPTION << " arg  ";
  cout << "weight format in input file:\n";
  for (const auto &kv : WEIGHT_FORMAT_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getWeightFormatName(kv.second);
    if (num == DEFAULT_WEIGHT_FORMAT_CHOICE) cout << "Default: " << DEFAULT_WEIGHT_FORMAT_CHOICE;
    cout << "\n";
  }
}

void util::printJtFileOption() {
  cout << "      --" << JT_FILE_OPTION << std::left << std::setw(56) << " arg  jt file path (to use stdin, type: '--" + JT_FILE_OPTION + " -')";
  cout << "Default: (no jt file)\n";
}

void util::printJtWaitOption() {
  cout << "      --" << JT_WAIT_DURAION_OPTION << std::left << std::setw(56) << " arg  jt wait duration before tree builder is killed";
  cout << "Default: " + to_string(DEFAULT_JT_WAIT_SECONDS) + " (seconds)\n";
}

void util::printOutputFormatOption() {
  cout << "      --" << OUTPUT_FORMAT_OPTION << " arg  ";
  cout << "output format:\n";
  for (const auto &kv : OUTPUT_FORMAT_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << getOutputFormatName(kv.second);
    if (num == DEFAULT_OUTPUT_FORMAT_CHOICE) {
      cout << std::left << std::setw(39) << " (using input jt file if provided)";
      cout << "Default: " << DEFAULT_OUTPUT_FORMAT_CHOICE;
    }
    cout << "\n";
  }
}

void util::printClusteringHeuristicOption() {
  cout << "      --" << CLUSTERING_HEURISTIC_OPTION << " arg  ";
  cout << "clustering heuristic:\n";
  for (const auto &kv : CLUSTERING_HEURISTIC_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getClusteringHeuristicName(kv.second);
    if (num == DEFAULT_CLUSTERING_HEURISTIC_CHOICE) cout << "Default: " << DEFAULT_CLUSTERING_HEURISTIC_CHOICE;
    cout << "\n";
  }
}

void util::printCnfVarOrderingHeuristicOption() {
  cout << "      --" << CLUSTER_VAR_ORDER_OPTION << " arg  ";
  cout << "cluster variable order heuristic (negate to invert):\n";
  for (const auto &kv : VAR_ORDERING_HEURISTIC_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getVarOrderingHeuristicName(kv.second);
    if (num == std::abs(DEFAULT_CNF_VAR_ORDERING_HEURISTIC_CHOICE)) cout << "Default: " << DEFAULT_CNF_VAR_ORDERING_HEURISTIC_CHOICE;
    cout << "\n";
  }
}

void util::printDdVarOrderingHeuristicOption() {
  cout << "      --" << DIAGRAM_VAR_ORDER_OPTION << " arg  ";
  cout << "diagram variable order heuristic (negate to invert):\n";
  for (const auto &kv : VAR_ORDERING_HEURISTIC_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getVarOrderingHeuristicName(kv.second);
    if (num == std::abs(DEFAULT_DD_VAR_ORDERING_HEURISTIC_CHOICE)) cout << "Default: " << DEFAULT_DD_VAR_ORDERING_HEURISTIC_CHOICE;
    cout << "\n";
  }
}

void util::printClauseCompilationHeuristicOption() {
  cout << "      --" << CLAUSE_COMPILATION_HEURISTIC_OPTION << " arg  ";
  cout << "pb clause compilation heuristic:\n";
  for (const auto &kv : CLAUSE_COMPILATION_HEURISTIC_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getClauseCompilationHeuristicName(kv.second);
    if (num == std::abs(DEFAULT_CLAUSE_COMPILATION_HEURISTIC_CHOICE)) cout << "Default: " << DEFAULT_CLAUSE_COMPILATION_HEURISTIC_CHOICE;
    cout << "\n";
  }
}

void util::printPreprocessingOption() {
  cout << "      --" << PREPROCESSING_OPTION << " arg  ";
  cout << "preprocessing options:\n";
  for (const auto &kv : PREPROCESSING_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getPreprocessingConfigName(kv.second);
    if (num == std::abs(DEFAULT_PREPROCESSING_CHOICE)) cout << "Default: " << DEFAULT_PREPROCESSING_CHOICE;
    cout << "\n";
  }
}

void util::printOperationModeOption() {
  cout << "      --" << OPERATION_MODE_OPTION << " arg  ";
  cout << "preprocessing options:\n";
  for (const auto &kv : OPERATION_MODES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getOperationModeChoiceName(kv.second);
    if (num == std::abs(DEFAULT_OPERATION_MODE_CHOICE)) cout << "Default: " << DEFAULT_OPERATION_MODE_CHOICE;
    cout << "\n";
  }
}

void util::printInteractionAdaptiveRestartOption() {
  cout << "      --" << INTERACTION_ADAPTIVE_RESTART_OPTION << " arg  ";
  cout << "adaptive restart options:\n";
  for (const auto &kv : INTERACTIVE_ADAPTIVE_RESTART_CHOICES) {
    int num = kv.first;
    cout << "           " << num << "    " << std::left << std::setw(50) << getInteractiveAdaptiveRestartChoiceName(kv.second);
    if (num == std::abs(DEFAULT_OPERATION_MODE_CHOICE)) cout << "Default: " << DEFAULT_INTERACTIVE_ADAPTIVE_RESTART_CHOICE;
    cout << "\n";
  }
}

void util::printRandomSeedOption() {
  cout << "      --" << RANDOM_SEED_OPTION << std::left << std::setw(56) << " arg  random seed";
  cout << "Default: " + to_string(DEFAULT_RANDOM_SEED) + "\n";
}

void util::printVerbosityLevelOption() {
  cout << "      --" << VERBOSITY_LEVEL_OPTION << " arg  ";
  cout << "verbosity level:\n";
  for (Int verbosityLevelOption : VERBOSITY_LEVEL_CHOICES) {
    cout << "           " << verbosityLevelOption << "    " << std::left << std::setw(50) << getVerbosityLevelName(verbosityLevelOption);
    if (verbosityLevelOption == DEFAULT_VERBOSITY_LEVEL_CHOICE) cout << "Default: " << DEFAULT_VERBOSITY_LEVEL_CHOICE;
    cout << "\n";
  }
}

/* functions: argument parsing ************************************************/

vector<string> util::getArgV(int argc, char *argv[]) {
  vector<string> argV;
  for (Int i = 0; i < argc; i++) argV.push_back(string(argv[i]));
  return argV;
}

string util::getWeightFormatName(WeightFormat weightFormat) {
  switch (weightFormat) {
    case WeightFormat::UNWEIGHTED: {
      return "UNWEIGHTED";
    }
    case WeightFormat::WEIGHTED: {
      return "WEIGHTED";
    }
    case WeightFormat::MINIC2D: {
      return "MINIC2D";
    }
    case WeightFormat::CACHET: {
      return "CACHET";
    }
    case WeightFormat::MCC: {
      return "MCC";
    }
    default: {
      showError("no such weightFormat");
      return DUMMY_STR;
    }
  }
}

string util::getOutputFormatName(OutputFormat outputFormat) {
  switch (outputFormat) {
    case OutputFormat::JOIN_TREE: {
      return "JOIN_TREE";
    }
    case OutputFormat::MODEL_COUNT: {
      return "MODEL_COUNT";
    }
    default: {
      showError("no such outputFormat");
      return DUMMY_STR;
    }
  }
}

string util::getClusteringHeuristicName(ClusteringHeuristic clusteringHeuristic) {
  switch (clusteringHeuristic) {
    case ClusteringHeuristic::MONOLITHIC: {
      return "MONOLITHIC";
    }
    case ClusteringHeuristic::LINEAR: {
      return "LINEAR";
    }
    case ClusteringHeuristic::BUCKET_LIST: {
      return "BUCKET_LIST";
    }
    case ClusteringHeuristic::BUCKET_TREE: {
      return "BUCKET_TREE";
    }
    case ClusteringHeuristic::BOUQUET_LIST: {
      return "BOUQUET_LIST";
    }
    case ClusteringHeuristic::BOUQUET_TREE: {
      return "BOUQUET_TREE";
    }
    case ClusteringHeuristic::COMPUTE_GRAPH_MIN_DEGREE: {
      return "COMPUTE_GRAPH_MIN_DEGREE";
    }
    default: {
      showError("no such clusteringHeuristic");
      return DUMMY_STR;
    }
  }
}

string util::getVarOrderingHeuristicName(VarOrderingHeuristic varOrderingHeuristic) {
  switch (varOrderingHeuristic) {
    case VarOrderingHeuristic::APPEARANCE: {
      return "APPEARANCE";
    }
    case VarOrderingHeuristic::DECLARATION: {
      return "DECLARATION";
    }
    case VarOrderingHeuristic::RANDOM: {
      return "RANDOM";
    }
    case VarOrderingHeuristic::LEXP: {
      return "LEXP";
    }
    case VarOrderingHeuristic::LEXM: {
      return "LEXM";
    }
    case VarOrderingHeuristic::MCS: {
      return "MCS";
    }
    case VarOrderingHeuristic::MINFILL: {
      return "MINFILL";
    }
    default: {
      showError("DUMMY_VAR_ORDERING_HEURISTIC in util::getVarOrderingHeuristicName");
      return DUMMY_STR;
    }
  }
}

string util::getClauseCompilationHeuristicName(ClauseCompilationHeuristic clauseCompilationHeuristic) {
  switch (clauseCompilationHeuristic) {
    case ClauseCompilationHeuristic::BOTTOMUP: {
      return "BOTTOMUP";
    }
    case ClauseCompilationHeuristic::TOPDOWN: {
      return "TOPDOWN";
    }
    case ClauseCompilationHeuristic::DYNAMIC: {
      return "DYNAMIC";
    }
    case ClauseCompilationHeuristic::OPT_BOTTOMUP: {
      return "OPTIMIZED_BOTTOMUP";
    }
    case ClauseCompilationHeuristic::OPT_TOPDOWN: {
      return "OPTIMIZED_TOPDOWN";
    }
    default: {
      showError("DUMMY_CLAUSE_COMPILATION_HEURISTIC in util::getClauseCompilationHeuristicName");
      return DUMMY_STR;
    }
  }
}

string util::getPreprocessingConfigName(PreprocessingConfig preprocessingConfig) {
  switch (preprocessingConfig) {
    case PreprocessingConfig::OFF: {
      return "OFF, No Preprocessing";
    }
    case PreprocessingConfig::ALL: {
      return "ALL";
    }
    default: {
      showError("DUMMY_PREPROCESSING_OPTION in util::getPreprocessingConfigName");
      return DUMMY_STR;
    }
  }
}

string util::getOperationModeChoiceName(OperationModeChoice operationMode) {
  switch (operationMode) {
    case OperationModeChoice::Single: {
      return "Single, one time model count";
    }
    case OperationModeChoice::Incremental: {
      return "Interactive mode, for incremental counting";
    }
    default: {
      showError("DUMMY_OPERATION_MODE_OPTION in util::getOperationModeChoiceName");
      return DUMMY_STR;
    }
  }
}

string util::getInteractiveAdaptiveRestartChoiceName(AdaptiveRestartChoice restartMode) {
  switch (restartMode) {
    case AdaptiveRestartChoice::ON: {
      return "ENABLE Adaptive restart";
    }
    case AdaptiveRestartChoice::OFF: {
      return "DISABLE Adaptive restart";
    }
    default: {
      showError("DUMMY_INTERACTION_ADAPTIVE_RESTART_CHOICE in util::getInteractiveAdaptiveRestartChoiceName");
      return DUMMY_STR;
    }
  }
}

string util::getVerbosityLevelName(Int verbosityLevel) {
  switch (verbosityLevel) {
    case 0: {
      return "solution only";
    }
    case 1: {
      return "parsed info as well";
    }
    case 2: {
      return "clusters as well";
    }
    case 3: {
      return "pb literal weights as well";
    }
    case 4: {
      return "input lines as well";
    }
    default: {
      showError("no such verbosityLevel");
      return DUMMY_STR;
    }
  }
}

/* functions: CNF *************************************************************/

Int util::getCnfVar(Int literal) {
  if (literal == 0) {
    showError("literal is 0");
  }
  return std::abs(literal);
}

Set<Int> util::getClauseCnfVars(const vector<Int> &clause) {
  Set<Int> cnfVars;
  for (Int literal : clause) cnfVars.insert(getCnfVar(literal));
  return cnfVars;
}

Set<Int> util::getClusterCnfVars(const vector<Int> &cluster, const vector<vector<Int>> &clauses) {
  Set<Int> cnfVars;
  for (Int clauseIndex : cluster) unionize(cnfVars, getClauseCnfVars(clauses.at(clauseIndex)));
  return cnfVars;
}

bool util::appearsIn(Int cnfVar, const vector<Int> &clause) {
  for (Int literal : clause) if (getCnfVar(literal) == cnfVar) return true;
  return false;
}

bool util::isPositiveLiteral(Int literal) {
  if (literal == 0) showError("literal is 0");
  return literal > 0;
}

Int util::getLiteralRank(Int literal, const vector<Int> &cnfVarOrdering) {
  Int cnfVar = getCnfVar(literal);
  auto it = std::find(cnfVarOrdering.begin(), cnfVarOrdering.end(), cnfVar);
  if (it == cnfVarOrdering.end()) showError("cnfVar not found in cnfVarOrdering");
  return it - cnfVarOrdering.begin();
}

Int util::getMinClauseRank(const vector<Int> &clause, const vector<Int> &cnfVarOrdering) {
  Int minRank = DUMMY_MAX_INT;
  for (Int literal : clause) {
    Int rank = getLiteralRank(literal, cnfVarOrdering);
    if (rank < minRank) minRank = rank;
  }
  return minRank;
}

Int util::getMaxClauseRank(const vector<Int> &clause, const vector<Int> &cnfVarOrdering) {
  Int maxRank = DUMMY_MIN_INT;
  for (Int literal : clause) {
    Int rank = getLiteralRank(literal, cnfVarOrdering);
    if (rank > maxRank) maxRank = rank;
  }
  return maxRank;
}

void util::printClause(const vector<Int> &clause) {
  for (Int literal : clause) {
    cout << std::right << std::setw(5) << literal << " ";
  }
  cout << "\n";
}

void util::printCnf(const vector<vector<Int>> &clauses) {
  printThinLine();
  printComment("cnf {");
  for (Int i = 0; i < clauses.size(); i++) {
    cout << COMMENT_WORD << "\t" "clause ";
    cout << std::right << std::setw(5) << i + 1 << " : ";
    printClause(clauses.at(i));
  }
  printComment("}");
  printThinLine();
}

void util::printLiteralWeights(const Map<Int, Float> &literalWeights) {
  Int maxCnfVar = DUMMY_MIN_INT;
  for (const std::pair<Int, Float> &kv : literalWeights) {
    Int cnfVar = kv.first;
    if (cnfVar > maxCnfVar) {
      maxCnfVar = cnfVar;
    }
  }

  printThinLine();
  printComment("literalWeights {");
  cout << std::right;
  for (Int cnfVar = 1; cnfVar <= maxCnfVar; cnfVar++) {
    cout << COMMENT_WORD << " " << std::right << std::setw(10) << cnfVar << "\t" << std::left << std::setw(15) << literalWeights.at(cnfVar) << "\n";
    cout << COMMENT_WORD << " " << std::right << std::setw(10) << -cnfVar << "\t" << std::left << std::setw(15) << literalWeights.at(-cnfVar) << "\n";
  }
  printComment("}");
  printThinLine();
}

/* functions: pb (sharing some function with cnf) *****************************/

Int util::getPbVar(Int literal) {
  if (literal == 0) {
    showError("literal is 0");
  }
  return std::abs(literal);
}

Set<Int> util::getClausePbVars(const PBclause &clause) {
  Set<Int> pbVars;
  for (Int literal : clause.lits) pbVars.insert(getPbVar(literal));
  return pbVars;
}

Set<Int> util::getClusterPbVars(const vector<Int> &cluster, const vector<PBclause> &clauses) {
  Set<Int> pbVars;
  for (Int clauseIndex : cluster) unionize(pbVars, getClausePbVars(clauses.at(clauseIndex)));
  return pbVars;
}

bool util::appearsIn(Int pbVar, const PBclause &clause) {
  for (Int literal : clause.lits) if (getPbVar(literal) == pbVar) return true;
  return false;
}

Int util::getMinClauseRank(const PBclause &clause, const vector<Int> &pbVarOrdering) {
  Int minRank = DUMMY_MAX_INT;
  for (Int literal : clause.lits) {
    Int rank = getLiteralRank(literal, pbVarOrdering);
    if (rank < minRank) minRank = rank;
  }
  return minRank;
}

Int util::getMaxClauseRank(const PBclause &clause, const vector<Int> &pbVarOrdering) {
  Int maxRank = DUMMY_MIN_INT;
  for (Int literal : clause.lits) {
    Int rank = getLiteralRank(literal, pbVarOrdering);
    if (rank > maxRank) maxRank = rank;
  }
  return maxRank;
}

void util::printClause(const PBclause &clause) {
  cout << std::right << std::setw(5);
  for (Int i = 0; i < clause.lits.size(); i++) {
    cout << clause.coeffs.at(i) << ' ';
    cout << "x" << clause.lits.at(i) << ' ';
  }
  if (clause.equals) {
    cout << " = ";
  } else {
    cout << " >= ";
  }
  cout << clause.clauseConsVal << std::endl;
}

void util::printPb(const vector<PBclause> &clauses) {
  printThinLine();
  printComment("pb {");
  for (Int i = 0; i < clauses.size(); i++) {
    cout << COMMENT_WORD << "\t" "clause ";
    cout << std::right << std::setw(5) << i + 1 << " : ";
    printClause(clauses.at(i));
  }
  printComment("}");
  printThinLine();
}

/* functions: pb compute graph counter *****************************/

/* model counting adjustment for compute graph based counters with preprocessing (inferred assignments)*/
Float util::adjustModelCountCG(Float apparentModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments) {
  /*
  This version is to handle counting with preprocessing.
  Unit clauses are removed in preprocessing and we want to account for
  the situation where x1>=1 (only 1 satisfying assignment) but x1 does not appear elsewhere so not counted. (we cannot assume both true and false satisfy now)
  */
  Float totalModelCount = apparentModelCount;

  Int totalLiteralCount = literalWeights.size();
  if (totalLiteralCount % 2 == 1) showError("odd total literal count");

  for (auto &assignment : inferredAssignments) {
    double posWeight = 1.0;
    double negWeight = 1.0;
    Int var = assignment.first;

    if (literalWeights.find(var) != literalWeights.end()){
      posWeight = literalWeights.at(var);
    }
    if (literalWeights.find(-var) != literalWeights.end()){
      negWeight = literalWeights.at(-var);
    }
    if (assignment.second) {
      // inferred assignment is true
      totalModelCount *= posWeight;
    } else {
      totalModelCount *= negWeight;
    }
  }

  if (totalModelCount == 0) {
    showWarning("floating-point underflow may have occured or unsat formula");
  }
  return totalModelCount;
}

Float util::adjustModelCountCGMissingVar (Float totalModelCount, Int parsedVarNum, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &processedVariables) {
  Float adjustedModelCount = totalModelCount;
  bool unappearedVar = false;
  Int totalVarNum = parsedVarNum;
  for (Int pbVar = 1; pbVar <= totalVarNum; pbVar++) {
    if (!isFound(pbVar, processedVariables)) {
      if (inferredAssignments.find(pbVar) == inferredAssignments.end()) {
        unappearedVar = true;
        Float posWeight = 1.0;
        Float negWeight = 1.0;
        if (literalWeights.find(pbVar) != literalWeights.end()){
          posWeight = literalWeights.at(pbVar);
        }
        if (literalWeights.find(-pbVar) != literalWeights.end()){
          negWeight = literalWeights.at(-pbVar);
        }
        adjustedModelCount *= posWeight + negWeight;
      }
    }
  }
  if (unappearedVar) {
    std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
  }
  return adjustedModelCount;
}

Float util::adjustProjectedModelCountCG(Float apparentModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &projectionSupportVarSet) {
  /*
  This version is to handle counting with preprocessing.
  Unit clauses are removed in preprocessing and we want to account for
  the situation where x1>=1 (only 1 satisfying assignment) but x1 does not appear elsewhere so not counted. (we cannot assume both true and false satisfy now)
  */
  Float totalModelCount = apparentModelCount;

  Int totalLiteralCount = literalWeights.size();
  if (totalLiteralCount % 2 == 1) showError("odd total literal count");

  for (auto &assignment : inferredAssignments) {
    double posWeight = 1.0;
    double negWeight = 1.0;
    Int var = assignment.first;
    // if not in projection set, just or abstract, so do not need to care about weight
    if (!isFound(var, projectionSupportVarSet)) { continue; }

    if (literalWeights.find(var) != literalWeights.end()){
      posWeight = literalWeights.at(var);
    }
    if (literalWeights.find(-var) != literalWeights.end()){
      negWeight = literalWeights.at(-var);
    }
    if (assignment.second) {
      // inferred assignment is true
      totalModelCount *= posWeight;
    } else {
      totalModelCount *= negWeight;
    }
  }

  if (totalModelCount == 0) {
    showWarning("floating-point underflow may have occured or unsat formula");
  }
  return totalModelCount;
}

Float util::adjustProjectedModelCountCGMissingVar (Float totalModelCount, Int parsedVarNum, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &processedVariables, const Set<Int> &projectionSupportVarSet) {
  Float adjustedModelCount = totalModelCount;
  bool unappearedVar = false;
  Int totalVarNum = parsedVarNum;
  for (Int pbVar = 1; pbVar <= totalVarNum; pbVar++) {
    if (!isFound(pbVar, processedVariables)) {
      if (!isFound(pbVar, projectionSupportVarSet)) { continue; }
      if (inferredAssignments.find(pbVar) == inferredAssignments.end()) {
        unappearedVar = true;
        Float posWeight = 1.0;
        Float negWeight = 1.0;
        if (literalWeights.find(pbVar) != literalWeights.end()){
          posWeight = literalWeights.at(pbVar);
        }
        if (literalWeights.find(-pbVar) != literalWeights.end()){
          negWeight = literalWeights.at(-pbVar);
        }
        adjustedModelCount *= posWeight + negWeight;
      }
    }
  }
  if (unappearedVar) {
    std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
  }
  return adjustedModelCount;
}

Float util::adjustInteractiveProjectedModelCountCG(Float apparentModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &projectionSupportVarSet, Set<Int> &activeVarSet) {
  Float totalModelCount = apparentModelCount;

  for (auto &assignment : inferredAssignments) {
    double posWeight = 1.0;
    double negWeight = 1.0;
    Int var = assignment.first;

    // if not in active variables, ignore
    if (!isFound(var, activeVarSet)) { continue; }
    // if not in projection set, just or abstract, so do not need to care about weight
    if (!isFound(var, projectionSupportVarSet)) { continue; }

    if (literalWeights.find(var) != literalWeights.end()){
      posWeight = literalWeights.at(var);
    }
    if (literalWeights.find(-var) != literalWeights.end()){
      negWeight = literalWeights.at(-var);
    }
    if (assignment.second) {
      // inferred assignment is true
      totalModelCount *= posWeight;
    } else {
      totalModelCount *= negWeight;
    }
  }

  Int totalLiteralCount = literalWeights.size();
  if (totalLiteralCount % 2 == 1) showError("odd total literal count");

  if (totalModelCount == 0) {
    showWarning("floating-point underflow may have occured or unsat formula");
  }
  return totalModelCount;
}

Float util::adjustInteractiveProjectedModelCountCGMissingVar(Float totalModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &processedVariables, const Set<Int> &projectionSupportVarSet, Set<Int> &activeVarSet) {
  Float adjustedModelCount = totalModelCount;
  bool unappearedVar = false;
  for (Int pbVar : activeVarSet) {
    if(!isFound(pbVar, processedVariables)) {
      if (!isFound(pbVar, projectionSupportVarSet)) { continue; }
      if (inferredAssignments.find(pbVar) == inferredAssignments.end()) {
        unappearedVar = true;
        Float posWeight = 1.0;
        Float negWeight = 1.0;
        if (literalWeights.find(pbVar) != literalWeights.end()){
          posWeight = literalWeights.at(pbVar);
        }
        if (literalWeights.find(-pbVar) != literalWeights.end()){
          negWeight = literalWeights.at(-pbVar);
        }
        adjustedModelCount *= posWeight + negWeight;
      }
    }
  }
  if (unappearedVar) {
    std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
  }
  return adjustedModelCount;
}


/* functions: timing **********************************************************/

TimePoint util::getTimePoint() {
  return std::chrono::steady_clock::now();
}

Float util::getSeconds(TimePoint startTime) {
  TimePoint endTime = getTimePoint();
  return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0;
}

Float util::getMilliseconds(TimePoint startTime) {
  TimePoint endTime = getTimePoint();
  return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
}

void util::printDuration(TimePoint startTime) {
  printThickLine();
  printRow("seconds", getSeconds(startTime));
  printThickLine();
}

/* functions: error handling **************************************************/

void util::showWarning(const string &message, bool commented) {
  printBoldLine(commented);
  printComment("MY_WARNING: " + message, 0, 1, commented);
  printBoldLine(commented);
}

void util::showError(const string &message, bool commented) {
  throw MyError(message, commented);
}

/* classes ********************************************************************/

/* class MyError **************************************************************/

MyError::MyError(const string &message, bool commented) {
  util::printBoldLine(commented);
  util::printComment("MY_ERROR: " + message, 0, 1, commented);
  util::printBoldLine(commented);
}
