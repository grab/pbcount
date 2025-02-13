/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#pragma once

/* inclusions *****************************************************************/

#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <random>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <sstream>

#include "pbclause.hpp"

#include "../../lib/cudd-3.0.0/cplusplus/cuddObj.hh"
#include "../../lib/cudd-3.0.0/cudd/cuddInt.h"

/* uses ***********************************************************************/

using std::cout;
using std::string;
using std::to_string;
using std::vector;

/* types **********************************************************************/

using Float = double; // std::stod // OPTIL would complain about 'long double'
using Int = int_fast64_t; // std::stoll
using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;

template<typename K, typename V> using Map = std::unordered_map<K, V>;
template<typename T> using Set = std::unordered_set<T>;

/* global variables ***********************************************************/

extern Int randomSeed; // for reproducibility
extern Int verbosityLevel;
extern TimePoint startTime;

/* constants ******************************************************************/

extern const string &COMMENT_WORD;
extern const string &PROBLEM_WORD;

extern const string &STDIN_CONVENTION;

extern const string &REQUIRED_OPTION_GROUP;
extern const string &OPTIONAL_OPTION_GROUP;

extern const string &HELP_OPTION;
extern const string &PB_FILE_OPTION;
extern const string &WEIGHT_FORMAT_OPTION;
extern const string &JT_FILE_OPTION;
extern const string &JT_WAIT_DURAION_OPTION;
extern const string &OUTPUT_FORMAT_OPTION;
extern const string &CLUSTERING_HEURISTIC_OPTION;
extern const string &CLUSTER_VAR_ORDER_OPTION;
extern const string &DIAGRAM_VAR_ORDER_OPTION;
extern const string &RANDOM_SEED_OPTION;
extern const string &VERBOSITY_LEVEL_OPTION;
extern const string &CLAUSE_COMPILATION_HEURISTIC_OPTION;
extern const string &PREPROCESSING_OPTION;
extern const string &OPERATION_MODE_OPTION;
extern const string &INTERACTION_ADAPTIVE_RESTART_OPTION;

// enum class WeightFormat { UNWEIGHTED, MINIC2D, CACHET, MCC };
enum class WeightFormat { UNWEIGHTED, WEIGHTED, MINIC2D, CACHET, MCC};
extern const std::map<Int, WeightFormat> WEIGHT_FORMAT_CHOICES;
extern const Int DEFAULT_WEIGHT_FORMAT_CHOICE;

extern const Float DEFAULT_JT_WAIT_SECONDS;

enum class OutputFormat { JOIN_TREE, MODEL_COUNT };
extern const std::map<Int, OutputFormat> OUTPUT_FORMAT_CHOICES;
extern const Int DEFAULT_OUTPUT_FORMAT_CHOICE;

enum class ClusteringHeuristic { MONOLITHIC, LINEAR, BUCKET_LIST, BUCKET_TREE, BOUQUET_LIST, BOUQUET_TREE, COMPUTE_GRAPH_MIN_DEGREE };
extern const std::map<Int, ClusteringHeuristic> CLUSTERING_HEURISTIC_CHOICES;
extern const Int DEFAULT_CLUSTERING_HEURISTIC_CHOICE;

enum class VarOrderingHeuristic {
  DUMMY_VAR_ORDERING_HEURISTIC, // would trigger error in Cnf::getVarOrdering
  APPEARANCE, DECLARATION, RANDOM, MCS, LEXP, LEXM, MINFILL
}; // note only add to the back of the enum to preserve previous enum nunbers
extern const std::map<Int, VarOrderingHeuristic> VAR_ORDERING_HEURISTIC_CHOICES;
extern const Int DEFAULT_CNF_VAR_ORDERING_HEURISTIC_CHOICE;
extern const Int DEFAULT_DD_VAR_ORDERING_HEURISTIC_CHOICE;

enum class ClauseCompilationHeuristic { BOTTOMUP, TOPDOWN, DYNAMIC, OPT_BOTTOMUP, OPT_TOPDOWN };
extern const std::map<Int, ClauseCompilationHeuristic> CLAUSE_COMPILATION_HEURISTIC_CHOICES;
extern const Int DEFAULT_CLAUSE_COMPILATION_HEURISTIC_CHOICE;

enum class PreprocessingConfig { OFF, ALL };
extern const std::map<Int, PreprocessingConfig> PREPROCESSING_CHOICES;
extern const Int DEFAULT_PREPROCESSING_CHOICE;

enum class OperationModeChoice { Single, Incremental };
extern const std::map<Int, OperationModeChoice> OPERATION_MODES;
extern const Int DEFAULT_OPERATION_MODE_CHOICE;

enum class AdaptiveRestartChoice { ON, OFF };
extern const std::map<Int, AdaptiveRestartChoice> INTERACTIVE_ADAPTIVE_RESTART_CHOICES;
extern const Int DEFAULT_INTERACTIVE_ADAPTIVE_RESTART_CHOICE;

extern const Int DEFAULT_RANDOM_SEED;

extern const vector<Int> VERBOSITY_LEVEL_CHOICES;
extern const Int DEFAULT_VERBOSITY_LEVEL_CHOICE;

extern const Float NEGATIVE_INFINITY;

extern const Int DUMMY_MIN_INT;
extern const Int DUMMY_MAX_INT;

extern const string &DUMMY_STR;

extern const string &DOT_DIR;

extern const string &PROJECTED_COUNTING_UNSUPPORTED_STR;
/* namespaces *****************************************************************/

namespace util {
  bool isInt(Float d);

  /* functions: printing ******************************************************/

  void printComment(const string &message, Int preceedingNewLines = 0, Int followingNewLines = 1, bool commented = true);
  void printSolutionLine(WeightFormat weightFormat, Float modelCount, Int preceedingThinLines = 1, Int followingThinLines = 1);

  void printBoldLine(bool commented);
  void printThickLine(bool commented = true);
  void printThinLine();

  void printHelpOption();
  void printCnfFileOption();
  void printWeightFormatOption();
  void printJtFileOption();
  void printJtWaitOption();
  void printOutputFormatOption();
  void printClusteringHeuristicOption();
  void printPreprocessingOption();
  void printOperationModeOption();
  void printInteractionAdaptiveRestartOption();
  void printCnfVarOrderingHeuristicOption();
  void printDdVarOrderingHeuristicOption();
  void printClauseCompilationHeuristicOption();
  void printRandomSeedOption();
  void printVerbosityLevelOption();

  /* functions: argument parsing **********************************************/

  vector<string> getArgV(int argc, char *argv[]);

  string getWeightFormatName(WeightFormat weightFormat);
  string getOutputFormatName(OutputFormat outputFormat);
  string getClusteringHeuristicName(ClusteringHeuristic clusteringHeuristic);
  string getVarOrderingHeuristicName(VarOrderingHeuristic varOrderingHeuristic);
  string getVerbosityLevelName(Int verbosityLevel);
  string getClauseCompilationHeuristicName(ClauseCompilationHeuristic clauseCompilationHeuristic);
  string getPreprocessingConfigName(PreprocessingConfig preprocessingConfig);
  string getOperationModeChoiceName(OperationModeChoice operationMode);
  string getInteractiveAdaptiveRestartChoiceName(AdaptiveRestartChoice restartMode);

  /* functions: CNF ***********************************************************/

  Int getCnfVar(Int literal);
  Set<Int> getClauseCnfVars(const vector<Int> &clause);
  Set<Int> getClusterCnfVars(const vector<Int> &cluster, const vector<vector<Int>> &clauses);

  bool appearsIn(Int cnfVar, const vector<Int> &clause);
  bool isPositiveLiteral(Int literal);

  Int getLiteralRank(Int literal, const vector<Int> &cnfVarOrdering);
  Int getMinClauseRank(const vector<Int> &clause, const vector<Int> &cnfVarOrdering);
  Int getMaxClauseRank(const vector<Int> &clause, const vector<Int> &cnfVarOrdering);

  void printClause(const vector<Int> &clause);
  void printCnf(const vector<vector<Int>> &clauses);
  void printLiteralWeights(const Map<Int, Float> &literalWeights);

  /* functions: PB ***********************************************************/

  Int getPbVar(Int literal);
  Set<Int> getClausePbVars(const PBclause &clause);
  Set<Int> getClusterPbVars(const vector<Int> &cluster, const vector<PBclause> &clauses);

  bool appearsIn(Int pbVar, const PBclause &clause);
  // same as cnf version, so do no need to implement
  // bool isPositiveLiteral(Int literal);
  //
  // Int getLiteralRank(Int literal, const vector<Int> &pbVarOrdering);
  Int getMinClauseRank(const PBclause &clause, const vector<Int> &pbVarOrdering);
  Int getMaxClauseRank(const PBclause &clause, const vector<Int> &pbVarOrdering);

  void printClause(const PBclause &clause);
  void printPb(const vector<PBclause> &clauses);
  // void printLiteralWeights(const Map<Int, Float> &literalWeights);

  /* functions: model count adjustment for compute graph counter ***********************************************************/

  Float adjustModelCountCG(Float apparentModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments);
  Float adjustModelCountCGMissingVar (Float totalModelCount, Int parsedVarNum, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &processedVariables);
  Float adjustProjectedModelCountCG(Float apparentModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &projectionSupportVarSet);
  Float adjustProjectedModelCountCGMissingVar (Float totalModelCount, Int parsedVarNum, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &processedVariables, const Set<Int> &projectionSupportVarSet);

  // interactive mode with projected model count
  Float adjustInteractiveProjectedModelCountCG(Float apparentModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &projectionSupportVarSet, Set<Int> &activeVarSet);
  Float adjustInteractiveProjectedModelCountCGMissingVar (Float totalModelCount, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments, const Set<Int> &processedVariables, const Set<Int> &projectionSupportVarSet, Set<Int> &activeVarSet);

  /* functions: timing ********************************************************/

  TimePoint getTimePoint();
  Float getSeconds(TimePoint startTime);
  Float getMilliseconds(TimePoint startTime);
  void printDuration(TimePoint startTime);

  /* functions: error handling ************************************************/

  void showWarning(const string &message, bool commented = true);
  void showError(const string &message, bool commented = true);

  /* functions: templates implemented in headers to avoid linker errors *******/

  template<typename T> void printRow(const string &name, const T &value) {
    cout << COMMENT_WORD << " " << std::left << std::setw(30) << name;
    cout << std::left << std::setw(15) << value << "\n";
  }

  template<typename T> void printContainer(const T &container) {
    cout << "printContainer:\n";
    for (const auto &member : container) {
      cout << "\t" << member << "\t";
    }
    cout << "\n";
  }

  template<typename K, typename V> void printMap(const Map<K, V> &m) {
    cout << "printMap:\n";
    for (const auto &kv : m) {
      cout << "\t" << kv.first << "\t:\t" << kv.second << "\n";
    }
    cout << "\n";
  }

  template<typename Key, typename Value> bool isLessValued(std::pair<Key, Value> a, std::pair<Key, Value> b) {
    return a.second < b.second;
  }

  template<typename T> T getSoleMember(const vector<T> &v) {
    if (v.size() != 1) showError("vector is not singleton");
    return v.front();
  }

  template<typename T> void popBack(T &element, vector<T> &v) {
    if (v.empty()) showError("vector is empty");
    element = v.back();
    v.pop_back();
  }

  template<typename T> void invert(T &t) {
    std::reverse(t.begin(), t.end());
  }

  template<typename T, typename U> bool isFound(const T &element, const U &container) {
    return std::find(std::begin(container), std::end(container), element) != std::end(container);
  }

  template<typename T, typename U, typename V> bool isFound(const T &element, const Map<U, V> &container) {
    return container.find(element) != container.end();
  }

  template<typename T, typename U1, typename U2> void differ(Set<T> &diff, const U1 &members, const U2 &nonmembers) {
    for (const auto &member : members) {
      if (!isFound(member, nonmembers)) {
        diff.insert(member);
      }
    }
  }

  template<typename T, typename U> void unionize(Set<T> &unionSet, const U &container) {
    for (const auto &member : container) unionSet.insert(member);
  }

  template<typename T, typename U> bool isDisjoint(const T &container, const U &container2) {
    for (const auto &member : container) {
      for (const auto &member2 : container2) {
        if (member == member2) {
          return false;
        }
      }
    }
    return true;
  }

  template<typename T> Float adjustModelCount(Float apparentModelCount, const T &projectedCnfVars, const Map<Int, Float> &literalWeights) {
    Float totalModelCount = apparentModelCount;

    Int totalLiteralCount = literalWeights.size();
    if (totalLiteralCount % 2 == 1) showError("odd total literal count");

    Int totalVarCount = totalLiteralCount / 2;
    if (totalVarCount < projectedCnfVars.size()) showError("more projected vars than total vars");

    // NOTE: potential out of range map, because the variable did not appear in any of the clauses, so its weight is not set
    bool unappearedVar = false;
    for (Int cnfVar = 1; cnfVar <= totalVarCount; cnfVar++) {
      if (!isFound(cnfVar, projectedCnfVars)) {
        // added to prevent error, may return unintended count if some variable did not appear
        double posWeight = 1.0;
        double negWeight = 1.0;
        if (literalWeights.find(cnfVar) != literalWeights.end()){
          posWeight = literalWeights.at(cnfVar);
        } else {
          unappearedVar = true;
        }
        if (literalWeights.find(-cnfVar) != literalWeights.end()){
          negWeight = literalWeights.at(-cnfVar);
        } else {
          unappearedVar = true;
        }
        totalModelCount *= posWeight + negWeight;
        //
        // totalModelCount *= literalWeights.at(cnfVar) + literalWeights.at(-cnfVar);
      }
    }
    // informing user of potential error
    if (unappearedVar) {
      std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
    }

    if (totalModelCount == 0) {
      showWarning("floating-point underflow may have occured or unsat formula");
    }
    return totalModelCount;
  }

  template<typename T> Float adjustProjectedModelCount(Float apparentModelCount, const T &projectedPBVars, const Map<Int, Float> &literalWeights, const Set<Int> &projectionVariableSet) {
    Float totalModelCount = apparentModelCount;

    Int totalLiteralCount = literalWeights.size();
    if (totalLiteralCount % 2 == 1) showError("odd total literal count");

    Int totalVarCount = totalLiteralCount / 2;
    if (totalVarCount < projectedPBVars.size()) showError("more projected vars than total vars");

    bool unappearedVar = false;
    for (Int pbVar = 1; pbVar <= totalVarCount; pbVar++) {
      if (!isFound(pbVar, projectedPBVars)) {
        if (isFound(pbVar, projectionVariableSet)) {
          double posWeight = 1.0;
          double negWeight = 1.0;
          if (literalWeights.find(pbVar) != literalWeights.end()){
            posWeight = literalWeights.at(pbVar);
          } else {
            unappearedVar = true;
          }
          if (literalWeights.find(-pbVar) != literalWeights.end()){
            negWeight = literalWeights.at(-pbVar);
          } else {
            unappearedVar = true;
          }
          totalModelCount *= posWeight + negWeight;
          // totalModelCount *= literalWeights.at(pbVar) + literalWeights.at(-pbVar);
        }
      }
    }
    // informing user of potential error
    if (unappearedVar) {
      std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
    }

    if (totalModelCount == 0) {
      showWarning("floating-point underflow may have occured");
    }
    return totalModelCount;
  }

  /* model counting adjustment with preprocessing (inferred assignments)*/
  template<typename T> Float adjustModelCount(Float apparentModelCount, const T &projectedCnfVars, const Map<Int, Float> &literalWeights, const Map<Int, bool> &inferredAssignments) {
    /*
    This version is to handle counting with preprocessing.
    Unit clauses are removed in preprocessing and we want to account for
    the situation where x1>=1 (only 1 satisfying assignment) but x1 does not appear elsewhere so not counted. (we cannot assume both true and false satisfy now)
    */
    // debug print
    // std::cout << "support size: " << projectedCnfVars.size() << std::endl;
    //
    Float totalModelCount = apparentModelCount;

    Int totalLiteralCount = literalWeights.size();
    if (totalLiteralCount % 2 == 1) showError("odd total literal count");

    Int totalVarCount = totalLiteralCount / 2;
    if (totalVarCount < projectedCnfVars.size()) showError("more projected vars than total vars");

    // NOTE: potential out of range map, because the variable did not appear in any of the clauses, so its weight is not set
    bool unappearedVar = false;
    for (Int var = 1; var <= totalVarCount; var++) {
      if (!isFound(var, projectedCnfVars)) {
        // added to prevent error, may return unintended count if some variable did not appear
        double posWeight = 1.0;
        double negWeight = 1.0;
        if (literalWeights.find(var) != literalWeights.end()){
          posWeight = literalWeights.at(var);
        } else {
          unappearedVar = true;
        }
        if (literalWeights.find(-var) != literalWeights.end()){
          negWeight = literalWeights.at(-var);
        } else {
          unappearedVar = true;
        }
        if (inferredAssignments.find(var) != inferredAssignments.end()) {
          if (inferredAssignments.at(var)) {
            // inferred assignment is true
            totalModelCount *= posWeight;
          } else {
            totalModelCount *= negWeight;
          }
        } else {
          totalModelCount *= posWeight + negWeight;
        }
        //
        // totalModelCount *= literalWeights.at(cnfVar) + literalWeights.at(-cnfVar);
      }
    }
    // informing user of potential error
    if (unappearedVar) {
      std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
    }

    if (totalModelCount == 0) {
      showWarning("floating-point underflow may have occured or unsat formula");
    }
    return totalModelCount;
  }

  template<typename T> Float adjustProjectedModelCount(Float apparentModelCount, const T &projectedPBVars, const Map<Int, Float> &literalWeights, const Set<Int> &projectionVariableSet, const Map<Int, bool> &inferredAssignments) {
    Float totalModelCount = apparentModelCount;

    Int totalLiteralCount = literalWeights.size();
    if (totalLiteralCount % 2 == 1) showError("odd total literal count");

    Int totalVarCount = totalLiteralCount / 2;
    if (totalVarCount < projectedPBVars.size()) showError("more projected vars than total vars");

    bool unappearedVar = false;
    for (Int pbVar = 1; pbVar <= totalVarCount; pbVar++) {
      if (!isFound(pbVar, projectedPBVars)) {
        if (isFound(pbVar, projectionVariableSet)) {
          double posWeight = 1.0;
          double negWeight = 1.0;
          if (literalWeights.find(pbVar) != literalWeights.end()){
            posWeight = literalWeights.at(pbVar);
          } else {
            unappearedVar = true;
          }
          if (literalWeights.find(-pbVar) != literalWeights.end()){
            negWeight = literalWeights.at(-pbVar);
          } else {
            unappearedVar = true;
          }
          if (inferredAssignments.find(pbVar) != inferredAssignments.end()) {
            if (inferredAssignments.at(pbVar)) {
              // inferred assignment is true
              totalModelCount *= posWeight;
            } else {
              totalModelCount *= negWeight;
            }
          } else {
            totalModelCount *= posWeight + negWeight;
          }
          // totalModelCount *= posWeight + negWeight;
        }
      }
    }
    // informing user of potential error
    if (unappearedVar) {
      std::cout << "Some variables did not appear in formula, setting weight to 1.0 in counting" << std::endl; 
    }

    if (totalModelCount == 0) {
      showWarning("floating-point underflow may have occured");
    }
    return totalModelCount;
  }
  // end of preprocessed model count adjustment

  template<typename T> void shuffleRandomly(T &container) {
    std::mt19937 generator;
    generator.seed(randomSeed);
    std::shuffle(container.begin(), container.end(), generator);
  }

  template<typename Dd> Set<Int> getSupport(const Dd &dd) {
    Set<Int> support;
    for (Int ddVar : dd.SupportIndices()) support.insert(ddVar);
    return support;
  }

  template<typename Dd> Set<Int> getSupportSuperset(const vector<Dd> &dds) {
    Set<Int> supersupport;
    for (const Dd &dd : dds) for (Int var : dd.SupportIndices()) supersupport.insert(var);
    return supersupport;
  }

  template<typename Dd> Int getMinDdRank(const Dd &dd, const vector<Int> &ddVarToCnfVarMap, const vector<Int> &cnfVarOrdering) {
    Int minRank = DUMMY_MAX_INT;
    for (Int ddVar : getSupport(dd)) {
      Int cnfVar = ddVarToCnfVarMap.at(ddVar);
      Int rank = getLiteralRank(cnfVar, cnfVarOrdering);
      if (rank < minRank) minRank = rank;
    }
    return minRank;
  }

  template<typename Dd> Int getMaxDdRank(const Dd &dd, const vector<Int> &ddVarToCnfVarMap, const vector<Int> &cnfVarOrdering) {
    Int maxRank = DUMMY_MIN_INT;
    for (Int ddVar : getSupport(dd)) {
      Int cnfVar = ddVarToCnfVarMap.at(ddVar);
      Int rank = getLiteralRank(cnfVar, cnfVarOrdering);
      if (rank > maxRank) maxRank = rank;
    }
    return maxRank;
  }
  // Zips elements of two vectors, similar to python zip
  // Assumes the vectors have equal length)
  template <typename A, typename B>
  void zip(const std::vector<A> &a, const std::vector<B> &b,
           std::vector<std::pair<A, B>> &zippedVector) {
    for (Int i = 0; i < a.size(); ++i) {
      zippedVector.push_back(std::make_pair(a[i], b[i]));
    }
  }
  // Unzips elements into individual vectors in place.
  // Assumes vectors have equal length)
  template <typename A, typename B>
  void unzip(const std::vector<std::pair<A, B>> &zippedVector,
             std::vector<A> &a, std::vector<B> &b) {
    for (Int i = 0; i < a.size(); i++) {
      a[i] = zippedVector[i].first;
      b[i] = zippedVector[i].second;
    }
  }

  template <typename T>
  Set<T> setIntersection(Set<T> &setA, Set<T> &setB) {
    Set<T> intersectionSet;
    if (setA.size() > setB.size()) {
      for (auto element : setB) {
        if (util::isFound(element, setA)) { intersectionSet.insert(element); }
      }
    } else {
      for (auto element : setA) {
        if (util::isFound(element, setB)) { intersectionSet.insert(element); }
      }
    }
    return intersectionSet;
  }

  template <typename T>
  Set<T> setUnion(Set<T> &setA, Set<T> &setB) {
    Set<T> unionSet;
    for (auto element : setA) {
      unionSet.insert(element);
    }
    for (auto element : setB) {
      unionSet.insert(element);
    }
    return unionSet;
  }
  // Set (unordered_set) check is subset
  template <typename T>
  bool isSubset(Set<T> &subsetA, Set<T> &setB) {
    // checks if A is subset of B
    if(subsetA.size() > setB.size()) { return false; }
    for (auto &element : subsetA) {
      // if (!util::isFound(element, setB)) { return false; }
      if(setB.find(element) == setB.end()) {return false;}
    }
    return true;
  }
}

/* classes ********************************************************************/

class MyError {
public:
  MyError(const string &message, bool commented);
};
