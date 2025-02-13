/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#pragma once

/* inclusions *****************************************************************/

#include "formula.hpp"
#include "join.hpp"
#include "visual.hpp"
#include "pbformula.hpp"

#include <sstream>
using util::isFound;
/* namespaces *****************************************************************/

namespace diagram {
  Float getTerminalValue(const ADD &terminal);
  Float countConstDdFloat(const ADD &dd);
  Int countConstDdInt(const ADD &dd);
  void printMaxDdVarCount(Int maxDdVarCount);
}


/* PB classes ********************************************************************/
class PBCounter { // abstract
protected:
  static WeightFormat weightFormat;

  Int dotFileIndex = 1;
  Cudd mgr;
  VarOrderingHeuristic ddVarOrderingHeuristic;
  bool inverseDdVarOrdering;
  ClauseCompilationHeuristic clauseCompilationHeuristic;
  Map<Int, Int> pbVarToDdVarMap; // e.g. {42: 0, 13: 1}
  vector<Int> ddVarToPbVarMap; // e.g. [42, 13], i.e. ddVarOrdering

  JoinNonterminal *joinRoot;

  static void handleSignals(int signal); // `timeout` sends SIGTERM

  void writeDotFile(ADD &dd, const string &dotFileDir = DOT_DIR);
  template<typename T> Set<Int> getPbVars(const T &ddVars) {
    Set<Int> pbVars;
    for (Int ddVar : ddVars) pbVars.insert(ddVarToPbVarMap.at(ddVar));
    return pbVars;
  }
  const vector<Int> &getDdVarOrdering() const; // ddVarToCnfVarMap
  void orderDdVars(const PBformula &pb); // writes: cnfVarToDdVarMap, ddVarToCnfVarMap
  ADD getClauseDd(const PBclause &clause) const;
  ADD getClauseDdTD(const PBclause &clause) const; // top down ITE construction
  ADD getClauseDdTDHelper(const PBclause &clause, Int idx, Int currentVal, bool remainCoeffPositive) const;
  ADD getClauseDdBU(const PBclause &clause) const; // bottom up threshold construction
  ADD getClauseDdDynamic(const PBclause &clause) const; // on the fly choose between top down or bottom up

  PBclause getOptimizedClauseBU(const PBclause &clause) const; // clause arrangement optimization for bottom up compilation
  PBclause getOptimizedClauseTD(const PBclause &clause) const; // clause arrangement optimization for top down compilation

  void abstract(ADD &dd, Int ddVar, const Map<Int, Float> &literalWeights);
  void abstractCube(ADD &dd, const Set<Int> &ddVars, const Map<Int, Float> &literalWeights);
  void projectAbstract(ADD &dd, Int ddVar, const Map<Int, Float> &literalWeights, const Set<Int> &projectionVariableSet); // project away variable without counting if in projection set, else normal abstract
  void projectAbstractCube(ADD &dd, const Set<Int> &ddVars, const Map<Int, Float> &literalWeights, const Set<Int> &projectionVariableSet);

  void printJoinTree(const PBformula &pb) const;

public:
  virtual void constructJoinTree(const PBformula &pb) = 0; // handles cnf without empty clause
  void setJoinTree(const PBformula &pb); // handles cnf with/without empty clause

  ADD countSubtree(JoinNode *joinNode, const PBformula &pb, Set<Int> &projectedPbVars); // handles cnf without empty clause
  Float countJoinTree(const PBformula &pb); // handles cnf with/without empty clause

  virtual Float computeModelCount(const PBformula &pb) = 0; // handles cnf without empty clause
  Float getModelCount(const PBformula &pb); // handles cnf with/without empty clause

  void output(const string &filePath, WeightFormat weightFormat, OutputFormat outputFormat, PreprocessingConfig preprocessingConfig);
};

class PBJoinTreeCounter : public PBCounter {
public:
  void constructJoinTree(const PBformula &pb) override;
  Float computeModelCount(const PBformula &pb) override;
  PBJoinTreeCounter(
    const string &jtFilePath,
    Float jtWaitSeconds,
    VarOrderingHeuristic ddVarOrderingHeuristic,
    bool inverseDdVarOrdering,
    ClauseCompilationHeuristic clauseCompilationHeuristic
  );
};

class PBMonolithicCounter : public PBCounter { // builds an ADD for the entire CNF
protected:
  void setMonolithicClauseDds(vector<ADD> &clauseDds, const PBformula &pb);
  void setPbDd(ADD &pbDd, const PBformula &pb);

public:
  void constructJoinTree(const PBformula &pb) override;
  Float computeModelCount(const PBformula &pb) override;
  PBMonolithicCounter(
    VarOrderingHeuristic ddVarOrderingHeuristic,
    bool inverseDdVarOrdering,
    ClauseCompilationHeuristic clauseCompilationHeuristic
  );
};

class PBFactoredCounter : public PBCounter {}; // abstract; builds an ADD for each clause

class PBLinearCounter : public PBFactoredCounter { // combines adjacent clauses
protected:
  vector<Set<Int>> projectablePbVarSets; // clauseIndex |-> cnfVars

  void fillProjectablePbVarSets(const vector<PBclause> &clauses);
  void setLinearClauseDds(vector<ADD> &clauseDds, const PBformula &pb);

public:
  void constructJoinTree(const PBformula &pb) override;
  Float computeModelCount(const PBformula &pb) override;
  PBLinearCounter(
    VarOrderingHeuristic ddVarOrderingHeuristic,
    bool inverseDdVarOrdering,
    ClauseCompilationHeuristic clauseCompilationHeuristic
  );
};

class PBNonlinearCounter : public PBFactoredCounter { // abstract; puts clauses in clusters
protected:
  bool usingTreeClustering;
  VarOrderingHeuristic pbVarOrderingHeuristic;
  bool inversePbVarOrdering;
  vector<vector<Int>> clusters; // clusterIndex |-> clauseIndices

  vector<Set<Int>> occurrentPbVarSets; // clusterIndex |-> pbVars
  vector<Set<Int>> projectablePbVarSets; // clusterIndex |-> pbVars
  vector<vector<JoinNode *>> joinNodeSets; // clusterIndex |-> non-null nodes

  vector<vector<ADD>> ddClusters; // clusterIndex |-> ADDs (if usingTreeClustering)
  vector<Set<Int>> projectingDdVarSets; // clusterIndex |-> ddVars (if usingTreeClustering)

  void printClusters(const vector<PBclause> &clauses) const;
  void fillClusters(const vector<PBclause> &clauses, const vector<Int> &pbVarOrdering, bool usingMinVar);

  void printOccurrentPbVarSets() const;
  void printProjectablePbVarSets() const;
  void fillPbVarSets(const vector<PBclause> &clauses, bool usingMinVar); // writes: occurrentCnfVarSets, projectableCnfVarSets

  Set<Int> getProjectingDdVars(Int clusterIndex, bool usingMinVar, const vector<Int> &pbVarOrdering, const vector<PBclause> &clauses);
  void fillDdClusters(const vector<PBclause> &clauses, const vector<Int> &pbVarOrdering, bool usingMinVar); // (if usingTreeClustering)
  void fillProjectingDdVarSets(const vector<PBclause> &clauses, const vector<Int> &pbVarOrdering, bool usingMinVar); // (if usingTreeClustering)

  Int getTargetClusterIndex(Int clusterIndex) const; // returns DUMMY_MAX_INT if no var remains
  Int getNewClusterIndex(const ADD &abstractedClusterDd, const vector<Int> &pbVarOrdering, bool usingMinVar) const; // returns DUMMY_MAX_INT if no var remains (if usingTreeClustering)
  Int getNewClusterIndex(const Set<Int> &remainingDdVars) const; // returns DUMMY_MAX_INT if no var remains (if usingTreeClustering) #MAVC

  void constructJoinTreeUsingListClustering(const PBformula &pb, bool usingMinVar);
  void constructJoinTreeUsingTreeClustering(const PBformula &pb, bool usingMinVar);

  Float countUsingListClustering(const PBformula &pb, bool usingMinVar);
  Float countUsingTreeClustering(const PBformula &pb, bool usingMinVar);
  Float countUsingTreeClustering(const PBformula &pb); // #MAVC
};

class PBBucketCounter : public PBNonlinearCounter { // bucket elimination
public:
  void constructJoinTree(const PBformula &pb) override;
  Float computeModelCount(const PBformula &pb) override;
  PBBucketCounter(
    bool usingTreeClustering,
    VarOrderingHeuristic pbVarOrderingHeuristic,
    bool inversePbVarOrdering,
    VarOrderingHeuristic ddVarOrderingHeuristic,
    bool inverseDdVarOrdering,
    ClauseCompilationHeuristic clauseCompilationHeuristic
  );
};

class PBBouquetCounter : public PBNonlinearCounter { // Bouquet's Method
public:
  void constructJoinTree(const PBformula &pb) override;
  Float computeModelCount(const PBformula &pb) override;
  PBBouquetCounter(
    bool usingTreeClustering,
    VarOrderingHeuristic pbVarOrderingHeuristic,
    bool inversePbVarOrdering,
    VarOrderingHeuristic ddVarOrderingHeuristic,
    bool inverseDdVarOrdering,
    ClauseCompilationHeuristic clauseCompilationHeuristic
  );
};

class PBComputeGraphCounter : public PBCounter { // compute graph counter
protected:
  Map<Int, int> preferredVariableOrdering; // storing an ordering of pbVar : ordering position
  VarOrderingHeuristic pbVarOrderingHeuristic;
  bool inversePbVarOrdering;
  void setPreferredVariableOrdering(VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, const PBformula &pb);

  void orAbstract(ADD &dd, Int ddVar);
  void orAbstractCube(ADD &dd, Set<Int> &ddVars);
  // tree construction not supported here, just overriding to be useable class and not virtual
  void constructJoinTree(const PBformula &pb) override;
  void eagerAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights);
  void eagerOrAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet); // projected MC
  Map<Int, ADD> compileClauses(const PBformula &pb);
  Int getLeastCommonVar(Map<Int, Set<Int>> &varToClauseMap) const;
  Int getLeastCommonNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const;
  Int getNextMergeSupportVar(Map<Int, Set<Int>> &varToClauseMap) const;
  Int getNextMergeNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const;
  void mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, const Map<Int, Float> &literalWeights);
  void mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet); // projected MC
  Set<Int> getClauseExclusiveVarIntersection(Set<Int> &clauseSet, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, Set<Int>> &varToClauseMap);
public:
  Float computeModelCount(const PBformula &pb) override;
  Float computeProjectedModelCount(const PBformula &pb);
  PBComputeGraphCounter(
    VarOrderingHeuristic pbVarOrderingHeuristic,
    bool inversePbVarOrdering,
    VarOrderingHeuristic ddVarOrderingHeuristic,
    bool inverseDdVarOrdering,
    ClauseCompilationHeuristic clauseCompilationHeuristic
  );
};

class PBInteractiveCounter : public PBCounter { // interactive counter with caching support
private:
  struct ComputeState {
    /* clause id is used as a mapping to dd (dd Id = clause Id) */
    Map<Int, ADD> clauseToDdMapState;
    Map<Int, Set<Int>> clauseToVarMapState;
    Map<Int, Set<Int>> varToClauseMapState;
    Map<Int, Set<Int>> clauseToAbstractedVarMapState;
    Map<Int, Set<Int>> clauseToOrAbstractedVarMapState;
    Map<Int, Set<Int>> clauseDDToFormulaClauseMapState;
  }; // used for resuming from cache
  

protected:
  Map<Int, int> preferredVariableOrdering; // storing an ordering of pbVar : ordering position
  VarOrderingHeuristic pbVarOrderingHeuristic;
  bool inversePbVarOrdering;
  void setPreferredVariableOrdering(VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, const PBformula &pb);

  vector<bool> activeClauses;
  Map<string, ADD> ddCache;
  Map<Int, ADD> singleClauseDdMap;
  Float CACHE_TIME_THRESHOLD = 0.0; // in milliseconds
  bool restartOnOrderingMisalign = false;
  Float ORDER_OFFSET_RESTART_THRESHOLD_RATIO = 0.2; // to adjust accordingly
  bool PRINT_ORDERING_OFFSET = false; // MANUALLY SET to compute and print offset each time, for determining optimal offset threshold ratio

  Float computeMaxOrderingOffsetRatio(const vector<Int>& originalOrdering, const vector<Int>& tentativeOrdering);
  Float computeMedianOrderingOffsetRatio(const vector<Int>& originalOrdering, const vector<Int>& tentativeOrdering);
  void orderDdVarsIncrementalWithRestart(const PBformula &pb);
  void orderDdVarsIncremental(const PBformula &pb);
  void constructJoinTree(const PBformula &pb) override;
  void saveDdCache(string &ddIdentifer, ADD &dd);
  string getDdIdentifierString(Set<Int> &mergedClauseSet, Set<Int> &earlyAbstractedVarSet);
  string getDdIdentifierString(Set<Int> &mergedClauseSet, Set<Int> &earlyAbstractedVarSet, Set<Int> &earlyOrAbstractedVarSet);
  vector<Set<Int>> decodeIdentifier(string &ddIdentifier);
  ComputeState resolveCache(const PBformula &pb, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap);
  Set<Int> getActiveVariables(const PBformula &pb);

  void orAbstract(ADD &dd, Int ddVar);
  void orAbstractCube(ADD &dd, Set<Int> &ddVars);
  void eagerAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights);
  void eagerOrAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet); // projected MC
  Map<Int, ADD> compileClauses(const PBformula &pb);
  Int getLeastCommonVar(Map<Int, Set<Int>> &varToClauseMap) const;
  Int getLeastCommonNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const;
  Int getNextAbstractionVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const;
  Int getNextMergeSupportVar(Map<Int, Set<Int>> &varToClauseMap) const;
  Int getNextMergeNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const;
  Set<Int> mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &clauseToAbstractedVarMap, Map<Int, Set<Int>> &clauseDDToFormulaClauseMap, const Map<Int, Float> &literalWeights);
  Set<Int> mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &clauseToAbstractedVarMap, Map<Int, Set<Int>> &clauseToOrAbstractedVarMap , Map<Int, Set<Int>> &clauseDDToFormulaClauseMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet); // projected MC
  Set<Int> getClauseExclusiveVarIntersection(Set<Int> &clauseSet, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, Set<Int>> &varToClauseMap);

public:
  void setAdaptiveRestartMode(bool restartOn);
  void setDDMgr(Cudd &newMgr);
  Cudd &getDDMgr();
  void setCache(Map<string, ADD> &newCache);
  void clearCache();
  Map<string, ADD> &getCache();
  void setActiveClauses(vector<bool> newActiveClauses);
  Float computeModelCount(const PBformula &pb) override;
  Float computeProjectedModelCount(const PBformula &pb);
  PBInteractiveCounter();
  
  void setDdVarOrderingHeuristic(VarOrderingHeuristic ddVarOrderingHeuristic);
  void setInverseDdVarOrdering(bool inverseDdVarOrdering);
  void setClauseCompilationHeuristic(ClauseCompilationHeuristic clauseCompilationHeuristic);
  void setPbVarOrderingHeuristic(VarOrderingHeuristic pdVarOrderingHeuristic);
  void setInversePbVarOrdering(bool inversePbVarOrdering);
};