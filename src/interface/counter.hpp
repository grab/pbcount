#pragma once

/* inclusions *****************************************************************/

#include "formula.hpp"
#include "join.hpp"
#include "visual.hpp"
#include "pbformula.hpp"

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
  virtual void constructJoinTree(const PBformula &pb) = 0; // unimplemented
  void setJoinTree(const PBformula &pb); // unimplemeneted

  ADD countSubtree(JoinNode *joinNode, const PBformula &pb, Set<Int> &projectedPbVars); 
  Float countJoinTree(const PBformula &pb); // unimplemented

  virtual Float computeModelCount(const PBformula &pb) = 0; 
  Float getModelCount(const PBformula &pb); 

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
