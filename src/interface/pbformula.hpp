#pragma once

/* inclusions *****************************************************************/
#include "util.hpp"
#include "graph.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <unordered_map>
#include "formula.hpp"
#include "pbclause.hpp"

#include <boost/process.hpp>
#include <boost/integer/common_factor.hpp>
namespace bp = boost::process;
namespace bi = boost::integer;

/* constants for PB*/
extern const string &PB_COMMENT_WORD;
extern const string &PB_EOL_WORD;
const string SAT_SOLVER_CMD = "timeout 2s ./roundingsat --print-sol=0 --verbosity=0 ";


/* classes ********************************************************************/

class PBformula
{
protected:
  //   WeightFormat weightFormat;
  Int declaredVarCount = DUMMY_MIN_INT;    // in opb file
  Int declaredClauseCount = DUMMY_MIN_INT; // in opb file
  Map<Int, Float> literalWeights;
  vector<PBclause> clauses;
  vector<Int> apparentVars; // vars appearing in clauses, ordered by 1st appearance
  bool projectedFormula; // true if pb input file contains ind lines (projected counting set)
  Set<Int> projectionVariableSet;
  Map<Int, bool> inferredAssignment; // from unit constraints
  bool inferredUnsat = false;
  bool solverDetected = false;

  // used for heuristics, currently copied over from addmc formula
  void updateApparentVars(Int literal); // adds var to apparentVars
  void addClause(const PBclause &clause); // writes: clauses, apparentVars
  Graph getGaifmanGraph() const;
  vector<Int> getAppearanceVarOrdering() const;
  vector<Int> getDeclarationVarOrdering() const;
  vector<Int> getRandomVarOrdering() const;
  vector<Int> getLexpVarOrdering() const;
  vector<Int> getLexmVarOrdering() const;
  vector<Int> getMcsVarOrdering() const;

public:
  bool isProjected() const;
  const Set<Int> &getProjectionVariableSet() const;
  vector<Int> getVarOrdering(VarOrderingHeuristic varOrderingHeuristic, bool inverse) const;
  Int getDeclaredVarCount() const;
  Int getDeclaredClauseCount() const;
  Map<Int, Float> getLiteralWeights() const;
  Int getEmptyClauseIndex() const; // first (nonnegative) index if found else DUMMY_MIN_INT
  const vector<PBclause> &getClauses() const;
  const vector<Int> &getApparentVars() const;
  Map<Int, bool> getInferredAssignments() const;
  void printLiteralWeights() const;
  void printClauses() const;
  PBformula(string &filePath, bool isWeighted=false);
  void inferAssignment(PBclause &clause, Map<Int, bool> &inferredAssignmentMap);
  void preprocess();
  Set<Int> probeInferAssign(Map<Int, bool> &inferredAssignmentMap, vector<PBclause> &clauseVector);
  PBclause applyProbe(PBclause const &clause, Int decLit);
  std::pair<Int, bool> inferProbe(PBclause& clause);
  void addCheckInferredAssign(Int inferredLit);
  void addCheckInferredAssign(Int inferredLit, Map<Int, bool>&currentInferredAssignment);
  bool isUnsat();
  bool satTest(vector<PBclause> &clauseVector, vector<Int> testLits);
  Set<Int> failedLiteralTest(Map<Int, bool> &inferredAssignmentMap, vector<PBclause> &clauseVector);
  void detectSolver();
  bool checkAlwaysTrue(PBclause& clause);

};