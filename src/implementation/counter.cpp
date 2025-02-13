/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

/* inclusions *****************************************************************/

#include "../interface/counter.hpp"

/* namespaces *****************************************************************/

/* namespace dd ***************************************************************/

Float diagram::getTerminalValue(const ADD &terminal) {
  DdNode *node = terminal.getNode();
  return (node -> type).value;
}

Float diagram::countConstDdFloat(const ADD &dd) {
  ADD minTerminal = dd.FindMin();
  ADD maxTerminal = dd.FindMax();

  Float minValue = getTerminalValue(minTerminal);
  Float maxValue = getTerminalValue(maxTerminal);

  if (minValue != maxValue) {
    showError("ADD is nonconst: min value " + to_string(minValue) +
      ", max value " + to_string(maxValue));
  }

  return minValue;
}

Int diagram::countConstDdInt(const ADD &dd) {
  Float value = countConstDdFloat(dd);

  if (!util::isInt(value)) showError("unweighted model count is not int");

  return value;
}

void diagram::printMaxDdVarCount(Int maxDdVarCount) {
  util::printRow("maxAddVarCount", maxDdVarCount);
}


/* pb classes ********************************************************************/

/* class Counter **************************************************************/

WeightFormat PBCounter::weightFormat;

void PBCounter::handleSignals(int signal) {
  cout << "\n";
  util::printDuration(startTime);
  cout << "\n";

  util::printSolutionLine(weightFormat, 0, 0, 0);
  showError("received system signal " + to_string(signal) + "; printed dummy model count");
}

void PBCounter::writeDotFile(ADD &dd, const string &dotFileDir) {
  writeDd(mgr, dd, dotFileDir + "dd" + to_string(dotFileIndex) + ".dot");
  dotFileIndex++;
}

const vector<Int> &PBCounter::getDdVarOrdering() const {
  return this->ddVarToPbVarMap;
}

void PBCounter::orderDdVars(const PBformula &pb) {
  ddVarToPbVarMap = pb.getVarOrdering(ddVarOrderingHeuristic, inverseDdVarOrdering);
  for (Int ddVar = 0; ddVar < ddVarToPbVarMap.size(); ddVar++) {
    Int pbVar = ddVarToPbVarMap.at(ddVar);
    pbVarToDdVarMap[pbVar] = ddVar;
    mgr.addVar(ddVar); // creates ddVar-th ADD var
  }
}

ADD PBCounter::getClauseDd(const PBclause &clause) const {
  ADD clauseDD;
  if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::TOPDOWN) {
    clauseDD = getClauseDdTD(clause);
  } else if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::DYNAMIC) {
    clauseDD = getClauseDdDynamic(clause);
  } else if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::BOTTOMUP){
    clauseDD = getClauseDdBU(clause);
  } else if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::OPT_BOTTOMUP) {
    PBclause optClause = getOptimizedClauseBU(clause);
    clauseDD = getClauseDdBU(optClause);
  } else if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::OPT_TOPDOWN) {
    PBclause optClause = getOptimizedClauseTD(clause);
    clauseDD = getClauseDdTD(optClause);
  } else {
    throw MyError("Invalid clause compilation heuristic option, please select valid option.", false);
  }
  return clauseDD;
}

ADD PBCounter::getClauseDdTD(const PBclause &clause) const {
  if (clause.lits.size() <= 0) {
    std::cout << "clause is of size zero, cannot get clause DD TD" << std::endl;
  }
  Int firstCoeff = clause.coeffs.at(0);
  bool remainCoeffPositive = firstCoeff >= 0;
  Int firstLit = clause.lits.at(0);
  Int ddVar = pbVarToDdVarMap.at(std::abs(firstLit));
  ADD clauseDd = mgr.addVar(ddVar);
  if (firstLit < 0) {
    clauseDd = ~clauseDd;
  }
  return clauseDd.Ite(
      getClauseDdTDHelper(clause, 1, clause.coeffs.at(0), remainCoeffPositive),
      getClauseDdTDHelper(clause, 1, 0, remainCoeffPositive));
}
ADD PBCounter::getClauseDdTDHelper(const PBclause &clause, Int idx,
                                   Int currentVal,
                                   bool remainCoeffPositive) const {
  if (idx >= (clause.coeffs.size())) { remainCoeffPositive = true; };
  if (!remainCoeffPositive && idx < clause.coeffs.size()) {
    if (clause.coeffs.at(idx) >= 0) {
      remainCoeffPositive = true;
    }
  }
  if (clause.equals && currentVal > clause.clauseConsVal &&
      remainCoeffPositive) {
    return mgr.addZero();
  } else if (!clause.equals && currentVal >= clause.clauseConsVal &&
             remainCoeffPositive) {
    return mgr.addOne();
  } else if (idx < clause.lits.size()) {
    Int ddVar = pbVarToDdVarMap.at(std::abs(clause.lits.at(idx)));
    Int litCoeff = clause.coeffs.at(idx);
    ADD literalDd = mgr.addVar(ddVar);
    if (clause.lits.at(idx) < 0) { literalDd = ~literalDd; };
    return literalDd.Ite(
          getClauseDdTDHelper(clause, idx + 1, currentVal + litCoeff,
                              remainCoeffPositive),
          getClauseDdTDHelper(clause, idx + 1, currentVal,
                              remainCoeffPositive));
  } else {
    if (clause.equals && currentVal == clause.clauseConsVal) { return mgr.addOne(); }; 
    return mgr.addZero();
  }
}

ADD PBCounter::getClauseDdBU(const PBclause &clause) const {
  ADD clauseDd = mgr.addZero();
  for (Int i=0; i < clause.lits.size(); i++) {
    Int ddVar = pbVarToDdVarMap.at(std::abs(clause.lits.at(i)));
    Int litCoeff = clause.coeffs.at(i);
    if (clause.lits.at(i) < 0) {
      ADD literalDd = ~mgr.addVar(ddVar) * mgr.constant(litCoeff);
      clauseDd += literalDd;
    } else {
      ADD literalDd = mgr.addVar(ddVar) * mgr.constant(litCoeff);
      clauseDd += literalDd;
    }
  }
  if (clause.equals) {
    // if pb clause is of the '=' type
    clauseDd -= mgr.constant(clause.clauseConsVal);
    clauseDd = clauseDd.Cmpl();
  } else {
    // if pb clause is of '>=' type
    clauseDd -= mgr.constant(clause.clauseConsVal - 1);
    clauseDd = clauseDd.Threshold(mgr.constant(1));
    clauseDd = clauseDd.Cmpl();
    clauseDd = clauseDd.Cmpl();
  }
  return clauseDd;
}

ADD PBCounter::getClauseDdDynamic(const PBclause &clause) const {
  // heuristics for dynamically selecting how to compile dd for a given clause
  bool bottomUp = true;
  float uniqueThreshold = 0.25;
  int clauseLength = clause.coeffs.size();
  double earlyTerminationPercentile = 0.3; // 30th percentile

  Int earlyTerminationThreshold = clause.coeffs.at(
      (int) std::floor(clauseLength * earlyTerminationPercentile));

  // compute min, max, mean and mode of coefficients
  Int min, max, mode, quarter;
  min = clause.coeffs.at(0);
  max = clause.coeffs.at(clauseLength-1);
  mode = clause.coeffs.at(clauseLength/2);
  quarter = clause.coeffs.at(clauseLength/4);

  // getting an estimate of collision rate quickly
  double totalCollision = 0;
  PBclause optimizedBUClause = getOptimizedClauseBU(clause);
  std::unordered_set<Int> uniqueDualSumSet;
  for (int i = 0; i < optimizedBUClause.coeffs.size(); i++) {
    if (i + 1 >= optimizedBUClause.coeffs.size()) {
      uniqueDualSumSet.insert(abs(optimizedBUClause.coeffs.at(i)));
      totalCollision += 1;
    } else {
      uniqueDualSumSet.insert(abs(optimizedBUClause.coeffs.at(i)) - abs(optimizedBUClause.coeffs.at(i+1)));
      uniqueDualSumSet.insert(-abs(optimizedBUClause.coeffs.at(i)) + abs(optimizedBUClause.coeffs.at(i+1)));
      totalCollision += 2;
    }
  }

  double dualSumUniqueness = (double) uniqueDualSumSet.size() / totalCollision;

  // integer division should be sufficient
  Int mean = std::accumulate(clause.coeffs.begin(), clause.coeffs.end(), (Int) 0) / clauseLength;
  // compute number of unique absolute coefficients
  std::unordered_set<Int> absUniqueSet;
  for (Int coefficient : clause.coeffs) {
    absUniqueSet.insert(coefficient);
  }
  double numAbsUnique = absUniqueSet.size();
  double absUniqueness = numAbsUnique / double(clauseLength) ;

  if (clauseLength <= 25 && clause.clauseConsVal < quarter) {
    bottomUp = false;
  } else if (dualSumUniqueness >= 0.85 && absUniqueness >= 0.9 && clause.clauseConsVal < quarter) {
    bottomUp = false;
  } else {
    bottomUp = true;
  }

  // optimizing the clause coefficients for each type of compilation technique
  ADD clauseDd;
  if (bottomUp){
    // bottom up
    clauseDd = getClauseDdBU(optimizedBUClause);
  } else {
    // top down
    PBclause optimizedClause = getOptimizedClauseTD(clause);
    clauseDd = getClauseDdTD(optimizedClause);
  }
  return clauseDd;
}

PBclause PBCounter::getOptimizedClauseBU(const PBclause &clause) const {
  PBclause optimizedClause;
  optimizedClause.equals = clause.equals;
  optimizedClause.clauseConsVal = clause.clauseConsVal;
  std::vector<std::pair<Int, Int>> coeffLitPairVect;
  util::zip(clause.coeffs, clause.lits, coeffLitPairVect);
  std::sort(
          std::begin(coeffLitPairVect), std::end(coeffLitPairVect),
          [&](const auto &a, const auto &b) { return std::abs(a.first) < std::abs(b.first); });
  // traverse and change signs of coefficients
  bool isPositive = coeffLitPairVect.at(0).first >= 0;
  for (Int i=0; i<coeffLitPairVect.size(); i++) {
    if ((coeffLitPairVect.at(i).first >= 0) != isPositive) {
      optimizedClause.clauseConsVal -= coeffLitPairVect.at(i).first;
      coeffLitPairVect.at(i).first = -coeffLitPairVect.at(i).first;
      coeffLitPairVect.at(i).second = -coeffLitPairVect.at(i).second;
      isPositive = !isPositive;
    } else {
      isPositive = !isPositive;
    }
  }
  vector<Int> optimzedCoeffVector(coeffLitPairVect.size());
  vector<Int> optimizedLitVector(coeffLitPairVect.size());
  util::unzip(coeffLitPairVect, optimzedCoeffVector, optimizedLitVector);
  optimizedClause.coeffs = optimzedCoeffVector;
  optimizedClause.lits = optimizedLitVector;
  return optimizedClause;
}

PBclause PBCounter::getOptimizedClauseTD(const PBclause &clause) const {
  PBclause optimizedClause;
  optimizedClause.equals = clause.equals;
  optimizedClause.clauseConsVal = clause.clauseConsVal;
  std::vector<std::pair<Int, Int>> coeffLitPairVect;
  util::zip(clause.coeffs, clause.lits, coeffLitPairVect);
  // reverse sorting, beacuse largest coefficient first allows for early termination
  std::sort(
          std::begin(coeffLitPairVect), std::end(coeffLitPairVect),
          [&](const auto &a, const auto &b) { return std::abs(a.first) > std::abs(b.first); });
  for (Int i=0; i<coeffLitPairVect.size(); i++) {
    if (coeffLitPairVect.at(i).first < 0) {
      // flip to positive
      optimizedClause.clauseConsVal -= coeffLitPairVect.at(i).first;
      coeffLitPairVect.at(i).first = -coeffLitPairVect.at(i).first;
      coeffLitPairVect.at(i).second = -coeffLitPairVect.at(i).second;
    }
  }
  vector<Int> optimzedCoeffVector(coeffLitPairVect.size());
  vector<Int> optimizedLitVector(coeffLitPairVect.size());
  util::unzip(coeffLitPairVect, optimzedCoeffVector, optimizedLitVector);
  optimizedClause.coeffs = optimzedCoeffVector;
  optimizedClause.lits = optimizedLitVector;
  return optimizedClause;
}

void PBCounter::abstract(ADD &dd, Int ddVar, const Map<Int, Float> &literalWeights) {
  Int pbVar = ddVarToPbVarMap.at(ddVar);
  ADD positiveWeight = mgr.constant(literalWeights.at(pbVar));
  ADD negativeWeight = mgr.constant(literalWeights.at(-pbVar));
  dd = positiveWeight * dd.Compose(mgr.addOne(), ddVar) + negativeWeight * dd.Compose(mgr.addZero(), ddVar);
}

void PBCounter::abstractCube(ADD &dd, const Set<Int> &ddVars, const Map<Int, Float> &literalWeights) {
  for (Int ddVar :ddVars) {
    abstract(dd, ddVar, literalWeights);
  }
}

void PBCounter::projectAbstract(ADD &dd, Int ddVar,
                                const Map<Int, Float> &literalWeights,
                                const Set<Int> &projectionVariableSet) {
  Int pbVar = ddVarToPbVarMap.at(ddVar);
  if (isFound(pbVar, projectionVariableSet)) {
    abstract(dd, ddVar, literalWeights);
  } else {
    dd = dd.OrAbstract(mgr.addVar(ddVar));
  }
}

void PBCounter::projectAbstractCube(ADD &dd, const Set<Int> &ddVars,
                                    const Map<Int, Float> &literalWeights,
                                    const Set<Int> &projectionVariableSet) {
  for (Int ddVar : ddVars) {
    projectAbstract(dd, ddVar, literalWeights, projectionVariableSet);
  }
}

void PBCounter::printJoinTree(const PBformula &pb) const {
  cout << PROBLEM_WORD << " " << JT_WORD << " " << pb.getDeclaredVarCount() << " " << joinRoot->getTerminalCount() << " " << joinRoot->getNodeCount() << "\n";
  joinRoot->printSubtree();
}

void PBCounter::setJoinTree(const PBformula &pb) {
  if (pb.getClauses().empty()) { // empty pb
    joinRoot = new JoinNonterminal(vector<JoinNode *>());
    return;
  }

  Int i = pb.getEmptyClauseIndex();
  if (i != DUMMY_MIN_INT) { // empty clause found
    showWarning("clause " + to_string(i + 1) + " of pb is empty (1-indexing); generating dummy join tree");
    joinRoot = new JoinNonterminal(vector<JoinNode *>());
  }
  else {
    constructJoinTree(pb);
  }
}

ADD PBCounter::countSubtree(JoinNode *joinNode, const PBformula &pb, Set<Int> &projectedPbVars) {
  if (joinNode->isTerminal()) {
    return getClauseDd(pb.getClauses().at(joinNode->getNodeIndex()));
  }
  else {
    ADD dd = mgr.addOne();
    for (JoinNode *child : joinNode->getChildren()) {
      dd *= countSubtree(child, pb, projectedPbVars);
    }
    for (Int pbVar : joinNode->getProjectableCnfVars()) {
      projectedPbVars.insert(pbVar);

      Int ddVar = pbVarToDdVarMap.at(pbVar);
      abstract(dd, ddVar, pb.getLiteralWeights());
    }
    return dd;
  }
}

Float PBCounter::countJoinTree(const PBformula &pb) {
  Int i = pb.getEmptyClauseIndex();
  if (i != DUMMY_MIN_INT) { // empty clause found
    showWarning("clause " + to_string(i + 1) + " of cnf is empty (1-indexing)");
    return 0;
  }
  else {
    orderDdVars(pb);

    Set<Int> projectedPbVars;
    ADD dd = countSubtree(static_cast<JoinNode *>(joinRoot), pb, projectedPbVars);

    Float modelCount = diagram::countConstDdFloat(dd);
    modelCount = util::adjustModelCount(modelCount, projectedPbVars, pb.getLiteralWeights(), pb.getInferredAssignments());
    return modelCount;
  }
}

Float PBCounter::getModelCount(const PBformula &pb) {
  if (pb.getClauses().empty()) {
    // all clauses removed during preprocessing (either assignment inferred or free variable)
    // unsat formula should have been handled prior to calling this
    Float modelCount = 1.0;
    Set<Int> emptySet; // empty support of diagram
    if (pb.isProjected()) {
      modelCount = util::adjustProjectedModelCount(modelCount, emptySet, pb.getLiteralWeights(), pb.getProjectionVariableSet(), pb.getInferredAssignments());
    } else {
      modelCount = util::adjustModelCount(modelCount, emptySet, pb.getLiteralWeights(), pb.getInferredAssignments());
    }
    return modelCount;
  }

  Int i = pb.getEmptyClauseIndex();
  if (i != DUMMY_MIN_INT) { // empty clause found
    showWarning("clause " + to_string(i + 1) + " of pb is empty (1-indexing)");
    return 0;
  }
  else {
    return computeModelCount(pb);
  }
}

void PBCounter::output(const string &filePath, WeightFormat weightFormat, OutputFormat outputFormat, PreprocessingConfig preprocessingConfig) {
  PBCounter::weightFormat = weightFormat;

  string temp_filepath = filePath;
  PBformula pb(temp_filepath, weightFormat == WeightFormat::WEIGHTED);
  if (preprocessingConfig != PreprocessingConfig::OFF) {
    pb.detectSolver();
    printComment("Simplifying...", 1);
    pb.preprocess(); 
  }
  bool unsat = pb.isUnsat();
  printComment("Computing output...", 1);

  signal(SIGINT, handleSignals); // Ctrl c
  signal(SIGTERM, handleSignals); // timeout

  // if inferred unsat in preprocessing, return 0 count
  if (unsat) { 
    util::printSolutionLine(weightFormat, 0); 
    return;
  }

  switch (outputFormat) {
    case OutputFormat::JOIN_TREE: {
      setJoinTree(pb);
      printThinLine();
      printJoinTree(pb);
      printThinLine();
      break;
    }
    case OutputFormat::MODEL_COUNT: {
      util::printSolutionLine(weightFormat, getModelCount(pb));
      break;
    }
    default: {
      showError("no such outputFormat");
    }
  }
}

/* class PBJoinTreeCounter ******************************************************/

void PBJoinTreeCounter::constructJoinTree(const PBformula &pb) {}

Float PBJoinTreeCounter::computeModelCount(const PBformula &pb) {
  if (pb.isProjected()) {
    showError(PROJECTED_COUNTING_UNSUPPORTED_STR);
  }
  bool testing = false;
  // testing = true;
  if (testing) {
    printJoinTree(pb);
  }

  return countJoinTree(pb);
}

PBJoinTreeCounter::PBJoinTreeCounter(const string &jtFilePath, Float jtWaitSeconds, VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;

  JoinTreeReader joinTreeReader(jtFilePath, jtWaitSeconds);
  joinRoot = joinTreeReader.getJoinTreeRoot();
}

/* class MonolithicCounter ****************************************************/

void PBMonolithicCounter::setMonolithicClauseDds(vector<ADD> &clauseDds, const PBformula &pb) {
  clauseDds.clear();
  
  for (const PBclause &clause : pb.getClauses()) {
    ADD clauseDd = getClauseDd(clause);
    clauseDds.push_back(clauseDd);
  }
}

void PBMonolithicCounter::setPbDd(ADD &pbDd, const PBformula &pb) {
  vector<ADD> clauseDds;

  setMonolithicClauseDds(clauseDds, pb);
  pbDd = mgr.addOne();

  for (const ADD &clauseDd : clauseDds) {
    pbDd &= clauseDd; // operator& is operator* in class ADD
  }
}

void PBMonolithicCounter::constructJoinTree(const PBformula &pb) {
  vector<JoinNode *> terminals;
  for (Int clauseIndex = 0; clauseIndex < pb.getClauses().size(); clauseIndex++) {
    terminals.push_back(new JoinTerminal());
  }

  vector<Int> projectablePbVars = pb.getApparentVars();

  joinRoot = new JoinNonterminal(terminals, Set<Int>(projectablePbVars.begin(), projectablePbVars.end()));
}

Float PBMonolithicCounter::computeModelCount(const PBformula &pb) {
  orderDdVars(pb);

  ADD pbDd;
  setPbDd(pbDd, pb);

  Set<Int> support = util::getSupport(pbDd);
  if (!pb.isProjected()) {
    for (Int ddVar : support) {
      abstract(pbDd, ddVar, pb.getLiteralWeights());
    }
    Float modelCount = diagram::countConstDdFloat(pbDd);
    modelCount = util::adjustModelCount(modelCount, getPbVars(support), pb.getLiteralWeights(), pb.getInferredAssignments());
    return modelCount;
  } else {
    //projected counting
    // project all variables not in projection set
    for (Int ddVar : support) {
      if (!isFound(ddVarToPbVarMap.at(ddVar), pb.getProjectionVariableSet())) {
        projectAbstract(pbDd, ddVar, pb.getLiteralWeights(), pb.getProjectionVariableSet());
      }
    }
    // model count for variables in projection set
    for (Int ddVar : support) {
      if (isFound(ddVarToPbVarMap.at(ddVar), pb.getProjectionVariableSet())) {
        abstract(pbDd, ddVar, pb.getLiteralWeights());
      }
    }
    Float modelCount = diagram::countConstDdFloat(pbDd);
    modelCount = util::adjustProjectedModelCount(modelCount, getPbVars(support), pb.getLiteralWeights(), pb.getProjectionVariableSet(), pb.getInferredAssignments());
    return modelCount;
  }
}

PBMonolithicCounter::PBMonolithicCounter(VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
}

/* class PBFactoredCounter ******************************************************/

/* class PBLinearCounter ******************************************************/

void PBLinearCounter::fillProjectablePbVarSets(const vector<PBclause> &clauses) {
  projectablePbVarSets = vector<Set<Int>>(clauses.size(), Set<Int>());

  Set<Int> placedPbVars; // cumulates vars placed in projectableCnfVarSets so far
  for (Int clauseIndex = clauses.size() - 1; clauseIndex >= 0; clauseIndex--) {
    Set<Int> clausePbVars = util::getClausePbVars(clauses.at(clauseIndex));

    Set<Int> placingPbVars;
    util::differ(placingPbVars, clausePbVars, placedPbVars);
    projectablePbVarSets[clauseIndex] = placingPbVars;
    util::unionize(placedPbVars, placingPbVars);
  }
}

void PBLinearCounter::setLinearClauseDds(vector<ADD> &clauseDds, const PBformula &pb) {
  clauseDds.clear();
  clauseDds.push_back(mgr.addOne());
  for (const PBclause &clause : pb.getClauses()) {
    ADD clauseDd = getClauseDd(clause);
    clauseDds.push_back(clauseDd);
  }
}

void PBLinearCounter::constructJoinTree(const PBformula &pb) {
  const vector<PBclause> &clauses = pb.getClauses();
  fillProjectablePbVarSets(clauses);

  vector<JoinNode *> clauseNodes;
  for (Int clauseIndex = 0; clauseIndex < clauses.size(); clauseIndex++) {
    clauseNodes.push_back(new JoinTerminal());
  }

  joinRoot = new JoinNonterminal({clauseNodes.at(0)}, projectablePbVarSets.at(0));

  for (Int clauseIndex = 1; clauseIndex < clauses.size(); clauseIndex++) {
    joinRoot = new JoinNonterminal({joinRoot, clauseNodes.at(clauseIndex)}, projectablePbVarSets.at(clauseIndex));
  }
}

Float PBLinearCounter::computeModelCount(const PBformula &pb) {
  if (pb.isProjected()) {
    showError(PROJECTED_COUNTING_UNSUPPORTED_STR);
  }
  orderDdVars(pb);

  vector<ADD> factorDds;
  setLinearClauseDds(factorDds, pb);
  Set<Int> projectedPbVars;
  while (factorDds.size() > 1) {
    ADD factor1, factor2;
    util::popBack(factor1, factorDds);
    util::popBack(factor2, factorDds);

    ADD product = factor1 * factor2;
    Set<Int> productDdVars = util::getSupport(product);

    Set<Int> otherDdVars = util::getSupportSuperset(factorDds);

    Set<Int> projectingDdVars;
    util::differ(projectingDdVars, productDdVars, otherDdVars);
    abstractCube(product, projectingDdVars, pb.getLiteralWeights());
    util::unionize(projectedPbVars, getPbVars(projectingDdVars));

    factorDds.push_back(product);
  }

  Float modelCount = diagram::countConstDdFloat(util::getSoleMember(factorDds));
  modelCount = util::adjustModelCount(modelCount, projectedPbVars, pb.getLiteralWeights(), pb.getInferredAssignments());
  return modelCount;
}

PBLinearCounter::PBLinearCounter(VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
}

/* class PBNonlinearCounter ********************************************************/

void PBNonlinearCounter::printClusters(const vector<PBclause> &clauses) const {
  printThinLine();
  printComment("clusters {");
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    printComment("\t" "cluster " + to_string(clusterIndex + 1) + ":");
    for (Int clauseIndex : clusters.at(clusterIndex)) {
      cout << COMMENT_WORD << "\t\t" "clause " << clauseIndex + 1 << + ":\t";
      util::printClause(clauses.at(clauseIndex));
    }
  }
  printComment("}");
  printThinLine();
}

void PBNonlinearCounter::fillClusters(const vector<PBclause> &clauses, const vector<Int> &pbVarOrdering, bool usingMinVar) {
  clusters = vector<vector<Int>>(pbVarOrdering.size(), vector<Int>());
  for (Int clauseIndex = 0; clauseIndex < clauses.size(); clauseIndex++) {
    Int clusterIndex = usingMinVar ? util::getMinClauseRank(clauses.at(clauseIndex), pbVarOrdering) : util::getMaxClauseRank(clauses.at(clauseIndex), pbVarOrdering);
    clusters.at(clusterIndex).push_back(clauseIndex);
  }
}

void PBNonlinearCounter::printOccurrentPbVarSets() const {
  printThinLine();
  printComment("occurrentPbVarSets {");
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    const Set<Int> &pbVarSet = occurrentPbVarSets.at(clusterIndex);
    cout << COMMENT_WORD << "\t" << "cluster " << clusterIndex + 1 << ":";
    for (Int pbVar : pbVarSet) {
      cout << " " << pbVar;
    }
    cout << "\n";
  }
  printComment("}");
  printThinLine();
}

void PBNonlinearCounter::printProjectablePbVarSets() const {
  printThinLine();
  printComment("projectableCnfVarSets {");
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    const Set<Int> &pbVarSet = projectablePbVarSets.at(clusterIndex);
    cout << COMMENT_WORD << "\t" << "cluster " << clusterIndex + 1 << ":";
    for (Int pbVar : pbVarSet) {
      cout << " " << pbVar;
    }
    cout << "\n";
  }
  cout << "}\n";
  printComment("}");
  printThinLine();
}

void PBNonlinearCounter::fillPbVarSets(const vector<PBclause> &clauses, bool usingMinVar) {
  occurrentPbVarSets = vector<Set<Int>>(clusters.size(), Set<Int>());
  projectablePbVarSets = vector<Set<Int>>(clusters.size(), Set<Int>());

  Set<Int> placedPbVars; // cumulates vars placed in projectableCnfVarSets so far
  for (Int clusterIndex = clusters.size() - 1; clusterIndex >= 0; clusterIndex--) {
    Set<Int> clusterPbVars = util::getClusterPbVars(clusters.at(clusterIndex), clauses);

    occurrentPbVarSets[clusterIndex] = clusterPbVars;

    Set<Int> placingPbVars;
    util::differ(placingPbVars, clusterPbVars, placedPbVars);
    projectablePbVarSets[clusterIndex] = placingPbVars;
    util::unionize(placedPbVars, placingPbVars);
  }
}

Set<Int> PBNonlinearCounter::getProjectingDdVars(Int clusterIndex, bool usingMinVar, const vector<Int> &pbVarOrdering, const vector<PBclause> &clauses) {
  Set<Int> projectablePbVars;

  if (usingMinVar) { // bucket elimination
    projectablePbVars.insert(pbVarOrdering.at(clusterIndex));
  }
  else { // Bouquet's Method
    Set<Int> activePbVars = util::getClusterPbVars(clusters.at(clusterIndex), clauses);

    Set<Int> otherPbVars;
    for (Int i = clusterIndex + 1; i < clusters.size(); i++) {
      util::unionize(otherPbVars, util::getClusterPbVars(clusters.at(i), clauses));
    }

    util::differ(projectablePbVars, activePbVars, otherPbVars);
  }

  Set<Int> projectingDdVars;
  for (Int cnfVar : projectablePbVars) {
    projectingDdVars.insert(pbVarToDdVarMap.at(cnfVar));
  }
  return projectingDdVars;
}

void PBNonlinearCounter::fillDdClusters(const vector<PBclause> &clauses, const vector<Int> &pbVarOrdering, bool usingMinVar) {
  fillClusters(clauses, pbVarOrdering, usingMinVar);
  if (verbosityLevel >= 2) printClusters(clauses);

  ddClusters = vector<vector<ADD>>(clusters.size(), vector<ADD>());
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    for (Int clauseIndex : clusters.at(clusterIndex)) {
      ADD clauseDd = getClauseDd(clauses.at(clauseIndex));
      ddClusters.at(clusterIndex).push_back(clauseDd);
    }
  }
}

void PBNonlinearCounter::fillProjectingDdVarSets(const vector<PBclause> &clauses, const vector<Int> &pbVarOrdering, bool usingMinVar) {
  fillDdClusters(clauses, pbVarOrdering, usingMinVar);

  projectingDdVarSets = vector<Set<Int>>(clusters.size(), Set<Int>());
  for (Int clusterIndex = 0; clusterIndex < ddClusters.size(); clusterIndex++) {
    projectingDdVarSets[clusterIndex] = getProjectingDdVars(clusterIndex, usingMinVar, pbVarOrdering, clauses);
  }
}

Int PBNonlinearCounter::getTargetClusterIndex(Int clusterIndex) const {
  const Set<Int> &remainingPbVars = occurrentPbVarSets.at(clusterIndex);
  for (Int i = clusterIndex + 1; i < clusters.size(); i++) {
    if (!util::isDisjoint(occurrentPbVarSets.at(i), remainingPbVars)) {
      return i;
    }
  }
  return DUMMY_MAX_INT;
}

Int PBNonlinearCounter::getNewClusterIndex(const ADD &abstractedClusterDd, const vector<Int> &pbVarOrdering, bool usingMinVar) const {
  if (usingMinVar) {
    return util::getMinDdRank(abstractedClusterDd, ddVarToPbVarMap, pbVarOrdering);
  }
  else {
    const Set<Int> &remainingDdVars = util::getSupport(abstractedClusterDd);
    for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
      if (!util::isDisjoint(projectingDdVarSets.at(clusterIndex), remainingDdVars)) {
        return clusterIndex;
      }
    }
    return DUMMY_MAX_INT;
  }
}
Int PBNonlinearCounter::getNewClusterIndex(const Set<Int> &remainingDdVars) const { // #MAVC
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    if (!util::isDisjoint(projectingDdVarSets.at(clusterIndex), remainingDdVars)) {
      return clusterIndex;
    }
  }
  return DUMMY_MAX_INT;
}

void PBNonlinearCounter::constructJoinTreeUsingListClustering(const PBformula &pb, bool usingMinVar) {
  vector<Int> pbVarOrdering = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
  const vector<PBclause> &clauses = pb.getClauses();

  fillClusters(clauses, pbVarOrdering, usingMinVar);
  if (verbosityLevel >= 2) printClusters(clauses);

  fillPbVarSets(clauses, usingMinVar);
  if (verbosityLevel >= 2) {
    printOccurrentPbVarSets();
    printProjectablePbVarSets();
  }

  vector<JoinNode *> terminals;
  for (Int clauseIndex = 0; clauseIndex < clauses.size(); clauseIndex++) {
    terminals.push_back(new JoinTerminal());
  }

  /* creates cluster nodes: */
  vector<JoinNonterminal *> clusterNodes(clusters.size(), nullptr); // null node for empty cluster
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    const vector<Int> &clauseIndices = clusters.at(clusterIndex);
    if (!clauseIndices.empty()) {
      vector<JoinNode *> children;
      for (Int clauseIndex : clauseIndices) {
        children.push_back(terminals.at(clauseIndex));
      }
      clusterNodes.at(clusterIndex) = new JoinNonterminal(children);
    }
  }

  Int nonNullClusterNodeIndex = 0;
  while (clusterNodes.at(nonNullClusterNodeIndex) == nullptr) {
    nonNullClusterNodeIndex++;
  }
  JoinNonterminal *nonterminal = clusterNodes.at(nonNullClusterNodeIndex);
  nonterminal->addProjectablePbVars(projectablePbVarSets.at(nonNullClusterNodeIndex));
  joinRoot = nonterminal;

  for (Int clusterIndex = nonNullClusterNodeIndex + 1; clusterIndex < clusters.size(); clusterIndex++) {
    JoinNonterminal *clusterNode = clusterNodes.at(clusterIndex);
    if (clusterNode != nullptr) {
      joinRoot = new JoinNonterminal({joinRoot, clusterNode}, projectablePbVarSets.at(clusterIndex));
    }
  }
}

void PBNonlinearCounter::constructJoinTreeUsingTreeClustering(const PBformula &pb, bool usingMinVar) {
  vector<Int> pbVarOrdering = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
  const vector<PBclause> &clauses = pb.getClauses();

  fillClusters(clauses, pbVarOrdering, usingMinVar);
  if (verbosityLevel >= 2) printClusters(clauses);

  fillPbVarSets(clauses, usingMinVar);
  if (verbosityLevel >= 2) {
    printOccurrentPbVarSets();
    printProjectablePbVarSets();
  }

  vector<JoinNode *> terminals;
  for (Int clauseIndex = 0; clauseIndex < clauses.size(); clauseIndex++) {
    terminals.push_back(new JoinTerminal());
  }

  Int clusterCount = clusters.size();
  joinNodeSets = vector<vector<JoinNode *>>(clusterCount, vector<JoinNode *>()); // clusterIndex -> non-null nodes

  /* creates cluster nodes: */
  for (Int clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++) {
    const vector<Int> &clauseIndices = clusters.at(clusterIndex);
    if (!clauseIndices.empty()) {
      vector<JoinNode *> children;
      for (Int clauseIndex : clauseIndices) {
        children.push_back(terminals.at(clauseIndex));
      }
      joinNodeSets.at(clusterIndex).push_back(new JoinNonterminal(children));
    }
  }

  vector<JoinNode *> rootChildren;
  for (Int clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++) {
    if (joinNodeSets.at(clusterIndex).empty()) continue;

    Set<Int> projectablePbVars = projectablePbVarSets.at(clusterIndex);

    Set<Int> remainingPbVars;
    util::differ(remainingPbVars, occurrentPbVarSets.at(clusterIndex), projectablePbVars);
    occurrentPbVarSets[clusterIndex] = remainingPbVars;

    Int targetClusterIndex = getTargetClusterIndex(clusterIndex);
    if (targetClusterIndex <= clusterIndex) {
      showError("targetClusterIndex == " + to_string(targetClusterIndex) + " <= clusterIndex == " + to_string(clusterIndex));
    }
    else if (targetClusterIndex < clusterCount) { // some var remains
      util::unionize(occurrentPbVarSets.at(targetClusterIndex), remainingPbVars);

      JoinNonterminal *nonterminal = new JoinNonterminal(joinNodeSets.at(clusterIndex), projectablePbVars);
      joinNodeSets.at(targetClusterIndex).push_back(nonterminal);
    }
    else if (targetClusterIndex < DUMMY_MAX_INT) {
      showError("clusterCount <= targetClusterIndex < DUMMY_MAX_INT");
    }
    else { // no var remains
      JoinNonterminal *nonterminal = new JoinNonterminal(joinNodeSets.at(clusterIndex), projectablePbVars);
      rootChildren.push_back(nonterminal);
    }
  }
  joinRoot = new JoinNonterminal(rootChildren);
}

Float PBNonlinearCounter::countUsingListClustering(const PBformula &pb, bool usingMinVar) {
  orderDdVars(pb);

  vector<Int> pbVarOrdering = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
  const vector<PBclause> &clauses = pb.getClauses();

  fillClusters(clauses, pbVarOrdering, usingMinVar);
  if (verbosityLevel >= 2) printClusters(clauses);

  /* builds ADD for PB: */
  ADD pbDd = mgr.addOne();
  Set<Int> projectedPbVars;
  for (Int clusterIndex = 0; clusterIndex < clusters.size(); clusterIndex++) {
    /* builds ADD for cluster: */
    ADD clusterDd = mgr.addOne();
    const vector<Int> &clauseIndices = clusters.at(clusterIndex);
    for (Int clauseIndex : clauseIndices) {
      ADD clauseDd = getClauseDd(clauses.at(clauseIndex));
      clusterDd *= clauseDd;
    }

    pbDd *= clusterDd;

    Set<Int> projectingDdVars = getProjectingDdVars(clusterIndex, usingMinVar, pbVarOrdering, clauses);
    abstractCube(pbDd, projectingDdVars, pb.getLiteralWeights());
    util::unionize(projectedPbVars, getPbVars(projectingDdVars));
  }

  Float modelCount = diagram::countConstDdFloat(pbDd);
  modelCount = util::adjustModelCount(modelCount, pbVarOrdering, pb.getLiteralWeights(), pb.getInferredAssignments());
  return modelCount;
}

Float PBNonlinearCounter::countUsingTreeClustering(const PBformula &pb, bool usingMinVar) {
  orderDdVars(pb);

  vector<Int> pbVarOrdering = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
  const vector<PBclause> &clauses = pb.getClauses();

  fillProjectingDdVarSets(clauses, pbVarOrdering, usingMinVar);

  /* builds ADD for PB: */
  ADD pbDd = mgr.addOne();
  Set<Int> projectedPbVars;
  Int clusterCount = clusters.size();
  for (Int clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++) {
    const vector<ADD> &ddCluster = ddClusters.at(clusterIndex);
    if (!ddCluster.empty()) {
      ADD clusterDd = mgr.addOne();
      for (const ADD &dd : ddCluster) {
        clusterDd *= dd;
      }

      Set<Int> projectingDdVars = projectingDdVarSets.at(clusterIndex);
      if (usingMinVar && projectingDdVars.size() != 1) showError("wrong number of projecting vars (bucket elimination)");

      abstractCube(clusterDd, projectingDdVars, pb.getLiteralWeights());
      util::unionize(projectedPbVars, getPbVars(projectingDdVars));

      Int newClusterIndex = getNewClusterIndex(clusterDd, pbVarOrdering, usingMinVar);

      if (newClusterIndex <= clusterIndex) {
        showError("newClusterIndex == " + to_string(newClusterIndex) + " <= clusterIndex == " + to_string(clusterIndex));
      }
      else if (newClusterIndex < clusterCount) { // some var remains
        ddClusters.at(newClusterIndex).push_back(clusterDd);
      }
      else if (newClusterIndex < DUMMY_MAX_INT) {
        showError("clusterCount <= newClusterIndex < DUMMY_MAX_INT");
      }
      else { // no var remains
        pbDd *= clusterDd;
      }
    }
  }

  Float modelCount = diagram::countConstDdFloat(pbDd);
  modelCount = util::adjustModelCount(modelCount, projectedPbVars, pb.getLiteralWeights(), pb.getInferredAssignments());
  return modelCount;
}
Float PBNonlinearCounter::countUsingTreeClustering(const PBformula &pb) { // #MAVC
  orderDdVars(pb);

  vector<Int> pbVarOrdering = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
  const vector<PBclause> &clauses = pb.getClauses();

  bool usingMinVar = false;

  fillProjectingDdVarSets(clauses, pbVarOrdering, usingMinVar);

  vector<Set<Int>> clustersDdVars; // clusterIndex |-> ddVars
  for (const auto &ddCluster : ddClusters) {
    clustersDdVars.push_back(util::getSupportSuperset(ddCluster));
  }

  Set<Int> pbDdVars;
  size_t maxDdVarCount = 0;
  Int clusterCount = clusters.size();
  for (Int clusterIndex = 0; clusterIndex < clusterCount; clusterIndex++) {
    const vector<ADD> &ddCluster = ddClusters.at(clusterIndex);
    if (!ddCluster.empty()) {
      Set<Int> clusterDdVars = clustersDdVars.at(clusterIndex);

      maxDdVarCount = std::max(maxDdVarCount, clusterDdVars.size());

      Set<Int> projectingDdVars = projectingDdVarSets.at(clusterIndex);

      Set<Int> remainingDdVars;
      util::differ(remainingDdVars, clusterDdVars, projectingDdVars);

      Int newClusterIndex = getNewClusterIndex(remainingDdVars);

      if (newClusterIndex <= clusterIndex) {
        showError("newClusterIndex == " + to_string(newClusterIndex) + " <= clusterIndex == " + to_string(clusterIndex));
      }
      else if (newClusterIndex < clusterCount) { // some var remains
        util::unionize(clustersDdVars.at(newClusterIndex), remainingDdVars);
      }
      else if (newClusterIndex < DUMMY_MAX_INT) {
        showError("clusterCount <= newClusterIndex < DUMMY_MAX_INT");
      }
      else { // no var remains
        util::unionize(pbDdVars, remainingDdVars);
        maxDdVarCount = std::max(maxDdVarCount, pbDdVars.size());
      }
    }
  }

  diagram::printMaxDdVarCount(maxDdVarCount);
  showWarning("NEGATIVE_INFINITY");
  return NEGATIVE_INFINITY;
}

/* class PBBucketCounter ********************************************************/

void PBBucketCounter::constructJoinTree(const PBformula &pb) {
  bool usingMinVar = true;
  return usingTreeClustering ? PBNonlinearCounter::constructJoinTreeUsingTreeClustering(pb, usingMinVar) : PBNonlinearCounter::constructJoinTreeUsingListClustering(pb, usingMinVar);
}

Float PBBucketCounter::computeModelCount(const PBformula &pb) {
  if (pb.isProjected()) {
    showError(PROJECTED_COUNTING_UNSUPPORTED_STR);
  }
  bool usingMinVar = true;
  return usingTreeClustering ? PBNonlinearCounter::countUsingTreeClustering(pb, usingMinVar) : PBNonlinearCounter::countUsingListClustering(pb, usingMinVar);
}

PBBucketCounter::PBBucketCounter(bool usingTreeClustering, VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->usingTreeClustering = usingTreeClustering;
  this->pbVarOrderingHeuristic = pbVarOrderingHeuristic;
  this->inversePbVarOrdering = inversePbVarOrdering;
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
}

/* class PBBouquetCounter *******************************************************/

void PBBouquetCounter::constructJoinTree(const PBformula &pb) {
  bool usingMinVar = false;
  return usingTreeClustering ? PBNonlinearCounter::constructJoinTreeUsingTreeClustering(pb, usingMinVar) : PBNonlinearCounter::constructJoinTreeUsingListClustering(pb, usingMinVar);
}

Float PBBouquetCounter::computeModelCount(const PBformula &pb) {
  bool usingMinVar = false;
  if (pb.isProjected()) {
    showError(PROJECTED_COUNTING_UNSUPPORTED_STR);
  }
  return usingTreeClustering ? PBNonlinearCounter::countUsingTreeClustering(pb, usingMinVar) : PBNonlinearCounter::countUsingListClustering(pb, usingMinVar);
}

PBBouquetCounter::PBBouquetCounter(bool usingTreeClustering, VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->usingTreeClustering = usingTreeClustering;
  this->pbVarOrderingHeuristic = pbVarOrderingHeuristic;
  this->inversePbVarOrdering = inversePbVarOrdering;
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
}


/* class PBComputeGraphCounter ****************************************************/

void PBComputeGraphCounter::orAbstract(ADD &dd, Int ddVar) {
  ADD varDd = mgr.addVar(ddVar);
  dd = dd.OrAbstract(varDd);
}
void PBComputeGraphCounter::orAbstractCube(ADD &dd, Set<Int> &ddVars) {
  for (Int ddVar : ddVars) {
    orAbstract(dd, ddVar);
  }
}

void PBComputeGraphCounter::constructJoinTree(const PBformula &pb) {
  ;
}

void PBComputeGraphCounter::eagerOrAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet) {
  vector<Int> abstractedVariables;
  for (auto element : varToClauseMap){
    Int var = element.first;
    if (!isFound(var, nonSupportVarSet)) {
      continue;
    }
    Int clauseID = 0;
    if (element.second.size() == 1) { // var only appears in one clause
      clauseID = *element.second.begin();
      ADD varADD = mgr.addVar(pbVarToDdVarMap.at(var));
      ADD clauseDD = clauseDDMap[clauseID].OrAbstract(varADD);
      clauseDDMap[clauseID] = clauseDD;
      abstractedVariables.push_back(var);
    }
  }
  for (Int var : abstractedVariables) {
    varToClauseMap.erase(var);
    nonSupportVarSet.erase(var);
  }
  for (auto &element : clauseToVarMap) {
    for (Int var : abstractedVariables) {
      // erase does not throw exception if element does not exist
      element.second.erase(var); 
    }
  }
}

void PBComputeGraphCounter::eagerAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights) {
  // modifies the ADDs in clauseDDMap
  vector<Int> abstractedVariables;
  for (auto element : varToClauseMap){
    Int var = element.first;
    Int clauseID = 0;
    if (element.second.size() == 1) { // var only appears in one clause
      clauseID = *element.second.begin();
      ADD clauseDD = clauseDDMap[clauseID];
      abstract(clauseDD, pbVarToDdVarMap.at(var), literalWeights);
      clauseDDMap[clauseID] = clauseDD;
      abstractedVariables.push_back(var);
    }
  }
  // remove variable from the varToClauseMap
  for (Int var : abstractedVariables) {
    varToClauseMap.erase(var);
  }
  // remove variable from clauseToVarMap
  for (auto &element : clauseToVarMap) {
    for (Int var : abstractedVariables) {
      // erase does not throw exception if element does not exist
      element.second.erase(var); 
    }
  }
}

Map<Int, ADD> PBComputeGraphCounter::compileClauses(const PBformula &pb) {
  Map<Int, ADD> clauseDDMap;
  for (const PBclause &clause : pb.getClauses()) {
    clauseDDMap[clause.clauseId] = getClauseDd(clause);
  }
  return clauseDDMap;
}

Int PBComputeGraphCounter::getLeastCommonNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const {
  if (nonSupportVarSet.size() == 0) {
    throw MyError("Cannot call getLeastCommonNonSupportVar with empty nonSupportVarSet", true);
  }
  Int lcv = -1;
  Int minNumClauses = DUMMY_MAX_INT;
  for (auto element : varToClauseMap) {
    if (isFound(element.first, nonSupportVarSet)){
      if (minNumClauses > element.second.size()) {
        minNumClauses = element.second.size();
        lcv = element.first;
      }
      if (minNumClauses == element.second.size()) {
        if (element.first < lcv) {
          lcv = element.first;
        }
      }
    }
  }
  return lcv;
}

Int PBComputeGraphCounter::getLeastCommonVar(Map<Int, Set<Int>> &varToClauseMap) const {
  Int lcv = -1;
  Int minNumClauses = DUMMY_MAX_INT;
  for (auto element : varToClauseMap) {
    if (minNumClauses > element.second.size()) {
      minNumClauses = element.second.size();
      lcv = element.first;
    }
    if (minNumClauses == element.second.size()) {
      if (element.first < lcv) {
        lcv = element.first;
      }
    }
  }
  return lcv;
}

Int PBComputeGraphCounter::getNextMergeNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const {
  if (nonSupportVarSet.size() == 0) {
    throw MyError("Cannot call getNextMergeNonSupportVar with empty nonSupportVarSet", true);
  }
  if (!preferredVariableOrdering.empty()) {
    Int cVar;
    Int cOrderIndex = DUMMY_MAX_INT;
    Int currentBestVar = -1;
    Int currentBestVarOrderIndex = DUMMY_MAX_INT;
    for (auto& element : varToClauseMap) {
      cVar = element.first;
      if(isFound(cVar, nonSupportVarSet)){
        if(currentBestVar == -1) { 
          currentBestVar = cVar;
        }
        cOrderIndex = preferredVariableOrdering.at(cVar);
        if (cOrderIndex < currentBestVarOrderIndex) {
          currentBestVar = cVar;
        }
      }
    }
    return currentBestVar;
  }
  return getLeastCommonNonSupportVar(varToClauseMap, nonSupportVarSet);
}

Int PBComputeGraphCounter::getNextMergeSupportVar(Map<Int, Set<Int>> &varToClauseMap) const {
  if (!preferredVariableOrdering.empty()) {
    Int cVar;
    Int cOrderIndex = DUMMY_MAX_INT;
    Int currentBestVar = -1;
    Int currentBestVarOrderIndex = DUMMY_MAX_INT;
    for (auto& element : varToClauseMap) {
      cVar = element.first;
      if(currentBestVar == -1) { 
        currentBestVar = cVar;
      }
      cOrderIndex = preferredVariableOrdering.at(cVar);
      if (cOrderIndex < currentBestVarOrderIndex) {
        currentBestVar = cVar;
      }
    }
    return currentBestVar;
  }
  return getLeastCommonVar(varToClauseMap);
}


Set<Int> PBComputeGraphCounter::getClauseExclusiveVarIntersection(Set<Int> &clauseSet, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, Set<Int>> &varToClauseMap) {
  Set<Int> commonVarSet = clauseToVarMap.at(*clauseSet.begin());
  for (Int clauseID : clauseSet) {
    commonVarSet = util::setIntersection(commonVarSet, clauseToVarMap.at(clauseID));
  }
  vector<Int> nonExclusiveVariables;
  for (Int var : commonVarSet) {
    if (clauseSet.size() != varToClauseMap.at(var).size()) {
      nonExclusiveVariables.push_back(var);
      continue;
    }
    bool match = true;
    for (Int candidateClauseID : varToClauseMap.at(var)) {
      if (!isFound(candidateClauseID, clauseSet)) {
        match = false;
      }
    }
    if (!match) { nonExclusiveVariables.push_back(var); }
  }
  // remove non exclusive elements
  for (Int var : nonExclusiveVariables) {
    commonVarSet.erase(var);
  }
  return commonVarSet;
}

void PBComputeGraphCounter::mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, const Map<Int, Float> &literalWeights) {
  ADD dd = mgr.addOne();
  Set<Int> clauseSet = varToClauseMap.at(targetPBVar);
  for (Int clauseID : clauseSet) {
    dd &= clauseDDMap[clauseID];
  }
  // find common variables apart from target pb var (that also does not exist elsewhere)
  Set<Int> commonExVarSet = getClauseExclusiveVarIntersection(clauseSet, clauseToVarMap, varToClauseMap); //common vars should include target pb var
  // project out all common such common variables
  Set<Int> ddVarSet;
  for (Int pbVar : commonExVarSet) {
    ddVarSet.insert(pbVarToDdVarMap.at(pbVar));
  }
  abstractCube(dd, ddVarSet, literalWeights);
  // add dd with variable set to mapping
  // delete the projected out vars from the map and update remaining vars
  for (Int pbVar : commonExVarSet) {
    varToClauseMap.erase(pbVar);
  }
  // erase the entire clause and reuse an old clause id for the merged dd
  // get one clause id to give dd
  Int newDDClauseID = *clauseSet.begin();
  Set<Int> newVarSet;
  for (Int clauseID : clauseSet) {
    for (Int pbVar : clauseToVarMap.at(clauseID)) {
      if (!isFound(pbVar, commonExVarSet)) {
        newVarSet.insert(pbVar);
      }
    }
    clauseToVarMap.erase(clauseID);
    clauseDDMap.erase(clauseID);
  }
  clauseDDMap[newDDClauseID] = dd;
  clauseToVarMap[newDDClauseID] = newVarSet;
  // update the var clause map of the non exclusive variables
  for (Int pbVar : newVarSet) {
    for (Int clauseID : clauseSet) {
      if ((clauseID != newDDClauseID) && isFound(clauseID, varToClauseMap.at(pbVar))) {
        varToClauseMap.at(pbVar).erase(clauseID);
      }
    }
    varToClauseMap.at(pbVar).insert(newDDClauseID);
  }
}

void PBComputeGraphCounter::mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet) {
  bool processedNonSupport = (nonSupportVarSet.size() == 0);
  ADD dd = mgr.addOne();
  Set<Int> clauseSet = varToClauseMap.at(targetPBVar);
  for (Int clauseID : clauseSet) {
    dd &= clauseDDMap[clauseID];
  }
  // find common variables apart from target pb var (that also does not exist elsewhere)
  Set<Int> commonExVarSet = getClauseExclusiveVarIntersection(clauseSet, clauseToVarMap, varToClauseMap); //common vars should include target pb var
  // project out all common such common variables
  if (!processedNonSupport) {
    Set<Int> nonSupportcommonExVarSet;
    for (Int pbVar : commonExVarSet) {
      if (isFound(pbVar, nonSupportVarSet)) {
        nonSupportcommonExVarSet.insert(pbVar);
      }
    }
    commonExVarSet = nonSupportcommonExVarSet;
  }

  Set<Int> ddVarSet;
  for (Int pbVar : commonExVarSet) {
    if (!processedNonSupport) {
      if (isFound(pbVar, nonSupportVarSet)) {
        ddVarSet.insert(pbVarToDdVarMap.at(pbVar));
      }
    } else {
      ddVarSet.insert(pbVarToDdVarMap.at(pbVar));
    }
  }
  if (!processedNonSupport) {
    orAbstractCube(dd, ddVarSet);
  } else {
    abstractCube(dd, ddVarSet, literalWeights);
  }
  // add dd with variable set to mapping
  // delete the projected out vars from the map and update remaining vars
  for (Int pbVar : commonExVarSet) {
    varToClauseMap.erase(pbVar);
    if (isFound(pbVar, nonSupportVarSet)) {
      nonSupportVarSet.erase(pbVar);
    }
  }
  // erase the entire clause and reuse an old clause id for the merged dd
  // get one clause id to give dd
  Int newDDClauseID = *clauseSet.begin();
  Set<Int> newVarSet;
  for (Int clauseID : clauseSet) {
    for (Int pbVar : clauseToVarMap.at(clauseID)) {
      if (!isFound(pbVar, commonExVarSet)) {
        newVarSet.insert(pbVar);
      }
    }
    clauseToVarMap.erase(clauseID);
    clauseDDMap.erase(clauseID);
  }
  clauseDDMap[newDDClauseID] = dd;
  clauseToVarMap[newDDClauseID] = newVarSet;
  // update the var clause map of the non exclusive variables
  for (Int pbVar : newVarSet) {
    for (Int clauseID : clauseSet) {
      if ((clauseID != newDDClauseID) && isFound(clauseID, varToClauseMap.at(pbVar))) {
        varToClauseMap.at(pbVar).erase(clauseID);
      }
    }
    varToClauseMap.at(pbVar).insert(newDDClauseID);
  }
}

// for setting custom merge order in compute graph counter
void PBComputeGraphCounter::setPreferredVariableOrdering(VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, const PBformula &pb) {
  switch (pbVarOrderingHeuristic) {
    case VarOrderingHeuristic::MINFILL: {
      preferredVariableOrdering.clear();
      vector<Int> varOrderVector = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
      for (int i = 0; i < varOrderVector.size(); i++) {
        preferredVariableOrdering[varOrderVector.at(i)] = i;
      }
      break;
    }
    default: {
      // showError("Unaccepted variable merge ordering for compute graph counter.");
      break;
    }
  }
}

Float PBComputeGraphCounter::computeModelCount(const PBformula &pb) {
  // setting the same for both projected and non projected
  setPreferredVariableOrdering(pbVarOrderingHeuristic, inversePbVarOrdering, pb);
  if (pb.isProjected()) { return computeProjectedModelCount(pb); }
  // set the decision diagram variable ordering
  orderDdVars(pb);
  // get the constraint graph
  MultiTypeGraph interactionGraph = pb.getInteractionGraph();
  Map<Int, Set<Int>> variableToClauseMap = interactionGraph.getVarToClauseMap();
  Map<Int, Set<Int>> clauseToVariableMap = interactionGraph.getClauseToVarMap();
  // get ADD for each clause
  Map<Int, ADD> clauseDDMap = compileClauses(pb);
  Set<Int> processVarSet;
  for (auto element : variableToClauseMap) {
    processVarSet.insert(element.first);
  }
  eagerAbstraction(clauseDDMap, variableToClauseMap, clauseToVariableMap, pb.getLiteralWeights());
  while (variableToClauseMap.size() > 0) {
    Int targetPBVar = getNextMergeSupportVar(variableToClauseMap); // take preferred ordering if there is one, otherwise getLeastCommonVar
    mergeUpdateClauses(targetPBVar, variableToClauseMap, clauseToVariableMap, clauseDDMap, pb.getLiteralWeights());
  }

  ADD pbDd = mgr.constant(1);
  if (clauseDDMap.size() > 1) {
    Int newClauseId = clauseDDMap.begin()->first;
    for (auto &element : clauseDDMap) {
      pbDd *= element.second;
    }
    clauseDDMap[newClauseId] = pbDd;
    for (auto &element : clauseDDMap) {
      if (element.first != newClauseId) {
        clauseDDMap.erase(element.first);
      }
    }
  }
  // getting the model count from the constant decision diagram
  pbDd = (clauseDDMap.begin()->second);
  Float modelCount = diagram::countConstDdFloat(pbDd);

  // adjust model count for simplifications during preprocessing
  modelCount = util::adjustModelCountCG(modelCount, pb.getLiteralWeights(), pb.getInferredAssignments());
  modelCount = util::adjustModelCountCGMissingVar(modelCount, pb.getDeclaredVarCount(), pb.getLiteralWeights(), pb.getInferredAssignments(), processVarSet);

  return modelCount;
}

Float PBComputeGraphCounter::computeProjectedModelCount(const PBformula &pb) {
  // set the decision diagram variable ordering
  orderDdVars(pb);

  Set<Int> projectionSupportVarSet = pb.getProjectionVariableSet();
  // get the constraint graph
  MultiTypeGraph interactionGraph = pb.getInteractionGraph();
  Map<Int, Set<Int>> variableToClauseMap = interactionGraph.getVarToClauseMap();
  Map<Int, Set<Int>> clauseToVariableMap = interactionGraph.getClauseToVarMap();
  // get ADD for each clause
  Map<Int, ADD> clauseDDMap = compileClauses(pb);

  Set<Int> nonSupportVarSet;
  Set<Int> processVarSet;
  for (auto element : variableToClauseMap) {
    processVarSet.insert(element.first);
    if (!isFound(element.first, projectionSupportVarSet)) {
      nonSupportVarSet.insert(element.first);
    }
  }
  eagerOrAbstraction(clauseDDMap, variableToClauseMap, clauseToVariableMap, pb.getLiteralWeights(), nonSupportVarSet);

  while (variableToClauseMap.size() > 0) {
    Int targetPBVar = -1;
    if (nonSupportVarSet.size() > 0) {
      targetPBVar = getNextMergeNonSupportVar(variableToClauseMap, nonSupportVarSet);
    } else {
      targetPBVar = getNextMergeSupportVar(variableToClauseMap);
    }
    // merge ADDs and project out variable
    mergeUpdateClauses(targetPBVar, variableToClauseMap, clauseToVariableMap, clauseDDMap, pb.getLiteralWeights(), nonSupportVarSet);
  }

  ADD pbDd = mgr.constant(1);
  if (clauseDDMap.size() > 1) {
    Int newClauseId = clauseDDMap.begin()->first;
    for (auto &element : clauseDDMap) {
      pbDd *= element.second;
    }
    clauseDDMap[newClauseId] = pbDd;
    for (auto &element : clauseDDMap) {
      if (element.first != newClauseId) {
        clauseDDMap.erase(element.first);
      }
    }
  }
  // getting the model count from the constant decision diagram
  pbDd = (clauseDDMap.begin()->second);
  Float modelCount = diagram::countConstDdFloat(pbDd);

  // adjust model count for simplifications during preprocessing
  modelCount = util::adjustProjectedModelCountCG(modelCount, pb.getLiteralWeights(), pb.getInferredAssignments(), projectionSupportVarSet);
  modelCount = util::adjustProjectedModelCountCGMissingVar(modelCount, pb.getDeclaredVarCount(), pb.getLiteralWeights(), pb.getInferredAssignments(), processVarSet, projectionSupportVarSet);

  return modelCount;
}

PBComputeGraphCounter::PBComputeGraphCounter(VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, VarOrderingHeuristic ddVarOrderingHeuristic, bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->pbVarOrderingHeuristic = pbVarOrderingHeuristic;
  this->inversePbVarOrdering = inversePbVarOrdering;
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
}

/* class PBInteractiveCounter ****************************************************/

//protected functions
Float PBInteractiveCounter::computeMaxOrderingOffsetRatio(const vector<Int>& originalOrdering, const vector<Int>& tentativeOrdering) {
  // getting the max offset variable between the two ordering, relative to length of ordering
  Int var = -1;
  Int maxOffset = -1;

  vector<Int> offsets;
  offsets.clear();

  Map<Int, Int> origVarToOrderIndexMap;
  for (Int i = 0; i < originalOrdering.size(); i++) {
    origVarToOrderIndexMap[originalOrdering.at(i)] = i;
  }
  for (Int i = 0; i < tentativeOrdering.size(); i++) {
    var = tentativeOrdering.at(i);
    if(util::isFound(var, origVarToOrderIndexMap)){
      Int offset = abs(origVarToOrderIndexMap.at(var) - i);
      offsets.push_back(offset);
      if(maxOffset < offset) {
        maxOffset = offset;
      }
    }
  }

  // std::cout << "[STATS] AllVariableDffsets: ";
  // for (Int offs : offsets) {
  //   std::cout << (Float) offs / (Float)originalOrdering.size() << " ";
  // }
  // std::cout << std::endl;

  // looking at the max offset for now, but other metrics should also be possible i.e. median
  Float maxOffsetRatio = (Float) maxOffset / (Float) originalOrdering.size();
  return maxOffsetRatio;
}

Float PBInteractiveCounter::computeMedianOrderingOffsetRatio(const vector<Int>& originalOrdering, const vector<Int>& tentativeOrdering) {
  // getting the max offset variable between the two ordering, relative to length of ordering
  Int var = -1;
  Int medianOffset = -1;

  vector<Int> offsets;
  offsets.clear();

  Map<Int, Int> origVarToOrderIndexMap;
  for (Int i = 0; i < originalOrdering.size(); i++) {
    origVarToOrderIndexMap[originalOrdering.at(i)] = i;
  }
  for (Int i = 0; i < tentativeOrdering.size(); i++) {
    var = tentativeOrdering.at(i);
    if(util::isFound(var, origVarToOrderIndexMap)){
      Int offset = abs(origVarToOrderIndexMap.at(var) - i);
      offsets.push_back(offset);
    }
  }

  std::sort(offsets.begin(), offsets.end());
  if(offsets.empty()) {
    throw MyError("Empty offsets, no variable ordering?", true);
  }
  int medianIdx = (int) ((float) offsets.size() / 2.0);
  Float medianOffsetRatio = (Float) offsets.at(medianIdx) / (Float) originalOrdering.size();
  return medianOffsetRatio;
}

void PBInteractiveCounter::orderDdVarsIncrementalWithRestart(const PBformula &pb) {
  PBformula activePb;
  Set<Int> activeVarSet = getActiveVariables(pb);
  vector<PBclause> allPBClauses = pb.getClauses();
  for (int i = 0; i < allPBClauses.size(); i++) {
    if (activeClauses.at(i)) {
      activePb.insertClause(allPBClauses.at(i)); // only insert if active clause
    }
  }
  vector<Int> newPbVarOrdering = activePb.getVarOrdering(ddVarOrderingHeuristic, inverseDdVarOrdering);

  if (ddVarToPbVarMap.empty()) {
    ddVarToPbVarMap = newPbVarOrdering;
    for (Int ddVar = 0; ddVar < ddVarToPbVarMap.size(); ddVar++) {
      Int pbVar = ddVarToPbVarMap.at(ddVar);
      pbVarToDdVarMap[pbVar] = ddVar;
      mgr.addVar(ddVar); // creates ddVar-th ADD var
    }
  } else {
    bool restart = false;
    Float offsetRatio = 0;
    if (restartOnOrderingMisalign || PRINT_ORDERING_OFFSET) {
      // determine if ordering deviated enough to restart
      vector<Int> tempPreviousOrdering;
      for (Int var : ddVarToPbVarMap) {
        if (isFound(var, activeVarSet)) {
          tempPreviousOrdering.push_back(var);
        }
      }
      offsetRatio = computeMedianOrderingOffsetRatio(tempPreviousOrdering, newPbVarOrdering);
      // offsetRatio = computeMaxOrderingOffsetRatio(tempPreviousOrdering, newPbVarOrdering);
      if (PRINT_ORDERING_OFFSET) {
        std::cout << "[STATS] Current step median variable ordering offset ratio: " << offsetRatio << std::endl;
      }
    }
    // deviation that has one variable being offset by x percent of length
    if (restartOnOrderingMisalign && offsetRatio >= ORDER_OFFSET_RESTART_THRESHOLD_RATIO) {
      restart = true;
    }
    if (restart) {
      singleClauseDdMap.clear();
      ddCache.clear();
      mgr = Cudd(); // new manager
      ddVarToPbVarMap.clear();
      pbVarToDdVarMap.clear();
      ddVarToPbVarMap = newPbVarOrdering;
      for (Int ddVar = 0; ddVar < ddVarToPbVarMap.size(); ddVar++) {
        Int pbVar = ddVarToPbVarMap.at(ddVar);
        pbVarToDdVarMap[pbVar] = ddVar;
        mgr.addVar(ddVar); // creates ddVar-th ADD var
      }
    } else {
      for (Int pbVar : newPbVarOrdering) {
        if (!isFound(pbVar, pbVarToDdVarMap)) {
          // ddVar started from 0
          Int ddVar = ddVarToPbVarMap.size();
          ddVarToPbVarMap.push_back(pbVar);
          pbVarToDdVarMap[pbVar] = ddVar;
          mgr.addVar(ddVar); // creates ddVar-th ADD var
        }
      }
    }
  }
}
// old implementaion
void PBInteractiveCounter::orderDdVarsIncremental(const PBformula &pb) {
  vector<Int> newPbVarOrdering = pb.getVarOrdering(ddVarOrderingHeuristic, inverseDdVarOrdering);
  for (Int pbVar : newPbVarOrdering) {
    if (!isFound(pbVar, pbVarToDdVarMap)) {
      // ddVar started from 0
      Int ddVar = ddVarToPbVarMap.size();
      ddVarToPbVarMap.push_back(pbVar);
      pbVarToDdVarMap[pbVar] = ddVar;
      mgr.addVar(ddVar); // creates ddVar-th ADD var
    }
  }
}
void PBInteractiveCounter::constructJoinTree(const PBformula &pb) {
  ;
}
void PBInteractiveCounter::saveDdCache(string &ddIdentifer, ADD &dd) {
  ddCache[ddIdentifer] = dd;
}
string PBInteractiveCounter::getDdIdentifierString(Set<Int> &mergedClauseSet, Set<Int> &earlyAbstractedVarSet) {
  // c1-c2-c3|v1-v2-v3 style identifier
  std::stringstream ss;
  vector<Int> elements(mergedClauseSet.begin(), mergedClauseSet.end());
  sort(elements.begin(), elements.end());
  for (int i = 0; i < elements.size(); i++) {
    ss << to_string(elements.at(i));
    if (i + 1 != elements.size()) {
      ss << "-";
    }
  }
  ss << "|";
  // elements.clear();
  elements = vector<Int>(earlyAbstractedVarSet.begin(), earlyAbstractedVarSet.end());
  sort(elements.begin(), elements.end());
  for (int i = 0; i < elements.size(); i++) {
    ss << to_string(elements.at(i));
    if (i + 1 != elements.size()) {
      ss << "-";
    }
  }
  return ss.str();
}
string PBInteractiveCounter::getDdIdentifierString(Set<Int> &mergedClauseSet, Set<Int> &earlyAbstractedVarSet, Set<Int> &earlyOrAbstractedVarSet) {
  // c1-c2-c3|v1-v2-v3|x1-x2-x3 style identifier (v in support (projection set), x not in )
  std::stringstream ss;
  vector<Int> elements(mergedClauseSet.begin(), mergedClauseSet.end());
  sort(elements.begin(), elements.end());
  for (int i = 0; i < elements.size(); i++) {
    ss << to_string(elements.at(i));
    if (i + 1 != elements.size()) {
      ss << "-";
    }
  }
  ss << "|";
  // elements.clear();
  elements = vector<Int>(earlyAbstractedVarSet.begin(), earlyAbstractedVarSet.end());
  sort(elements.begin(), elements.end());
  for (int i = 0; i < elements.size(); i++) {
    ss << to_string(elements.at(i));
    if (i + 1 != elements.size()) {
      ss << "-";
    }
  }
  ss << "|";
  elements = vector<Int>(earlyOrAbstractedVarSet.begin(), earlyOrAbstractedVarSet.end());
  sort(elements.begin(), elements.end());
  for (int i = 0; i < elements.size(); i++) {
    ss << to_string(elements.at(i));
    if (i + 1 != elements.size()) {
      ss << "-";
    }
  }
  return ss.str();
}
vector<Set<Int>> PBInteractiveCounter::decodeIdentifier(string &ddIdentifier) {
  vector<Set<Int>> decodedData;
  int numSep = 0; // number of seperators processed
  string currentString = "";
  decodedData.push_back(Set<Int>());
  for (int i = 0; i < ddIdentifier.size(); i++) {
    if (ddIdentifier.at(i) == '|') {
      if (currentString != "") {
        decodedData.at(numSep).insert(std::stol(currentString));
      }
      numSep += 1;
      currentString = "";
      decodedData.push_back(Set<Int>());
    } else if (ddIdentifier.at(i) == '-') {
      decodedData.at(numSep).insert(std::stol(currentString));
      currentString = "";
    } else {
      currentString += ddIdentifier.at(i);
    }
  }
  // last string without any delimiters following
  if (currentString != "") {
    decodedData.at(numSep).insert(std::stol(currentString));
  }
  return decodedData;
}
PBInteractiveCounter::ComputeState PBInteractiveCounter::resolveCache(const PBformula &pb, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap) {
  // find the set of active variables

  std::cout << "Number of cache entry: " << ddCache.size() << std::endl;
  TimePoint timer = util::getTimePoint();

  Set<Int> activeClauseSet;
  Set<Int> activeVarSet;
  for (PBclause const &clause : pb.getClauses()) {
    if (activeClauses.at(clause.clauseId - 1)) {
      for (Int lit : clause.lits) {
        activeVarSet.insert(abs(lit));
      }
      activeClauseSet.insert(clause.clauseId);
    }
  }
  Set<Int> projectionSupportVarSet;
  if (pb.isProjected()) {
    projectionSupportVarSet = pb.getProjectionVariableSet();
  } else {
    projectionSupportVarSet = activeVarSet; // if not projected model counting, then all variable is support
  }
  // loop through cache id strings and figure out if resumable
  vector<ADD> candidateDds;
  vector<Set<Int>> candidateDdClauseSet;
  vector<Set<Int>> candidateDdAbstractedVariables;
  vector<Set<Int>> candidateDdOrAbstractedVariables;
  vector<string> deletionIdentifier;

  vector<Set<Int>> currentIdentifierData;
  Set<Int> cachedDDClauseSet;
  Set<Int> cachedDDAbstractedVarSet;
  Set<Int> cachedDDOrAbstractedVarSet;

  for (auto &element : ddCache) {
    string currentIdentifier = element.first;
    ADD currentDd = element.second;
    currentIdentifierData.clear();
    currentIdentifierData = decodeIdentifier(currentIdentifier);
    cachedDDClauseSet.clear();
    cachedDDClauseSet = currentIdentifierData.at(0);
    cachedDDAbstractedVarSet.clear();
    cachedDDAbstractedVarSet = currentIdentifierData.at(1);
    cachedDDOrAbstractedVarSet.clear();
    cachedDDOrAbstractedVarSet = currentIdentifierData.at(2);
    
    if (util::isSubset(cachedDDClauseSet, activeClauseSet)) {
      // check if variables that are early projected out are compatible (i.e. new constraints added later that reintroduced projected out variables)
      bool compatibleProjection = true;
      for (Int orAbstractedVar : cachedDDOrAbstractedVarSet) {
        if (projectionSupportVarSet.find(orAbstractedVar) != projectionSupportVarSet.end()) {
          // checking compatibility with specified projection set
          compatibleProjection = false;
          break;
        }
      }
      for (Int abstractedVar : cachedDDAbstractedVarSet) {
        if (projectionSupportVarSet.find(abstractedVar) == projectionSupportVarSet.end()) {
          compatibleProjection = false;
          break;
        }
      }
      // check if abstracted variable here appears in clauses that current candidate did not cover, but still active (i.e. added new clause later on with same variable)
      for (Int clauseId : activeClauseSet) {
        if (cachedDDClauseSet.find(clauseId) == cachedDDClauseSet.end()) {
          for (Int var : clauseToVarMap.at(clauseId)) {
            if ((cachedDDAbstractedVarSet.find(var) != cachedDDAbstractedVarSet.end()) || 
            (cachedDDOrAbstractedVarSet.find(var) != cachedDDOrAbstractedVarSet.end())) {
              compatibleProjection = false;
              break;
            }
          }
        }
        if (!compatibleProjection) {
          break;
        }
      } 
      if (compatibleProjection) {
        candidateDds.push_back(currentDd);
        // also push back the early abstracted variables to vector
        candidateDdAbstractedVariables.push_back(cachedDDAbstractedVarSet);
        candidateDdClauseSet.push_back(cachedDDClauseSet);
        candidateDdOrAbstractedVariables.push_back(cachedDDOrAbstractedVarSet);
      }
    }
  }
  // retrieved candidates, select subset of dd with largest non-intersecting clause coverage
  // sort dds by descending order of number of clauses represented
  vector<Int> sortingVec(candidateDdClauseSet.size());
  for (Int i = 0; i < candidateDdClauseSet.size(); i++) {
    sortingVec.push_back(i);
  }
  std::sort(sortingVec.begin(), sortingVec.end(), [&](Int i, Int j) { 
    return candidateDdClauseSet.at(i).size() > candidateDdClauseSet.at(j).size(); 
  });
  // traverse in decending order of clauses size to get filtered candidate dd list
  vector<Int> filteredDdIndices;
  Set<Int> coveredClauses;
  vector<Set<Int>> filteredDdVarSet;
  for (Int index : sortingVec) {
    bool noCommonClause = true;
    if (!candidateDdClauseSet.at(index).empty() && !coveredClauses.empty()) {
      Int indvTotalSize = candidateDdClauseSet.at(index).size() + coveredClauses.size();
      if (util::setUnion(candidateDdClauseSet.at(index), coveredClauses).size() < indvTotalSize) {
        noCommonClause = false;
      }
    }
    if (noCommonClause) {
      bool abstractedVariableCompatible = true;
      Set<Int> currentCandidateVariableSet = Set<Int>(); // setting to empty set
      for (Int candidateClauseId : candidateDdClauseSet.at(index)) {
        currentCandidateVariableSet = util::setUnion(currentCandidateVariableSet, clauseToVarMap.at(candidateClauseId));
      }
      Set<Int> currentAbstractedVariableSet = candidateDdAbstractedVariables.at(index);
      Set<Int> currentOrAbstractedVariableSet = candidateDdOrAbstractedVariables.at(index);

      // pairwise comparison against all previously selected cached dds
      for (Int filteredIndex : filteredDdIndices) {
        // get variable set
        Set<Int> selectedDdVariableSet;
        for (Int clauseId : candidateDdClauseSet.at(filteredIndex)) {
          selectedDdVariableSet = util::setUnion(selectedDdVariableSet, clauseToVarMap.at(clauseId));
        }
        Set<Int> selectedDdAbstractedVariableSet = candidateDdAbstractedVariables.at(filteredIndex);
        Set<Int> selectedDdOrAbstractedVariableSet = candidateDdOrAbstractedVariables.at(filteredIndex);
        // early abstraction/projection compatibility check -- either both projected or both not projected
        for (Int var : currentAbstractedVariableSet) {
          if ((selectedDdVariableSet.find(var) != selectedDdVariableSet.end()) && 
          !(selectedDdAbstractedVariableSet.find(var) != selectedDdAbstractedVariableSet.end())) {
            abstractedVariableCompatible = false;
            break;
          }
        }
        for (Int var : selectedDdAbstractedVariableSet) {
          if ((currentCandidateVariableSet.find(var) != currentCandidateVariableSet.end()) && 
          !(currentAbstractedVariableSet.find(var) != currentAbstractedVariableSet.end())) {
            abstractedVariableCompatible = false;
            break;
          }
        }
        for (Int var : currentOrAbstractedVariableSet) {
          if ((selectedDdVariableSet.find(var) != selectedDdVariableSet.end()) &&
          !(selectedDdOrAbstractedVariableSet.find(var) != selectedDdOrAbstractedVariableSet.end())) {
            abstractedVariableCompatible = false;
            break;
          }
        }
        for (Int var : selectedDdOrAbstractedVariableSet) {
          if ((currentCandidateVariableSet.find(var) != currentCandidateVariableSet.end()) && 
          !(currentOrAbstractedVariableSet.find(var) != currentOrAbstractedVariableSet.end())) {
            abstractedVariableCompatible = false;
            break;
          }
        }
        if (!abstractedVariableCompatible) {
          break;
        }
      }
      if (abstractedVariableCompatible) {
        filteredDdIndices.push_back(index);
        filteredDdVarSet.push_back(currentCandidateVariableSet);
        for (Int clauseId : candidateDdClauseSet.at(index)){
          coveredClauses.insert(clauseId);
        }
      }
    }
  }
  std::cout << "Retrieved " << filteredDdIndices.size() << " dds from cache" << std::endl;
  // starting from empty maps, rebuilding state from cache
  ComputeState rebuiltState;
  for (Int i = 0; i < filteredDdIndices.size(); i++) {
    Int index = filteredDdIndices.at(i);
    ADD selectedDd = candidateDds.at(index);
    Set<Int> selectedDdClauseSet = candidateDdClauseSet.at(index);
    Int newClauseId = *selectedDdClauseSet.begin();
    rebuiltState.clauseToDdMapState[newClauseId] = selectedDd;
    for (Int var : filteredDdVarSet.at(i)){
      if (!(candidateDdAbstractedVariables.at(index).find(var) != candidateDdAbstractedVariables.at(index).end()) && 
      !(candidateDdOrAbstractedVariables.at(index).find(var) != candidateDdOrAbstractedVariables.at(index).end())) {
        rebuiltState.clauseToVarMapState[newClauseId].insert(var);
      }
    }
    rebuiltState.clauseToAbstractedVarMapState[newClauseId] = candidateDdAbstractedVariables.at(index);
    rebuiltState.clauseToOrAbstractedVarMapState[newClauseId] = candidateDdOrAbstractedVariables.at(index);
    rebuiltState.clauseDDToFormulaClauseMapState[newClauseId] = selectedDdClauseSet;

    for (Int var : filteredDdVarSet.at(i)) {
      if (!(candidateDdAbstractedVariables.at(index).find(var) != candidateDdAbstractedVariables.at(index).end()) && 
        !(candidateDdOrAbstractedVariables.at(index).find(var) != candidateDdOrAbstractedVariables.at(index).end())
      ) {
        if (rebuiltState.varToClauseMapState.find(var) == rebuiltState.varToClauseMapState.end()) {
          rebuiltState.varToClauseMapState[var] = Set<Int>(); 
        }
        rebuiltState.varToClauseMapState.at(var).insert(newClauseId);
      }
    }
  }
  // insert those (mappings) that are not in covered clauses (not resumed)
  for (Int clauseId : activeClauseSet) {
    if (coveredClauses.find(clauseId) == coveredClauses.end()) {
      rebuiltState.clauseToDdMapState[clauseId] = clauseDDMap.at(clauseId);
      rebuiltState.clauseToVarMapState[clauseId] = clauseToVarMap.at(clauseId);
      rebuiltState.clauseToAbstractedVarMapState[clauseId] = Set<Int>();
      rebuiltState.clauseToOrAbstractedVarMapState[clauseId] = Set<Int>();
      rebuiltState.clauseDDToFormulaClauseMapState[clauseId] = Set<Int>({clauseId});
      for (Int var : clauseToVarMap.at(clauseId)) {
        if (rebuiltState.varToClauseMapState.find(var) == rebuiltState.varToClauseMapState.end()) { rebuiltState.varToClauseMapState[var] = Set<Int>(); }
        rebuiltState.varToClauseMapState.at(var).insert(clauseId);
      }
    }
  }
  Float timeElapsedMilli = util::getMilliseconds(timer);
  std::cout << "Cache retrieval time spent: " << timeElapsedMilli / 1000.0 << " s" << std::endl;
  return rebuiltState;
}
Set<Int> PBInteractiveCounter::getActiveVariables(const PBformula &pb) {
  Set<Int> activeVariables;
  for (PBclause const &clause : pb.getClauses()) {
    if (activeClauses.at(clause.clauseId - 1)) {
      for (Int lit : clause.lits) {
        activeVariables.insert(abs(lit));
      }
    }
  }
  return activeVariables;
}
void PBInteractiveCounter::orAbstract(ADD &dd, Int ddVar) {
  ADD varDd = mgr.addVar(ddVar);
  dd = dd.OrAbstract(varDd);
}
void PBInteractiveCounter::orAbstractCube(ADD &dd, Set<Int> &ddVars) {
  for (Int ddVar : ddVars) {
    orAbstract(dd, ddVar);
  }
}
void PBInteractiveCounter::eagerAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights) {
  ;
}
void PBInteractiveCounter::eagerOrAbstraction(Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet){
  ;
}
Map<Int, ADD> PBInteractiveCounter::compileClauses(const PBformula &pb) {
  Map<Int, ADD> activeClauseDDMap;
  for (const PBclause &clause : pb.getClauses()) {
    if (activeClauses.at(clause.clauseId - 1)) {
      // if is active clause
      if (isFound(clause.clauseId, singleClauseDdMap)) {
        activeClauseDDMap[clause.clauseId] = mgr.addOne() * singleClauseDdMap.at(clause.clauseId);
      } else {
        ADD clauseDD = getClauseDd(clause);
        activeClauseDDMap[clause.clauseId] = mgr.addOne() * clauseDD;
        singleClauseDdMap[clause.clauseId] = mgr.addOne() * clauseDD; // save cache
      }
    }
  }
  return activeClauseDDMap;
}
Int PBInteractiveCounter::getLeastCommonVar(Map<Int, Set<Int>> &varToClauseMap) const {
  Int lcv = -1;
  Int minNumClauses = DUMMY_MAX_INT;
  for (auto element : varToClauseMap) {
    if (minNumClauses > element.second.size()) {
      minNumClauses = element.second.size();
      lcv = element.first;
    }
    if (minNumClauses == element.second.size()) {
      if (element.first < lcv) {
        lcv = element.first;
      }
    }
  }
  return lcv;
}
Int PBInteractiveCounter::getLeastCommonNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const {
  if (nonSupportVarSet.size() == 0) {
    throw MyError("Cannot call getLeastCommonNonSupportVar with empty nonSupportVarSet", true);
  }
  Int lcv = -1;
  Int minNumClauses = DUMMY_MAX_INT;
  for (auto element : varToClauseMap) {
    if (isFound(element.first, nonSupportVarSet)){
      if (minNumClauses > element.second.size()) {
        minNumClauses = element.second.size();
        lcv = element.first;
      }
      if (minNumClauses == element.second.size()) {
        if (element.first < lcv) {
          lcv = element.first;
        }
      }
    }
  }
  return lcv;
}

Int PBInteractiveCounter::getNextAbstractionVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const {
  if (nonSupportVarSet.size() == 0) {
    return getNextMergeSupportVar(varToClauseMap);
  } else {
    return getNextMergeNonSupportVar(varToClauseMap, nonSupportVarSet);
  }
}

Int PBInteractiveCounter::getNextMergeSupportVar(Map<Int, Set<Int>> &varToClauseMap) const {
  if (!preferredVariableOrdering.empty()) {
    Int cVar;
    Int cOrderIndex = DUMMY_MAX_INT;
    Int currentBestVar = -1;
    Int currentBestVarOrderIndex = DUMMY_MAX_INT;
    for (auto& element : varToClauseMap) {
      cVar = element.first;
      if(currentBestVar == -1) { 
        currentBestVar = cVar;
      }
      cOrderIndex = preferredVariableOrdering.at(cVar);
      if (cOrderIndex < currentBestVarOrderIndex) {
        currentBestVar = cVar;
      }
    }
    return currentBestVar;
  }
  return getLeastCommonVar(varToClauseMap);
}

Int PBInteractiveCounter::getNextMergeNonSupportVar(Map<Int, Set<Int>> &varToClauseMap, Set<Int> &nonSupportVarSet) const {
  if (nonSupportVarSet.size() == 0) {
    throw MyError("Cannot call getNextMergeNonSupportVar with empty nonSupportVarSet", true);
  }
  if (!preferredVariableOrdering.empty()) {
    Int cVar;
    Int cOrderIndex = DUMMY_MAX_INT;
    Int currentBestVar = -1;
    Int currentBestVarOrderIndex = DUMMY_MAX_INT;
    for (auto& element : varToClauseMap) {
      cVar = element.first;
      if(isFound(cVar, nonSupportVarSet)){
        if(currentBestVar == -1) { 
          currentBestVar = cVar;
        }
        cOrderIndex = preferredVariableOrdering.at(cVar);
        if (cOrderIndex < currentBestVarOrderIndex) {
          currentBestVar = cVar;
        }
      }
    }
    return currentBestVar;
  }
  return getLeastCommonNonSupportVar(varToClauseMap, nonSupportVarSet);
}

// for setting custom merge order in compute graph counter
void PBInteractiveCounter::setPreferredVariableOrdering(VarOrderingHeuristic pbVarOrderingHeuristic, bool inversePbVarOrdering, const PBformula &pb) {
  switch (pbVarOrderingHeuristic) {
    case VarOrderingHeuristic::MINFILL: {
      if (!preferredVariableOrdering.empty() && preferredVariableOrdering.size() >= pb.getApparentVars().size()) {
        break;
      }
      preferredVariableOrdering.clear();
      vector<Int> varOrderVector = pb.getVarOrdering(pbVarOrderingHeuristic, inversePbVarOrdering);
      for (int i = 0; i < varOrderVector.size(); i++) {
        preferredVariableOrdering[varOrderVector.at(i)] = i;
      }
      break;
    }
    default: {
      // showError("Unaccepted variable merge ordering for compute graph counter.");
      break;
    }
  }
}
Set<Int> PBInteractiveCounter::mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &clauseToAbstractedVarMap, Map<Int, Set<Int>> &clauseDDToFormulaClauseMap, const Map<Int, Float> &literalWeights) {
  TimePoint timer = util::getTimePoint();
  ADD dd = mgr.addOne();
  Set<Int> clauseSet = varToClauseMap.at(targetPBVar);
  for (Int clauseID : clauseSet) {
    dd &= clauseDDMap[clauseID];
  }
  // find common variables apart from target pb var (that also does not exist elsewhere)
  Set<Int> commonExVarSet = getClauseExclusiveVarIntersection(clauseSet, clauseToVarMap, varToClauseMap); //common vars should include target pb var
  // project out all common such common variables
  Set<Int> ddVarSet;
  for (Int pbVar : commonExVarSet) {
    ddVarSet.insert(pbVarToDdVarMap.at(pbVar));
  }
  abstractCube(dd, ddVarSet, literalWeights);
  Float timeElapsedMilli = util::getMilliseconds(timer);

  // add dd with variable set to mapping
  // delete the projected out vars from the map and update remaining vars
  for (Int pbVar : commonExVarSet) {
    varToClauseMap.erase(pbVar);
  }
  Set<Int> newAbstractedVarSet;
  Set<Int> newFormulaClauseSet;
  // erase the entire clause and reuse an old clause id for the merged dd
  Int newDDClauseID = *clauseSet.begin();
  Set<Int> newVarSet;
  for (Int clauseID : clauseSet) {
    for (Int pbVar : clauseToVarMap.at(clauseID)) {
      if (!isFound(pbVar, commonExVarSet)) {
        newVarSet.insert(pbVar);
      }
    }
    clauseToVarMap.erase(clauseID);
    clauseDDMap.erase(clauseID);
    newAbstractedVarSet = util::setUnion(newAbstractedVarSet, clauseToAbstractedVarMap.at(clauseID));
    newFormulaClauseSet = util::setUnion(newFormulaClauseSet, clauseDDToFormulaClauseMap.at(clauseID));
    clauseToAbstractedVarMap.erase(clauseID);
    clauseDDToFormulaClauseMap.erase(clauseID);
  }
  clauseDDMap[newDDClauseID] = dd;
  clauseToVarMap[newDDClauseID] = newVarSet;
  clauseToAbstractedVarMap[newDDClauseID] = newAbstractedVarSet;
  clauseDDToFormulaClauseMap[newDDClauseID] = newFormulaClauseSet;
  // update the var clause map of the non exclusive variables
  for (Int pbVar : newVarSet) {
    for (Int clauseID : clauseSet) {
      if ((clauseID != newDDClauseID) && isFound(clauseID, varToClauseMap.at(pbVar))) {
        varToClauseMap.at(pbVar).erase(clauseID);
      }
    }
    varToClauseMap.at(pbVar).insert(newDDClauseID);
  }

  // if more than CACHE_TIME_THRESHOLD seconds, store dd into cache
  if (timeElapsedMilli > CACHE_TIME_THRESHOLD) {
    string ddIdentifier = getDdIdentifierString(clauseDDToFormulaClauseMap.at(newDDClauseID), clauseToAbstractedVarMap.at(newDDClauseID));
    ddCache[ddIdentifier] = dd;
  }
  return commonExVarSet;
}
Set<Int> PBInteractiveCounter::mergeUpdateClauses(Int targetPBVar, Map<Int, Set<Int>> &varToClauseMap, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, ADD> &clauseDDMap, Map<Int, Set<Int>> &clauseToAbstractedVarMap, Map<Int, Set<Int>> &clauseToOrAbstractedVarMap , Map<Int, Set<Int>> &clauseDDToFormulaClauseMap, const Map<Int, Float> &literalWeights, Set<Int> &nonSupportVarSet) {
  bool processedNonSupport = (nonSupportVarSet.size() == 0);
  TimePoint timer = util::getTimePoint();
  ADD dd = mgr.addOne();
  Set<Int> clauseSet = varToClauseMap.at(targetPBVar);
  for (Int clauseID : clauseSet) {
    dd &= clauseDDMap[clauseID];
  }
  // find common variables apart from target pb var (that also does not exist elsewhere)
  Set<Int> commonExVarSet = getClauseExclusiveVarIntersection(clauseSet, clauseToVarMap, varToClauseMap); //common vars should include target pb var
  // project out all common such common variables
  if (!processedNonSupport) {
    Set<Int> nonSupportcommonExVarSet;
    for (Int pbVar : commonExVarSet) {
      if (isFound(pbVar, nonSupportVarSet)) {
        nonSupportcommonExVarSet.insert(pbVar);
      }
    }
    commonExVarSet = nonSupportcommonExVarSet;
  }

  Set<Int> ddVarSet;
  for (Int pbVar : commonExVarSet) {
    if (!processedNonSupport) {
      if (isFound(pbVar, nonSupportVarSet)) {
        ddVarSet.insert(pbVarToDdVarMap.at(pbVar));
      }
    } else {
      ddVarSet.insert(pbVarToDdVarMap.at(pbVar));
    }
  }
  if (!processedNonSupport) {
    orAbstractCube(dd, ddVarSet);
  } else {
    abstractCube(dd, ddVarSet, literalWeights);
  }
  Float timeElapsedMilli = util::getMilliseconds(timer);
  // add dd with variable set to mapping
  // delete the projected out vars from the map and update remaining vars
  for (Int pbVar : commonExVarSet) {
    varToClauseMap.erase(pbVar);
    if (isFound(pbVar, nonSupportVarSet)) {
      nonSupportVarSet.erase(pbVar);
    }
  }
  Set<Int> newAbstractedVarSet;
  Set<Int> newOrAbstractedVarSet;
  Set<Int> newFormulaClauseSet;
  // erase the entire clause and reuse an old clause id for the merged dd
  Int newDDClauseID = *clauseSet.begin();
  Set<Int> newVarSet;
  for (Int clauseID : clauseSet) {
    for (Int pbVar : clauseToVarMap.at(clauseID)) {
      if (!isFound(pbVar, commonExVarSet)) {
        newVarSet.insert(pbVar);
      }
    }
    clauseToVarMap.erase(clauseID);
    clauseDDMap.erase(clauseID);
    newAbstractedVarSet = util::setUnion(newAbstractedVarSet, clauseToAbstractedVarMap.at(clauseID));
    newOrAbstractedVarSet = util::setUnion(newOrAbstractedVarSet, clauseToOrAbstractedVarMap.at(clauseID));
    newFormulaClauseSet = util::setUnion(newFormulaClauseSet, clauseDDToFormulaClauseMap.at(clauseID));
    clauseToAbstractedVarMap.erase(clauseID);
    clauseToOrAbstractedVarMap.erase(clauseID);
    clauseDDToFormulaClauseMap.erase(clauseID);
  }
  // update for abstracted variables during this round
  for (Int clauseId : commonExVarSet) {
    if (processedNonSupport) {
      newAbstractedVarSet.insert(clauseId);
    } else {
      newOrAbstractedVarSet.insert(clauseId);
    }
  }
  clauseDDMap[newDDClauseID] = dd;
  clauseToVarMap[newDDClauseID] = newVarSet;
  clauseToAbstractedVarMap[newDDClauseID] = newAbstractedVarSet;
  clauseToOrAbstractedVarMap[newDDClauseID] = newOrAbstractedVarSet;
  clauseDDToFormulaClauseMap[newDDClauseID] = newFormulaClauseSet;
  // update the var clause map of the non exclusive variables
  for (Int pbVar : newVarSet) {
    for (Int clauseID : clauseSet) {
      if ((clauseID != newDDClauseID) && isFound(clauseID, varToClauseMap.at(pbVar))) {
        varToClauseMap.at(pbVar).erase(clauseID);
      }
    }
    varToClauseMap.at(pbVar).insert(newDDClauseID);
  }
  // if more than CACHE_TIME_THRESHOLD seconds, store dd into cache
  if (timeElapsedMilli >= CACHE_TIME_THRESHOLD) {
    string ddIdentifier = getDdIdentifierString(clauseDDToFormulaClauseMap.at(newDDClauseID), clauseToAbstractedVarMap.at(newDDClauseID), clauseToOrAbstractedVarMap.at(newDDClauseID));
    ddCache[ddIdentifier] = dd;
  }
  return commonExVarSet;
}
Set<Int> PBInteractiveCounter::getClauseExclusiveVarIntersection(Set<Int> &clauseSet, Map<Int, Set<Int>> &clauseToVarMap, Map<Int, Set<Int>> &varToClauseMap) {
  Set<Int> commonVarSet = clauseToVarMap.at(*clauseSet.begin());
  for (Int clauseID : clauseSet) {
    commonVarSet = util::setIntersection(commonVarSet, clauseToVarMap.at(clauseID));
  }
  vector<Int> nonExclusiveVariables;
  for (Int var : commonVarSet) {
    if (clauseSet.size() != varToClauseMap.at(var).size()) {
      nonExclusiveVariables.push_back(var);
      continue;
    }
    bool match = true;
    for (Int candidateClauseID : varToClauseMap.at(var)) {
      if (!isFound(candidateClauseID, clauseSet)) {
        match = false;
      }
    }
    if (!match) { nonExclusiveVariables.push_back(var); }
  }
  // remove non exclusive elements
  for (Int var : nonExclusiveVariables) {
    commonVarSet.erase(var);
  }
  return commonVarSet;
}

//public functions

void PBInteractiveCounter::setAdaptiveRestartMode(bool restartOn) {
  restartOnOrderingMisalign = restartOn;
}
void PBInteractiveCounter::setDDMgr(Cudd &newMgr) {
  mgr = newMgr;
}
Cudd &PBInteractiveCounter::getDDMgr() {
  return mgr;
}
void PBInteractiveCounter::setCache(Map<string, ADD> &newCache) {
  ddCache = newCache;
}
void PBInteractiveCounter::clearCache() {
  ddCache.clear();
}
Map<string, ADD> &PBInteractiveCounter::getCache(){
  return ddCache;
}
void PBInteractiveCounter::setActiveClauses(vector<bool> newActiveClauses) {
  activeClauses = newActiveClauses;
}
Float PBInteractiveCounter::computeModelCount(const PBformula &pb) {
  // setting merge variable order if there is a preferredOrdering
  setPreferredVariableOrdering(pbVarOrderingHeuristic, inversePbVarOrdering, pb);
  if (restartOnOrderingMisalign || PRINT_ORDERING_OFFSET) {
    orderDdVarsIncrementalWithRestart(pb);
  } else {
    orderDdVarsIncremental(pb);
  }
  MultiTypeGraph interactionGraph = pb.getInteractionGraph();
  Map<Int, Set<Int>> variableToClauseMap = interactionGraph.getVarToClauseMap();
  Map<Int, Set<Int>> clauseToVariableMap = interactionGraph.getClauseToVarMap();
  Map<Int, ADD> clauseDDMap = compileClauses(pb);
  Map<Int, Set<Int>> clauseToAbstractedVarMap;
  Map<Int, Set<Int>> clauseToOrAbstractedVarMap;

  Set<Int> projectionSupportVarSet;
  if (pb.isProjected()) {
    projectionSupportVarSet = pb.getProjectionVariableSet();
  } else {
    for (PBclause const &clause : pb.getClauses()) {
      if (activeClauses.at(clause.clauseId - 1)) {
        for (Int lit : clause.lits) {
          projectionSupportVarSet.insert(abs(lit));
        }
      }
    }
  }
  
  
  // resume cache
  ComputeState resumedComputeState = resolveCache(pb, variableToClauseMap, clauseToVariableMap, clauseDDMap);
  variableToClauseMap = resumedComputeState.varToClauseMapState;
  clauseToVariableMap = resumedComputeState.clauseToVarMapState;
  clauseDDMap = resumedComputeState.clauseToDdMapState;
  clauseToAbstractedVarMap = resumedComputeState.clauseToAbstractedVarMapState;
  clauseToOrAbstractedVarMap = resumedComputeState.clauseToOrAbstractedVarMapState;

  Map<Int, Set<Int>> clauseDDToFormulaClauseMap;
  clauseDDToFormulaClauseMap = resumedComputeState.clauseDDToFormulaClauseMapState;

  Set<Int> activeVarSet = getActiveVariables(pb);
  Set<Int> nonSupportVarSet;
  Set<Int> processVarSet;
  for (auto const &element : clauseToAbstractedVarMap) {
    for (Int var : element.second) {
      processVarSet.insert(var);
    }
  }
  for (auto const &element : clauseToOrAbstractedVarMap) {
    for (Int var : element.second) {
      processVarSet.insert(var);
    }
  }
  for (Int var : activeVarSet) {
    if (!isFound(var, processVarSet) && !isFound(var, projectionSupportVarSet)) {
      nonSupportVarSet.insert(var);
    }
  }

  while (variableToClauseMap.size() > 0) {
    Int targetPBVar = getNextAbstractionVar(variableToClauseMap, nonSupportVarSet);
    Set<Int> newlyProcessedVariables = mergeUpdateClauses(targetPBVar, variableToClauseMap, clauseToVariableMap, clauseDDMap, clauseToAbstractedVarMap, clauseToOrAbstractedVarMap, clauseDDToFormulaClauseMap, pb.getLiteralWeights(), nonSupportVarSet);
    processVarSet = util::setUnion(processVarSet, newlyProcessedVariables);
  }
  // handling case where early abstraction handled all variables, left with more than 1 constant ADD.
  ADD pbDd = mgr.addOne();
  if (clauseDDMap.size() > 1) {
    Int newClauseId = clauseDDMap.begin()->first;
    for (auto element : clauseDDMap) {
      pbDd *= element.second;
    }
    clauseDDMap.clear();
    clauseDDMap[newClauseId] = pbDd;
  }
  // getting the model count from the constant decision diagram
  pbDd = (clauseDDMap.begin()->second);
  Float modelCount = diagram::countConstDdFloat(pbDd);
  // adjust model count for simplifications during preprocessing
  modelCount = util::adjustInteractiveProjectedModelCountCG(modelCount, pb.getLiteralWeights(), pb.getInferredAssignments(), projectionSupportVarSet, activeVarSet);
  modelCount = util::adjustInteractiveProjectedModelCountCGMissingVar(modelCount, pb.getLiteralWeights(), pb.getInferredAssignments(), processVarSet, projectionSupportVarSet, activeVarSet);

  return modelCount;
}
Float PBInteractiveCounter::computeProjectedModelCount(const PBformula &pb) {
  return 0;
}
PBInteractiveCounter::PBInteractiveCounter() {
  ddVarOrderingHeuristic = VarOrderingHeuristic::MCS;
  clauseCompilationHeuristic = ClauseCompilationHeuristic::DYNAMIC;
  inverseDdVarOrdering = false;
}
void PBInteractiveCounter::setDdVarOrderingHeuristic(VarOrderingHeuristic ddVarOrderingHeuristic) {
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
}
void PBInteractiveCounter::setInverseDdVarOrdering(bool inverseDdVarOrdering) {
  this->inverseDdVarOrdering = inverseDdVarOrdering;
}
void PBInteractiveCounter::setClauseCompilationHeuristic(ClauseCompilationHeuristic clauseCompilationHeuristic) {
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
}
void PBInteractiveCounter::setPbVarOrderingHeuristic(VarOrderingHeuristic pdVarOrderingHeuristic) {
  this->pbVarOrderingHeuristic = pdVarOrderingHeuristic;
}
void PBInteractiveCounter::setInversePbVarOrdering(bool inversePbVarOrdering) {
  this->inversePbVarOrdering = inversePbVarOrdering;
}