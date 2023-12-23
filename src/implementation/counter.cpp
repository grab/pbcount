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

  if (verbosityLevel > 0) {
    std::cout << "Compiling clause id: " << clause.clauseId <<std::endl;
    util::printClause(clause);
  }

  ADD clauseDD;
  if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::TOPDOWN) {
    clauseDD = getClauseDdTD(clause);
  } else if (this->clauseCompilationHeuristic == ClauseCompilationHeuristic::DYNAMIC) {
    clauseDD = getClauseDdDynamic(clause);
  } else {
    clauseDD = getClauseDdBU(clause);
  }
  if (verbosityLevel > 0) {
    std::cout << "Completed compiling clause id: " << clause.clauseId <<std::endl;
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
  // checking if coefficients going to be positive for later literals, for early  termination
  if (!remainCoeffPositive && idx < clause.coeffs.size()) {
    if (clause.coeffs.at(idx) >= 0) {
      remainCoeffPositive = true;
    }
  }

  if (clause.equals && currentVal > clause.clauseConsVal &&
      remainCoeffPositive) {
    // exceeded equals, since all positive then can early terminate
    return mgr.addZero();
  } else if (!clause.equals && currentVal >= clause.clauseConsVal &&
             remainCoeffPositive) {
    // geq constraint satisfied
    return mgr.addOne();
  } else if (idx < clause.lits.size()) {
    // yet to terminate (reach all variables)
    Int ddVar = pbVarToDdVarMap.at(std::abs(clause.lits.at(idx)));
    Int litCoeff = clause.coeffs.at(idx);
    // ADD literalDd;
    ADD literalDd = mgr.addVar(ddVar);

    if (clause.lits.at(idx) < 0) { literalDd = ~literalDd; };

    return literalDd.Ite(
          getClauseDdTDHelper(clause, idx + 1, currentVal + litCoeff,
                              remainCoeffPositive),
          getClauseDdTDHelper(clause, idx + 1, currentVal,
                              remainCoeffPositive));
  } else {
    // reached end, check if it is equals constraint and satisfied
    if (clause.equals && currentVal == clause.clauseConsVal) { return mgr.addOne(); }; 
    // reached end, but not satisfied
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
  if (util::isFound(pbVar, projectionVariableSet)) {
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
      if (!util::isFound(ddVarToPbVarMap.at(ddVar), pb.getProjectionVariableSet())) {
        projectAbstract(pbDd, ddVar, pb.getLiteralWeights(), pb.getProjectionVariableSet());
      }
    }
    // model count for variables in projection set
    for (Int ddVar : support) {
      if (util::isFound(ddVarToPbVarMap.at(ddVar), pb.getProjectionVariableSet())) {
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

  Set<Int> placedPbVars; 
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

  Set<Int> placedPbVars; 
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
