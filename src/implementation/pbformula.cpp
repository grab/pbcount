/* inclusions *****************************************************************/

#include "../interface/pbformula.hpp"

/* constants for PB************************************************************/
const string &PB_COMMENT_WORD = "*";
const string &PB_EOL_WORD = ";";

/* class PBformula
 * ******************************************************************/
// protected

void PBformula::updateApparentVars(Int literal) {
  Int var = std::abs(literal);
  if (std::find(this->apparentVars.begin(), this->apparentVars.end(), var) !=
      this->apparentVars.end()) {
    this->apparentVars.push_back(var);
  }
}
void PBformula::addClause(const PBclause &clause) {
  this->clauses.push_back(clause);
  for (Int literal : clause.lits) {
    this->updateApparentVars(literal);
  }
}
Graph PBformula::getGaifmanGraph() const {
  std::unordered_set<Int> vars;
  for (Int var : this->apparentVars) {
    vars.insert(var);
  }
  Graph graph(vars);

  for (const PBclause &clause : this->clauses) {
    for (int i = 0; i < clause.lits.size(); i++) {
      for (int j = i + 1; j < clause.lits.size(); j++) {
        Int var1 = std::abs(clause.lits.at(i));
        Int var2 = std::abs(clause.lits.at(j));
        graph.addEdge(var1, var2);
      }
    }
  }
  return graph;
}
vector<Int> PBformula::getAppearanceVarOrdering() const {
  return this->apparentVars;
}
vector<Int> PBformula::getDeclarationVarOrdering() const {
  vector<Int> varOrdering = this->apparentVars;
  std::sort(varOrdering.begin(), varOrdering.end());
  return varOrdering;
}

vector<Int> PBformula::getRandomVarOrdering() const {
  vector<Int> varOrdering = this->apparentVars;
  util::shuffleRandomly(varOrdering);
  return varOrdering;
}

vector<Int> PBformula::getLexpVarOrdering() const {
  Map<Int, Label> unnumberedVertices;
  for (Int vertex : this->apparentVars) unnumberedVertices[vertex] = Label();
  vector<Int> numberedVertices;  // whose \alpha numbers are decreasing
  Graph graph = this->getGaifmanGraph();
  for (Int number = this->apparentVars.size(); number > 0; number--) {
    auto vertexIt =
        std::max_element(unnumberedVertices.begin(), unnumberedVertices.end(),
                         util::isLessValued<Int, Label>);
    Int vertex = vertexIt->first;  // ignores label
    numberedVertices.push_back(vertex);
    unnumberedVertices.erase(vertex);
    for (auto neighborIt = graph.beginNeighbors(vertex);
         neighborIt != graph.endNeighbors(vertex); neighborIt++) {
      Int neighbor = *neighborIt;
      auto unnumberedNeighborIt = unnumberedVertices.find(neighbor);
      if (unnumberedNeighborIt != unnumberedVertices.end()) {
        Int unnumberedNeighbor = unnumberedNeighborIt->first;
        unnumberedVertices.at(unnumberedNeighbor).addNumber(number);
      }
    }
  }
  return numberedVertices;
}

vector<Int> PBformula::getLexmVarOrdering() const {
  Map<Int, Label> unnumberedVertices;
  for (Int vertex : this->apparentVars) unnumberedVertices[vertex] = Label();
  vector<Int> numberedVertices;  // whose \alpha numbers are decreasing
  Graph graph = this->getGaifmanGraph();
  for (Int i = this->apparentVars.size(); i > 0; i--) {
    auto vIt =
        std::max_element(unnumberedVertices.begin(), unnumberedVertices.end(),
                         util::isLessValued<Int, Label>);
    Int v = vIt->first;  // ignores label
    numberedVertices.push_back(v);
    unnumberedVertices.erase(v);

    /* updates numberedVertices: */
    Graph subgraph =
        getGaifmanGraph();  // will only contain v, w, and unnumbered vertices
                            // whose labels are less than w's
    for (auto wIt = unnumberedVertices.begin(); wIt != unnumberedVertices.end();
         wIt++) {
      Int w = wIt->first;
      Label &wLabel = wIt->second;

      /* removes numbered vertices except v: */
      for (Int numberedVertex : numberedVertices)
        if (numberedVertex != v) subgraph.removeVertex(numberedVertex);

      /* removes each non-w unnumbered vertex whose label is not less than w's
       */
      for (const std::pair<Int, Label> &kv : unnumberedVertices) {
        Int unnumberedVertex = kv.first;
        const Label &label = kv.second;
        if (unnumberedVertex != w && label >= wLabel)
          subgraph.removeVertex(unnumberedVertex);
      }

      if (subgraph.hasPath(v, w)) wLabel.addNumber(i);
    }
  }
  return numberedVertices;
}

vector<Int> PBformula::getMcsVarOrdering() const {
  Graph graph = this->getGaifmanGraph();

  auto startVertex = graph.beginVertices();
  if (startVertex == graph.endVertices())  // empty graph
    return vector<Int>();

  Map<Int, Int>
      rankedNeighborCounts;  // unranked vertex |-> number of ranked neighbors
  for (auto it = std::next(startVertex); it != graph.endVertices(); it++)
    rankedNeighborCounts[*it] = 0;

  Int bestVertex = *startVertex;
  Int bestRankedNeighborCount = DUMMY_MIN_INT;

  vector<Int> varOrdering;
  do {
    varOrdering.push_back(bestVertex);

    rankedNeighborCounts.erase(bestVertex);

    for (auto n = graph.beginNeighbors(bestVertex);
         n != graph.endNeighbors(bestVertex); n++) {
      auto entry = rankedNeighborCounts.find(*n);
      if (entry != rankedNeighborCounts.end()) entry->second++;
    }

    bestRankedNeighborCount = DUMMY_MIN_INT;
    for (const std::pair<Int, Int> &entry : rankedNeighborCounts)
      if (entry.second > bestRankedNeighborCount) {
        bestRankedNeighborCount = entry.second;
        bestVertex = entry.first;
      }
  } while (bestRankedNeighborCount != DUMMY_MIN_INT);

  return varOrdering;
}

vector<Int> PBformula::getVarOrdering(VarOrderingHeuristic varOrderingHeuristic,
                                      bool inverse) const {
  vector<Int> varOrdering;
  switch (varOrderingHeuristic) {
    case VarOrderingHeuristic::APPEARANCE: {
      varOrdering = this->getAppearanceVarOrdering();
      break;
    }
    case VarOrderingHeuristic::DECLARATION: {
      varOrdering = this->getDeclarationVarOrdering();
      break;
    }
    case VarOrderingHeuristic::RANDOM: {
      varOrdering = this->getRandomVarOrdering();
      break;
    }
    case VarOrderingHeuristic::LEXP: {
      varOrdering = this->getLexpVarOrdering();
      break;
    }
    case VarOrderingHeuristic::LEXM: {
      varOrdering = this->getLexmVarOrdering();
      break;
    }
    case VarOrderingHeuristic::MCS: {
      varOrdering = this->getMcsVarOrdering();
      break;
    }
    default: {
      showError("DUMMY_VAR_ORDERING_HEURISTIC -- Cnf::getVarOrdering");
    }
  }
  if (inverse) {
    util::invert(varOrdering);
  }
  return varOrdering;
}

// public

bool PBformula::isProjected() const { return this->projectedFormula; }
const Set<Int> &PBformula::getProjectionVariableSet() const {
  return this->projectionVariableSet;
}
Int PBformula::getDeclaredVarCount() const { return this->declaredVarCount; }
Int PBformula::getDeclaredClauseCount() const {
  return this->declaredClauseCount;
}

Map<Int, Float> PBformula::getLiteralWeights() const {
  return this->literalWeights;
}
Int PBformula::getEmptyClauseIndex() const {
  // first (nonnegative) index if found else DUMMY_MIN_INT
  // todo: figure out and document why is this function needed
  for (Int clauseIndex = 0; clauseIndex < this->clauses.size(); clauseIndex++) {
    if (clauses.at(clauseIndex).lits.empty()) {
      return clauseIndex;
    }
  }
  return DUMMY_MIN_INT;
}
const vector<PBclause> &PBformula::getClauses() const { return this->clauses; }
const vector<Int> &PBformula::getApparentVars() const {
  return this->apparentVars;
}
Map<Int, bool> PBformula::getInferredAssignments() const {
  return this->inferredAssignment;
}
void PBformula::printLiteralWeights() const {
  std::cout << "Variables that did not appear in formula will be treated as if their literals have weight 1.0" << std::endl;
  util::printLiteralWeights(this->literalWeights);
}
void PBformula::printClauses() const {
  util::printPb(this->clauses);
}

PBformula::PBformula(std::string &filePath, bool isWeighted) {
  printComment("Reading PB formula...", 1);

  std::ifstream inputFileStream(filePath);
  std::istream *inputStream;
  if (filePath == STDIN_CONVENTION) {
    inputStream = &std::cin;

    printThickLine();
    printComment(
        "Getting PB from stdin... (end input with 'Enter' then 'Ctrl d')");
  } else {
    if (!inputFileStream.is_open()) {
      showError("unable to open file '" + filePath + "'");
    }
    inputStream = &inputFileStream;
  }

  Int declaredClauseCount = DUMMY_MIN_INT;
  Int processedClauseCount = 0;
  Int declaredVarCount = DUMMY_MIN_INT;
  Int lineIndex = 0;
  Int minic2dWeightLineIndex = DUMMY_MIN_INT;
  std::unordered_map<Int, Int> varInputOrderMap;
  Int varCounter = 0;

  string line;
  while (std::getline(*inputStream, line)) {
    lineIndex++;
    std::istringstream inputStringStream(line);

    if (verbosityLevel >= 4)
      printComment("Line " + to_string(lineIndex) + "\t" + line);

    vector<string> words;
    std::copy(std::istream_iterator<string>(inputStringStream),
              std::istream_iterator<string>(), std::back_inserter(words));

    Int wordCount = words.size();

    if (wordCount < 1) continue;

    if (wordCount == 1 && words.at(0).at(0) == '*') continue;

    bool isEndTokenSpaced = false;
    const string &endWord = words.at(wordCount - 1);
    if (endWord == ";") {
      isEndTokenSpaced = true;
      // remove ";" at end of words vector
      words.pop_back();
      wordCount -= 1;
    }
    const string &startWord = words.at(0);
    if (startWord == PB_COMMENT_WORD) {
      // parsing declared variable and clause number (first line comment)
      if (lineIndex == 1) {
        if (wordCount == 5) {
          for (int i = 0; i < wordCount; i++) {
            if (words.at(i) == "#variable=") {
              declaredVarCount = std::stoll(words.at(i + 1));
              this->declaredVarCount = declaredVarCount;
            } else if (words.at(i) == "#constraint=") {
              declaredClauseCount = std::stoll(words.at(i + 1));
              this->declaredClauseCount = declaredClauseCount;
            } else {
              continue;
            }
          }
        }
      } else if (wordCount >= 2 && words.at(1) == "ind") {
        // parsing projection variable lines
        if (words.at(wordCount- 1) != "0") {
          std::cout << "Detected * ind line without 0 as line end tokem, please check input file" << std::endl;
        }
        for (Int i=2; i< wordCount - 1; i++) {
          Int var = std::stoll(words.at(i));
          if (var < 0) {
            std::cout << "Detected * ind negative variable index, parsing absolute index, pleas check input file" << std::endl;
            var = std::abs(var);
          }
          this->projectionVariableSet.insert(var);
        }
      } else if (isWeighted && wordCount >= 4 && words.at(1) == "w") {
        // * w lit weight
        Int literal = std::stoll(words.at(2));
        Float literalWeight = std::stod(words.at(3));
        this->literalWeights[literal] = literalWeight;
      } else {
        continue;
      }
    } else {
      // constraint line
      // each word before >= or = could be variable or coefficient (coefficent
      // must be followed by variable)
      bool prevIsCoeff = false;
      string currentWord;
      PBclause currentClause;
      Int lit = 0;
      for (int i = 0; i < wordCount; i++) {
        currentWord = words.at(i);
        if (i == wordCount - 1) {
          // last word
          // removing line end ';' and parsing as Int (int64) only if ";" is not spaced
          if (isEndTokenSpaced) {
            currentClause.clauseConsVal = std::stoll(currentWord);
          } else {
            currentClause.clauseConsVal =
                std::stoll(currentWord.substr(0, currentWord.size() - 1));
          }
        } else if (i == wordCount - 2) {
          // second last word, should be the constraint comparator = or >=
          if (words.at(i) == "=") {
            currentClause.equals = true;
          }
        } else {
          if (currentWord.at(0) == 'x') {
            // literal
            if (!prevIsCoeff) {
              currentClause.coeffs.push_back(1);
            }
            lit = std::stoll(currentWord.substr(1, currentWord.size() - 1));
            currentClause.lits.push_back(lit);
            prevIsCoeff = false;

            if (varInputOrderMap.find(lit) == varInputOrderMap.end()) {
              varInputOrderMap[lit] = varCounter;
              varCounter++;
            }
          } else if (currentWord.at(0) == '~') {
            // negated literal
            if (!prevIsCoeff) {
              currentClause.coeffs.push_back(1);
            }
            lit = -1 * std::stoll(currentWord.substr(2, currentWord.size() - 2));
            currentClause.lits.push_back(lit);
            prevIsCoeff = false;
            if (varInputOrderMap.find(std::abs(lit)) ==
                varInputOrderMap.end()) {
              varInputOrderMap[abs(lit)] = varCounter;
              varCounter++;
            }
          } else {
            // coefficient
            currentClause.coeffs.push_back(std::stoll(currentWord));
            prevIsCoeff = true;
          }
        }
      }
      // sorting literals by coefficient, negative first so that top down
      // compilation can terminate early correctly
      std::vector<std::pair<Int, Int>> coeffLitPairVect;
      util::zip(currentClause.coeffs, currentClause.lits, coeffLitPairVect);
      std::sort(
          std::begin(coeffLitPairVect), std::end(coeffLitPairVect),
          [&](const auto &a, const auto &b) { return a.first < b.first; });
      util::unzip(coeffLitPairVect, currentClause.coeffs, currentClause.lits);
      currentClause.clauseId = processedClauseCount + 1;
      this->clauses.push_back(currentClause);
      processedClauseCount++;
    }
    this->apparentVars = vector<Int>(varInputOrderMap.size());
    for (auto i : varInputOrderMap) {
      this->apparentVars.at(i.second) = i.first;
    }
  }
  if (!isWeighted) {
    for (Int var : this->apparentVars) {
      this->literalWeights[var] = 1.0;
      this->literalWeights[-var] = 1.0;
    }
  } else {
    // normalizing literal weights
    for (Int var : this->apparentVars) {
      if (this->literalWeights.find(var) != this->literalWeights.end() &&
          this->literalWeights.find(-var) != this->literalWeights.end()) {
        continue;
      } else if (this->literalWeights.find(var) != this->literalWeights.end()) {
        if (this->literalWeights[var] < 1.0) {
          this->literalWeights[-var] = (Float) 1.0 - this->literalWeights[var];
        } else {
          this->literalWeights[-var] = 1.0;
        }
      } else if (this->literalWeights.find(-var) != this->literalWeights.end()) {
        if (this->literalWeights[-var] < 1.0) {
          this->literalWeights[var] = (Float) 1.0 - this->literalWeights[-var];
        } else {
          this->literalWeights[-var] = 1.0;
        }
      } else {
        this->literalWeights[var] = 1.0;
        this->literalWeights[-var] = 1.0;
      }
    }
  }
  
  if (projectionVariableSet.size() > 0) {
    this->projectedFormula = true;
  } else {
    this->projectedFormula = false;
  }

  if (verbosityLevel >= 1) {
    util::printRow("declaredVarCount", declaredVarCount);
    util::printRow("apparentVarCount", apparentVars.size());
    util::printRow("declaredClauseCount", declaredClauseCount);
    util::printRow("apparentClauseCount", processedClauseCount);
  }

  if (verbosityLevel >= 3) {
    printClauses();
    printLiteralWeights();
  }

  std::cout << "Completed parsing" << std::endl;
}

void PBformula::inferAssignment(PBclause &clause, Map<Int, bool> &inferredAssignmentMap) {
  /*
    takes in a pb clause, analyses it to determine if assignment can be inferred from it
  */
  float normConsVal = (float) clause.clauseConsVal / (float) clause.coeffs.at(0);
  if (clause.equals) {
    // float normConsVal = (float) currentClause.clauseConsVal / (float)currentClause.coeffs.at(0);
    if (normConsVal > 1.0) { // if normConsVal > -1 any assignment sat
      this->inferredUnsat = true;
    } else {
      if (clause.lits.at(0) > 0 && normConsVal == 1) {
        addCheckInferredAssign(std::abs(clause.lits.at(0)), inferredAssignmentMap);
      } else if (clause.lits.at(0) > 0 && normConsVal == 0) {
        addCheckInferredAssign(-std::abs(clause.lits.at(0)), inferredAssignmentMap);
      } else if (clause.lits.at(0) < 0 && normConsVal == 1) {
        addCheckInferredAssign(-std::abs(clause.lits.at(0)), inferredAssignmentMap);
      } else if (clause.lits.at(0) < 0 && normConsVal == 0) {
        addCheckInferredAssign(std::abs(clause.lits.at(0)), inferredAssignmentMap);
      } else { 
        // when normConsVal is not a whole number eg 3x = 2
        // to check if it captures when lit >= -2 (cannot infer)
        this->inferredUnsat = true;
      }
    }
  } else {
    // unit clause not equals so >=
    if (normConsVal > 1.0 && clause.coeffs.at(0) > 0) {
      // remains as >= after division
      this->inferredUnsat = true;
    } else if (normConsVal < 0.0 && clause.coeffs.at(0) < 0) {
      // <= after division
      this->inferredUnsat = true;
    } else {
      if (clause.coeffs.at(0) > 0) {
        // >=
        if (clause.lits.at(0) > 0 && normConsVal > 0){ 
          addCheckInferredAssign(std::abs(clause.lits.at(0)), inferredAssignmentMap);
        } else if (clause.lits.at(0) < 0 && normConsVal > 0) {
          addCheckInferredAssign(-std::abs(clause.lits.at(0)), inferredAssignmentMap);
        } else {
          // cannot infer anything
          ;
        }
      } else {
        // <= after division
        if (clause.lits.at(0) < 0 && normConsVal < 1.0){ 
          // inferredAssignmentMap[std::abs(clause.lits.at(0))] = true;
          addCheckInferredAssign(std::abs(clause.lits.at(0)), inferredAssignmentMap);
        } else if (clause.lits.at(0) > 0 && normConsVal < 1.0) {
          // inferredAssignmentMap[std::abs(clause.lits.at(0))] = false;
          addCheckInferredAssign(-std::abs(clause.lits.at(0)), inferredAssignmentMap);
        } else {
          // cannot infer anything
          ;
        }
      }
    }
  }
}

void PBformula::preprocess() {
  /*
    processing currently performs:
      - propagation repeatedly until nothing to propergate
    modifies the clause list
  */
  Map<Int, bool> currentInferredAssignment = this->inferredAssignment;
  bool hasNextIter = true;
  bool hasInfer = true;
  bool hasAssign = true;
  bool hasProbe = true;
  bool failedLitPerformed = false;
  while (hasNextIter) {
    hasInfer = false;
    hasAssign = false;
    hasProbe = false;
    vector<PBclause> updatedClauseVector;
    // loop through clauses and then infer
    Set<Int> removalSet;
    for (Int i = 0; i < this->clauses.size(); i++) {
      if (this->clauses.at(i).lits.size() == 1) {
        inferAssignment(this->clauses.at(i), currentInferredAssignment);
        removalSet.insert(i);
      } else if (this->clauses.at(i).lits.size() == 0) {
        if (this->clauses.at(i).equals && this->clauses.at(i).clauseConsVal != 0) {
          this->inferredUnsat = true;
        }
        if (!this->clauses.at(i).equals && this->clauses.at(i).clauseConsVal > 0) {
          this->inferredUnsat = true;
        }
        removalSet.insert(i);
      } else {
        // more than 1 lits
        // check if always sat (only for >= case)
        if (!this->clauses.at(i).equals) {
          bool clauseAlwaysTrue = checkAlwaysTrue(this->clauses.at(i));
          if (clauseAlwaysTrue) {
            removalSet.insert(i);
          }
        }
      }
    }
    if (!removalSet.empty()) {hasInfer = true;}
    // done inferring for current iteration, now apply to each clause
    for (Int i = 0; i < this->clauses.size(); i++) {
      if (removalSet.find(i) != removalSet.end()) {
        continue;
      } else {
        PBclause updatedClause;
        updatedClause.clauseId = this->clauses.at(i).clauseId;
        updatedClause.equals = this->clauses.at(i).equals;
        Int updatedConsVal = this->clauses.at(i).clauseConsVal;
        for (Int j = 0; j < this->clauses.at(i).lits.size(); j++) {
          if ( currentInferredAssignment.find(std::abs(this->clauses.at(i).lits.at(j))) == currentInferredAssignment.end() ) {
            Int updatedCoeff = this->clauses.at(i).coeffs.at(j);
            Int updatedLit = this->clauses.at(i).lits.at(j);
            updatedClause.coeffs.push_back(updatedCoeff);
            updatedClause.lits.push_back(updatedLit);
          } else {
            Int currentVar = std::abs(this->clauses.at(i).lits.at(j));
            Int appliedValue = this->clauses.at(i).coeffs.at(j);
            if (currentInferredAssignment.at(currentVar) && this->clauses.at(i).lits.at(j) < 0) {
              appliedValue = 0;
            }
            if (!currentInferredAssignment.at(currentVar) && this->clauses.at(i).lits.at(j) > 0) {
              appliedValue = 0;
            }
            updatedConsVal -= appliedValue;
            hasAssign = true;
          }
          updatedClause.clauseConsVal = updatedConsVal;
        }
        updatedClauseVector.push_back(updatedClause);
      }
    }
    this->clauses = updatedClauseVector;
    // adding probing infer here since that does not involve immediately modifying clauses, we just add to inferred assignments and propagate next iteration
    Set<Int> probeInferredAssignments = probeInferAssign(currentInferredAssignment, this->clauses);
    if (!probeInferredAssignments.empty()) {
      hasProbe = true;
      for (Int inferredLit : probeInferredAssignments) {
        addCheckInferredAssign(inferredLit, currentInferredAssignment);
      }
    }
    // adding failed literal test here (call sat solver with decision on literal, if unsat means alternate setting)
    if (!failedLitPerformed) {
      Set<Int> failedLitInferredAssignments = failedLiteralTest(currentInferredAssignment, this->clauses);
      if (!failedLitInferredAssignments.empty()) {
        for (Int inferredLit : failedLitInferredAssignments) {
          addCheckInferredAssign(inferredLit, currentInferredAssignment);
        }
      }
      failedLitPerformed = true;
    }

    if (!hasAssign && !hasInfer && !hasProbe) {
      hasNextIter = false;
    }
  }
  this->inferredAssignment = currentInferredAssignment;
}

PBclause PBformula::applyProbe(PBclause const &clause, Int decLit) {
  PBclause probeClause;
  probeClause.equals = clause.equals;
  vector<Int> probeLits;
  vector<Int> probeCoeffs;
  Int probeConsVal = clause.clauseConsVal;
  for (int i = 0; i < clause.lits.size(); i++) {
    if (std::abs(clause.lits.at(i)) != std::abs(decLit)) {
      probeLits.push_back(clause.lits.at(i));
      probeCoeffs.push_back(clause.coeffs.at(i));
    } else {
      if (clause.lits.at(i) == decLit) {
        // same sign so have to account for coefficient
        probeConsVal -= clause.coeffs.at(i);
      }
    }
  }
  probeClause.clauseConsVal = probeConsVal;
  probeClause.lits = probeLits;
  probeClause.coeffs = probeCoeffs;
  return probeClause;
}

std::pair<Int, bool> PBformula::inferProbe(PBclause& clause) {
  // should be unit clause here
  bool isSat = true;
  Int inferredLit = 0;
  
  float normConsVal = (float) clause.clauseConsVal / (float) clause.coeffs.at(0);
  if (clause.equals) {
    if (normConsVal > 1.0) { // if normConsVal > -1 any assignment sat
      isSat = false;
    } else {
      if (clause.lits.at(0) > 0 && normConsVal == 1) {
        inferredLit = std::abs(clause.lits.at(0));
      } else if (clause.lits.at(0) > 0 && normConsVal == 0) {
        inferredLit = -std::abs(clause.lits.at(0));
      } else if (clause.lits.at(0) < 0 && normConsVal == 1) {
        inferredLit = -std::abs(clause.lits.at(0));
      } else if (clause.lits.at(0) < 0 && normConsVal == 0) {
        inferredLit = std::abs(clause.lits.at(0));
      } else { 
        isSat = false;
      }
    }
  } else {
    // unit clause >=
    if (normConsVal > 1.0 && clause.coeffs.at(0) > 0) {
      // remains as >= after division
      isSat = false;
    } else if (normConsVal < 0.0 && clause.coeffs.at(0) < 0) {
      // <= after division
      isSat = false;
    } else {
      if (clause.coeffs.at(0) > 0) {
        // >=
        if (clause.lits.at(0) > 0 && normConsVal > 0){ 
          inferredLit = std::abs(clause.lits.at(0));
        } else if (clause.lits.at(0) < 0 && normConsVal > 0) {
          inferredLit = -std::abs(clause.lits.at(0));
        } else {
          // cannot infer anything
          inferredLit = 0;
        }
      } else {
        // <= after division
        if (clause.lits.at(0) < 0 && normConsVal < 1.0){ 
          inferredLit = std::abs(clause.lits.at(0));
        } else if (clause.lits.at(0) > 0 && normConsVal < 1.0) {
          inferredLit = -std::abs(clause.lits.at(0));
        } else {
          // cannot infer anything
          inferredLit = 0;
        }
      }
    }
  }

  return std::pair<Int, bool>(inferredLit, isSat);
}

Set<Int> PBformula::probeInferAssign(Map<Int, bool> &inferredAssignmentMap, vector<PBclause> &clauseVector) {
  Set<Int> probeInferredAssignment;
  for (Int var = 1; var <= declaredVarCount; var++) {
    // loop through all variable x, probe when variable is assigned true and false.
    if (inferredAssignmentMap.find(var) == inferredAssignmentMap.end()) {
      // only look at variables that are not yet inferred (not applied yet)
      for (PBclause const &clause : clauseVector) {
        // only need to look at clauses with exactly 2 literals (set 1 and infer the other)
        if (clause.lits.size() == 2) {
          // assumption: literal of same variable cannot appear twice
          // check if worth probing
          if (std::abs(clause.lits.at(0)) == var || std::abs(clause.lits.at(1)) == var) {
            // probe pos literal
            PBclause posProbeClause = applyProbe(clause, var);
            std::pair<Int, bool> posProbeResult = inferProbe(posProbeClause);
            Int posProbeLit = posProbeResult.first;
            bool posSat = posProbeResult.second;
            // probe neg literal
            PBclause negProbeClause = applyProbe(clause, -var);
            std::pair<Int, bool> negProbeResult = inferProbe(negProbeClause);
            Int negProbeLit = negProbeResult.first;
            bool negSat = negProbeResult.second;
            // infer from the two probes
            if (!posSat && !negSat) {
              // both assignments of variable is unsat, formula is unsat
              this->inferredUnsat = true;
            } else if (!posSat) {
              if (this->inferredAssignment.find(var) == this->inferredAssignment.end()) {
                this->inferredAssignment[var] = false;
              } else {
                if (this->inferredAssignment[var] == true) {
                  this->inferredUnsat = true;
                }
              }
            } else if (!negSat) {
              if (this->inferredAssignment.find(var) == this->inferredAssignment.end()) {
                this->inferredAssignment[var] = true;
              } else {
                if (this->inferredAssignment[var] == false) {
                  this->inferredUnsat = true;
                }
              }
            } else {
              // both probes sat so can infer
              if (posProbeLit != 0 && negProbeLit != 0) {
                // using 0 to mean undecidable
                if (posProbeLit == negProbeLit) {
                  probeInferredAssignment.insert(posProbeLit);
                }
              }
            }
          }
        }
      }
    }
  }
  return probeInferredAssignment;
}

void PBformula::addCheckInferredAssign(Int inferredLit) {
  if (this->inferredAssignment.find(std::abs(inferredLit)) != this->inferredAssignment.end()) {
    if (this->inferredAssignment[std::abs(inferredLit)] != (inferredLit > 0)) {
      this->inferredUnsat = true;
    }
  } else {
    this->inferredAssignment[std::abs(inferredLit)] = inferredLit > 0;
  }  
}
void PBformula::addCheckInferredAssign(Int inferredLit, Map<Int, bool>&currentInferredAssignment) {
  if (currentInferredAssignment.find(std::abs(inferredLit)) != currentInferredAssignment.end()) {
    if (currentInferredAssignment[std::abs(inferredLit)] != (inferredLit > 0)) {
      this->inferredUnsat = true;
    }
  } else {
    currentInferredAssignment[std::abs(inferredLit)] = inferredLit > 0;
  }  
}

bool PBformula::isUnsat() {
  return this->inferredUnsat;
}

bool PBformula::satTest(vector<PBclause> &clauseVector, vector<Int> testLits) {
  // part of preprocessing function, failed literal test
  bool sat = true;
  // check if solver is available, if not available just return sat
  if (!this->solverDetected) {return sat;}
  std::stringstream ss;
  ss << "* #variable= " << this->declaredVarCount << "#constraint= " << clauseVector.size() << std::endl;
  for (const PBclause& clause : clauseVector) {
    for (int i = 0; i < clause.lits.size(); i++) {
      ss << clause.coeffs.at(i) << " ";
      if (clause.lits.at(i) < 0) {
        ss << "~";
      }
      ss << "x" << std::abs(clause.lits.at(i)) << " ";
    }
    if (clause.equals) {
      ss << "= ";
    } else {
      ss << ">= ";
    }
    ss << clause.clauseConsVal << " ;" << std::endl;
  }
  for (int i = 0; i < testLits.size(); i++) {
    ss << "1 ";
    if (testLits.at(i) < 0) {
      ss << "~";
    }
    ss << "x" << std::abs(testLits.at(i)) << " ";
    ss << ">= 1 ;" << std::endl;
  }

  string formulaString = ss.str();

  string cmd = SAT_SOLVER_CMD;
  bp::opstream in;
  bp::ipstream out;
  vector<string> output;

  bp::child solverProc(cmd, bp::std_out > out, bp::std_err > stderr, bp::std_in < in);

  in << formulaString << std::endl;
  in.pipe().close();
  string line;
  while (solverProc.running() && std::getline(out, line) && !line.empty()) {
    output.push_back(line);
  }
  solverProc.join();

  for (string line : output) {
    std::stringstream ss(line);
    vector<string> words;
    string word;
    while (ss >> word) {
      words.push_back(word);
    }
    if (words.at(0) == "s") {
      if (words.at(1) == "UNSATISFIABLE") {
        sat = false;
      }
    }
  }
  return sat;
}

Set<Int> PBformula::failedLiteralTest(Map<Int, bool> &inferredAssignmentMap, vector<PBclause> &clauseVector) {
  Set<Int> failedLitInferredAssignment;
  for (Int var = 1; var < this->declaredClauseCount + 1; var++) {
    if (inferredAssignmentMap.find(var) == inferredAssignmentMap.end()) {
      vector<Int> testLits(1);
      testLits[1] = -var;
      bool isSatNegLit = satTest(clauseVector, testLits);
      testLits[1] = var;
      bool isSatPosLit = satTest(clauseVector, testLits);
      if (!isSatNegLit & !isSatPosLit) {
        this->inferredUnsat = true;
      } else if (!isSatNegLit) {
        failedLitInferredAssignment.insert(var);
      } else if (!isSatPosLit) {
        failedLitInferredAssignment.insert(-var);
      } else {
        // both sat, cannot infer anything
        ;
      }
    }
  }
  return failedLitInferredAssignment;
}
void PBformula::detectSolver() {
  struct stat buffer;  
  string name = "./roundingsat"; 
  bool solverPresent = (stat (name.c_str(), &buffer) == 0);
  this->solverDetected = solverPresent;
}
bool PBformula::checkAlwaysTrue(PBclause& clause) {
  // use only on >= clauses
  assert(!clause.equals);
  bool alwaysTrue = false;
  Int currentConsVal = clause.clauseConsVal;
  for (int i = 0; i < clause.coeffs.size(); i++) {
    // loop through coefficients, change all to positive
    if (clause.coeffs.at(i) < 0) {
      currentConsVal -= clause.coeffs.at(i);
    }
  }
  if (currentConsVal <= 0) {
    alwaysTrue = true;
  }
  return alwaysTrue;
}
