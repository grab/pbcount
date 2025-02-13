/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#include "../interface/interactive.hpp"

/**
 * namespace interative
 */

PBclause Interactor::parseClause(vector<string> &userInputWords, Int newClauseID) {
  // assuming that the first word in vector is the command word for add clause
  // not comment line, so its constraint line
  // each word before >= or = could be variable or coefficient (coefficent
  // must be followed by variable)
  bool isEndTokenSpaced = false;
  if (userInputWords.at(userInputWords.size() - 1) == ";") {
    isEndTokenSpaced = true;
    userInputWords.pop_back();
  }
  int wordCount = userInputWords.size();
  bool prevIsCoeff = false;
  string currentWord;
  PBclause currentClause;
  Int lit = 0;
  for (int i = 1; i < wordCount; i++) {
    currentWord = userInputWords.at(i);
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
      // second last word, should be = or >=
      if (userInputWords.at(i) == "=") {
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
      } else if (currentWord.at(0) == '~') {
        // negated literal
        if (!prevIsCoeff) {
          currentClause.coeffs.push_back(1);
        }
        lit = -1 * std::stoll(currentWord.substr(2, currentWord.size() - 2));
        currentClause.lits.push_back(lit);
        prevIsCoeff = false;
      } else {
        // coefficient
        currentClause.coeffs.push_back(std::stoll(currentWord));
        prevIsCoeff = true;
      }
    }
  }
  // sorting literals by coefficient, negative first
  std::vector<std::pair<Int, Int>> coeffLitPairVect;
  util::zip(currentClause.coeffs, currentClause.lits, coeffLitPairVect);
  std::sort(
      std::begin(coeffLitPairVect), std::end(coeffLitPairVect),
      [&](const auto &a, const auto &b) { return a.first < b.first; });
  util::unzip(coeffLitPairVect, currentClause.coeffs, currentClause.lits);
  currentClause.clauseId = newClauseID;
  return currentClause;
}

std::list<vector<string>> Interactor::parseScriptFile(string scriptFilePath) {
  std::list<vector<string>> scriptLineList;
  std::ifstream inputFileStream(scriptFilePath);
  std::istream *inputStream;
  if (!inputFileStream.is_open()) {
    showError("unable to open file '" + scriptFilePath + "'");
  }
  inputStream = &inputFileStream;
  string line;
  while (std::getline(*inputStream, line)) {
    std::istringstream inputStringStream(line);
    vector<string> words;
    std::copy(std::istream_iterator<string>(inputStringStream), 
              std::istream_iterator<string>(), std::back_inserter(words));
    scriptLineList.push_back(words);
  }
  if (scriptLineList.size() == 0) {
    showError("empty script file parsed");
  }
  return scriptLineList;
}

void Interactor::handleAddClause(vector<string> &userInputWords) {
  PBclause clause = parseClause(userInputWords, nextClauseID);
  nextClauseID += 1;
  interactionFormula.insertClause(clause);
  activeClauses.push_back(true);
}

void Interactor::handleDisableClause(vector<string> &userInputWords) {
  // disables multiple clauses that are space seperated id1 id2 id3
  Int targetClauseId;
  for (Int i = 1; i < userInputWords.size(); i++) {
    targetClauseId = std::stol(userInputWords.at(i));
    if (activeClauses.size() < targetClauseId) {
      std::cout << to_string(targetClauseId) << " is not a valid clause id, please enter a valid clause id." << std::endl;
      return;
    }
    activeClauses[targetClauseId - 1] = false;
  }
}
void Interactor::handleEnableClause(vector<string> &userInputWords) {
  Int targetClauseId;
  for (Int i = 1; i < userInputWords.size(); i++) {
    targetClauseId = std::stol(userInputWords.at(i));
    if (activeClauses.size() < targetClauseId) {
      std::cout << to_string(targetClauseId) << " is not a valid clause id, please enter a valid clause id." << std::endl;
      return;
    }
    activeClauses[targetClauseId - 1] = true;
  }
}
void Interactor::handleClearClause() {
  // resetting interaction state
  nextClauseID = 1;
  activeClauses = vector<bool>(); // clause id starts from 1
  interactionFormula = PBformula();
}
void Interactor::handleClearDD() {
  // clear cached dds
  Map<string, ADD> newDDCache;
  ddCache = newDDCache;
  counter.setCache(ddCache);
}
void Interactor::handleNewInputFile(const string &pbFilePath, WeightFormat weightFormat) {
  bool isWeighted = weightFormat == WeightFormat::WEIGHTED;
  string filePath = pbFilePath;
  interactionFormula = PBformula(filePath, isWeighted);
  nextClauseID = interactionFormula.getClauses().size() + 1;
  activeClauses = vector<bool>(nextClauseID-1, true);
}
void Interactor::handleStartCount() {
  counter.setDDMgr(mgr);
  counter.setCache(ddCache);
  counter.setActiveClauses(activeClauses);
  counter.setAdaptiveRestartMode(adaptiveRestarts);
  counter.setPbVarOrderingHeuristic(pbVarOrderingHeuristic);
  counter.setInversePbVarOrdering(inversePbVarOrdering);
  // clear current cache and mgr reference, might get reset with adaptive restarts
  mgr = Cudd();
  ddCache = Map<string, ADD>();

  // start counter
  TimePoint timer = util::getTimePoint();
  Float count = counter.computeModelCount(interactionFormula);
  Float timeElapsed = util::getSeconds(timer);
  std::cout << "Computed count: " << std::to_string(count) << std::endl;
  std::cout << "Time elapsed: " << std::to_string(timeElapsed) << " s" << std::endl;

  // retrieve the updated cache and mgr
  ddCache = counter.getCache();
  mgr = counter.getDDMgr();
}

void Interactor::handleShowClauses() {
  printComment("Current clauses (A - active, D - disabled):");
  for (PBclause clause : interactionFormula.getClauses()) {
    if (activeClauses.at(clause.clauseId - 1)) {
      std::cout << "A -- ";
    } else {
      std::cout << "D -- ";
    }
    std::cout << "clause id : " << "\t" << clause.clauseId << "\t --> ";
    util::printClause(clause);
  }
  std::cout << "Cache size: " << ddCache.size() << std::endl;
}
void Interactor::handleSetProjection(vector<string> &userInputWords) {
  Set<Int> newProjection;
  for (Int i = 1; i < userInputWords.size(); i++) {
    newProjection.insert(std::stol(userInputWords.at(i)));
  }
  interactionFormula.setProjectionVariableSet(newProjection);
}
void Interactor::handleClearProjection() {
  Set<Int> emptyProjection;
  interactionFormula.setProjectionVariableSet(emptyProjection);
}
std::list<vector<string>> Interactor::handleInteractionScript(vector<string> &userInputWords) {
  std::list<vector<string>> scriptLineList;
  if (userInputWords.size() > 2) {
    showError("Please use indicate script file directories without space");
  } else if (userInputWords.size() < 2) {
    showError("Please specify script file");
  } else {
    printComment("Parsing script file... ");
    string scriptFilePath = userInputWords.at(1);
    scriptLineList = parseScriptFile(scriptFilePath);
  }
  return scriptLineList;
}

void Interactor::printUserAvailableCommands() {
  std::cout << "Available commands: " << std::endl;

  std::cout << interactive::Quit << "/" << interactive::Exit << "\t\t\t\t" << "Exit program" << std::endl;

  std::cout << interactive::AddClause << " <single clause> " << "\t\t\t" << "Add a new PB clause in opb format" << std::endl;

  std::cout << interactive::DisableClause << " <id1 id2 id3 ...> " << "\t\t" << "Disables one or more PB clauses based on ids, space separated" << std::endl;

  std::cout << interactive::EnableClause << " <id1 id2 id3 ...> " << "\t\t" << "Enables one or more PB existing clauses based on ids, space separated" << std::endl;

  std::cout << interactive::ClearClauses << "\t\t\t\t\t" << "Clears all existing clauses" << std::endl;

  std::cout << interactive::ClearDD << "\t\t\t\t\t" << "Clears stored cache, if any" << std::endl;

  std::cout << interactive::SelectInputFile << " <opb file>" << "\t\t\t\t" << "Parse clauses from opb file" << std::endl;

  std::cout << interactive::StartCount << "\t\t\t\t\t" << "Compute count for all enabled PB clauses" << std::endl;

  std::cout << interactive::ShowClauses << "\t\t\t\t\t" << "Show all PB clauses" << std::endl;

  std::cout << interactive::SetProjection << " <x1 x2 x3 ...>" << "\t\t\t" << "Set specified variables as projection set" << std::endl;
  
  std::cout << interactive::ClearProjection << "\t\t\t\t" << "Clears projection set, no projection" << std::endl;

  std::cout << interactive::InteractionScript << " <script file>"<< "\t\t\t" << "Script mode, takes in script file containing series of commands, one per line" << std::endl;

  std::cout << std::endl;
}

vector<string> Interactor::getUserInteraction() {
  printComment("Getting user command input... \t(end input with 'Enter'), 'help' to show available commands.");
  std::istream *inputStream;
  inputStream = &std::cin;
  string line;
  std::getline(*inputStream, line);
  std::istringstream inputStringStream(line);
  vector<string> userInputWords;
  std::copy(std::istream_iterator<string>(inputStringStream),
              std::istream_iterator<string>(), std::back_inserter(userInputWords));
  return userInputWords;
}

void Interactor::startInteraction(const string &pbFilePath, 
                                  WeightFormat weightFormat,
                                  OutputFormat outputFormat,
                                  ClusteringHeuristic clusteringHeuristic,
                                  VarOrderingHeuristic pbVarOrderingHeuristic,
                                  bool inversePbVarOrdering,
                                  VarOrderingHeuristic ddVarOrderingHeuristic,
                                  bool inverseDdVarOrdering, 
                                  ClauseCompilationHeuristic clauseCompilationHeuristic,
                                  AdaptiveRestartChoice adaptiveRestartChoice) {
  this->clusteringHeuristic = clusteringHeuristic;
  this->pbVarOrderingHeuristic = pbVarOrderingHeuristic;
  this->inversePbVarOrdering = inversePbVarOrdering;
  this->ddVarOrderingHeuristic = ddVarOrderingHeuristic;
  this->inverseDdVarOrdering = inverseDdVarOrdering;
  this->clauseCompilationHeuristic = clauseCompilationHeuristic;
  this->adaptiveRestartChoice = adaptiveRestartChoice;

  counter.setDdVarOrderingHeuristic(ddVarOrderingHeuristic);
  counter.setInverseDdVarOrdering(inverseDdVarOrdering);
  counter.setClauseCompilationHeuristic(clauseCompilationHeuristic);

  printComment("Starting interactive mode...");
  bool isWeighted = weightFormat == WeightFormat::WEIGHTED;
  if (adaptiveRestartChoice == AdaptiveRestartChoice::ON) {
    adaptiveRestarts = true;
  }
  string initialPbFile = pbFilePath;
  PBformula pbformula;
  if (pbFilePath != STDIN_CONVENTION) {
    pbformula = PBformula(initialPbFile, isWeighted);
  }
  handleShowClauses();
  vector<string> userInputWords;
  bool scriptTerminate = false;
  std::list<vector<string>> scriptCommandList;
  while(!(scriptTerminate && scriptCommandList.empty())) {
    if (!scriptCommandList.empty()) {
      userInputWords = scriptCommandList.front();
      scriptCommandList.pop_front();
    } else {
      userInputWords = getUserInteraction();
    }
    std::cout << "Parsed command: ";
    for (string word : userInputWords) {
      std::cout << word << ' ';
    }
    std::cout << std::endl;
    if (userInputWords.size() < 1) {
      printComment("Empty input, please enter input");
      printUserAvailableCommands();
    }
    
    string command = userInputWords.at(0);
    if (command == interactive::Quit || command == interactive::Exit) {
      exit(0);
    } else if (command == interactive::AddClause) {
      handleAddClause(userInputWords);
    } else if (command == interactive::DisableClause) {
      handleDisableClause(userInputWords);
    } else if (command == interactive::EnableClause) {
      handleEnableClause(userInputWords);
    } else if (command == interactive::ClearClauses) {
      // remove all clauses
      handleClearClause();
    } else if (command == interactive::ClearDD) {
      // delete and create a new dd manager
      handleClearDD();
    } else if (command == interactive::SelectInputFile) {
      // remove all clauses and parse new file
      const string filePath = userInputWords.at(1);
      handleNewInputFile(filePath);
    } else if (command == interactive::StartCount) {
      handleStartCount();
    } else if (command == interactive::ShowClauses) {
      handleShowClauses();
    } else if (command == interactive::SetProjection) {
      handleSetProjection(userInputWords);
    } else if (command == interactive::ClearProjection) {
      handleClearProjection();
    } else if (command == interactive::InteractionScript) {
      scriptCommandList = handleInteractionScript(userInputWords);
      scriptTerminate = true;
    } else {
      printComment("Unrecognized command, please enter valid command");
      printUserAvailableCommands();
    }
  }
  exit(0); // exited from interaction loop (due to run script)
}

Interactor::Interactor() {
  nextClauseID = 1;
  adaptiveRestarts = false;
}