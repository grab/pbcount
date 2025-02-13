/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#pragma once

/* inclusions *****************************************************************/
#include "counter.hpp"
#include "pbformula.hpp"

#include <string>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

/* namespace interactive ******************************************************/

namespace interactive
{
  /**command words for interactive mode*/
  const string Quit = "quit";
  const string Exit = "exit";
  const string AddClause = "add";
  const string DisableClause = "disable";
  const string EnableClause = "enable";
  const string ClearClauses = "restart";
  const string ClearDD = "free";
  const string SelectInputFile = "file";
  const string StartCount = "count";
  const string ShowClauses = "show";
  const string SetProjection = "project";
  const string ClearProjection = "unproject";
  const string Help = "help";
  const string InteractionScript = "script";
}

class Interactor 
{
/**
 * Interactor class to keep track of interaction elements and data structures
 * 
 */
protected:
  Int nextClauseID;
  bool adaptiveRestarts;
  // cache
  Map<string, ADD> ddCache;
  PBformula interactionFormula;
  vector<bool> activeClauses;
  // compute trace -> cached DD mapping
  Cudd mgr; // mgr should persist even when counter is deleted. so that cached DD remains
  PBInteractiveCounter counter;

  /*counter parameters, in case of swapping counter instances*/
  // WeightFormat weightFormat;
  // OutputFormat outputFormat;
  ClusteringHeuristic clusteringHeuristic;
  VarOrderingHeuristic pbVarOrderingHeuristic;
  bool inversePbVarOrdering;
  VarOrderingHeuristic ddVarOrderingHeuristic;
  bool inverseDdVarOrdering;
  ClauseCompilationHeuristic clauseCompilationHeuristic;
  AdaptiveRestartChoice adaptiveRestartChoice;
  /*end of counter parameters*/


  PBclause parseClause(vector<string> &userInputWords, Int newClauseID);
  std::list<vector<string>> parseScriptFile(string scriptFilePath);

public:
  // handling of commands
  void handleAddClause(vector<string> &userInputWords);
  void handleDisableClause(vector<string> &userInputWords);
  void handleEnableClause(vector<string> &userInputWords);
  void handleClearClause();
  void handleClearDD();
  void handleNewInputFile(const string &pbFilePath, WeightFormat weightFormat=WeightFormat::UNWEIGHTED);
  void handleStartCount();
  void handleShowClauses();
  void handleSetProjection(vector<string> &userInputWords);
  void handleClearProjection();
  std::list<vector<string>> handleInteractionScript(vector<string> &userInputWords);

  void printUserAvailableCommands();
  vector<string> getUserInteraction();
  void startInteraction(const string &pbFilePath, WeightFormat weightFormat,
                        OutputFormat outputFormat,
                        ClusteringHeuristic clusteringHeuristic,
                        VarOrderingHeuristic pbVarOrderingHeuristic,
                        bool inversePbVarOrdering,
                        VarOrderingHeuristic ddVarOrderingHeuristic,
                        bool inverseDdVarOrdering, ClauseCompilationHeuristic clauseCompilationHeuristic,
                        AdaptiveRestartChoice adaptiveRestartChoice);
  Interactor();
};