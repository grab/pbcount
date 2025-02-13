/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

#pragma once

/* inclusions *****************************************************************/

#include "util.hpp"

/* uses ***********************************************************************/

using util::printComment;
using util::printThickLine;
using util::printThinLine;
using util::showError;
using util::showWarning;

using std::string;

/* classes ********************************************************************/

class Graph { // undirected
// protected:
//   Set<Int> vertices;
//   Map<Int, Set<Int>> adjacencyMap;

public:
  // miving into public for minfill easier to iterate
  Set<Int> vertices;
  Map<Int, Set<Int>> adjacencyMap;
  //
  void printVertices() const;
  void printAdjacencyMap() const;
  Graph(const Set<Int> &cnfrVars);
  void addEdge(Int cnfrVar1, Int cnfrVar2);
  Set<Int>::const_iterator beginVertices() const;
  Set<Int>::const_iterator endVertices() const;
  Set<Int>::const_iterator beginNeighbors(Int cnfrVar);
  Set<Int>::const_iterator endNeighbors(Int cnfrVar);
  void removeVertex(Int v); // also removes v's edges
  bool hasPath(Int from, Int to, Set<Int> &visitedVertices) const;
  bool hasPath(Int from, Int to) const;
};

// graph structure that supports vertex types and edge weights
class MultiTypeGraph {
/*
vertex ID should start at 1 to match var and clause
*/
protected:
  Set<Int> vertices;
  Map<Int, Set<Int>> adjacencyMap;
  Map<Int, string> vertexTypeMap;
  Map<string, Int> edgeWeightMap;
  Map<Int, Int> valueMap;

  string getEdgeString(Int v1, Int v2) const;
  bool findInEdgeString(string edgeString, Int v);
  Int getNextAvailID() const;

public:
  void printVertices() const;
  void printAdjacencyMap() const;
  void printEdgeWeightMap() const;
  MultiTypeGraph() = default;
  MultiTypeGraph(const Set<Int> &vertexSet);
  MultiTypeGraph(const Set<Int> &vertexSet, string type);
  void addEdge(Int v1, Int v2, Int edgeWeight=1);
  void addVertex(Int v, Int value, string type);
  Int addClauseVertex(Int clauseId);
  Int addVarVertex(Int var);
  Set<Int>::const_iterator beginVertices() const;
  Set<Int>::const_iterator endVertices() const;
  Set<Int>::const_iterator beginNeighbors(Int v);
  Set<Int>::const_iterator endNeighbors(Int v);
  void removeVertex(Int v); // also removes v's edges
  void removeEdge(Int v1, Int v2);
  bool hasPath(Int from, Int to, Set<Int> &visitedVertices) const;
  bool hasPath(Int from, Int to) const;

  Map<Int, Set<Int>> getVarToClauseMap();
  Map<Int, Set<Int>> getClauseToVarMap();
};