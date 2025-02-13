/**
 * Copyright 2025 Grabtaxi Holdings Pte Ltd (GRAB). All rights reserved. 
 * Use of this source code is governed by an MIT-style license that can be found in the LICENSE file. 
 */

/* inclusions *****************************************************************/

#include "../interface/graph.hpp"

/* classes ********************************************************************/

/* class Graph ****************************************************************/

void Graph::printVertices() const {
  cout << "vertices: ";
  for (Int vertex : vertices) {
    cout << vertex << " ";
  }
  cout << "\n\n";
}

void Graph::printAdjacencyMap() const {
  cout << "adjacency map {\n";
  for (auto pair = adjacencyMap.begin(); pair != adjacencyMap.end(); pair++) {
    auto vertex = pair->first;
    auto neighbors = pair->second;
    cout << "\t" << vertex << " : ";
    for (auto neighbor = neighbors.begin(); neighbor != neighbors.end(); neighbor++) {
      cout << *neighbor << " ";
    }
    cout << "\n";
  }
  cout << "}\n\n";
}

Graph::Graph(const Set<Int> &cnfVars) {
  vertices = cnfVars;
  for (Int cnfVar : cnfVars) {
    adjacencyMap[cnfVar] = Set<Int>();
  }
}

void Graph::addEdge(Int cnfVar1, Int cnfVar2) {
  adjacencyMap.at(cnfVar1).insert(cnfVar2);
  adjacencyMap.at(cnfVar2).insert(cnfVar1);
}

Set<Int>::const_iterator Graph::beginVertices() const {
  return vertices.begin();
}

Set<Int>::const_iterator Graph::endVertices() const {
  return vertices.end();
}

Set<Int>::const_iterator Graph::beginNeighbors(Int cnfVar) {
  return adjacencyMap.at(cnfVar).begin();
}

Set<Int>::const_iterator Graph::endNeighbors(Int cnfVar) {
  return adjacencyMap.at(cnfVar).end();
}

void Graph::removeVertex(Int v) {
  vertices.erase(v);

  adjacencyMap.erase(v); // edges from v

  for (std::pair<const Int, Set<Int>> &vertexAndNeighbors: adjacencyMap)
    vertexAndNeighbors.second.erase(v); // edges to v
}

bool Graph::hasPath(Int from, Int to, Set<Int> &visitedVertices) const {
  if (from == to) return true;

  visitedVertices.insert(from);

  Set<Int> unvisitedNeighbors;
  util::differ(unvisitedNeighbors, adjacencyMap.at(from), visitedVertices);

  for (Int v : unvisitedNeighbors) if (hasPath(v, to, visitedVertices)) return true;

  return false;
}

bool Graph::hasPath(Int from, Int to) const {
  Set<Int> visitedVertices;
  return hasPath(from, to, visitedVertices);
}


/* class MultiTypeGraph ****************************************************************/

/* protected ******/
string MultiTypeGraph::getEdgeString(Int v1, Int v2) const {
  std::stringstream ss;
  if (v1 > v2) {
    ss << std::to_string(v2) << ":" << std::to_string(v1);
  } else {
    ss << std::to_string(v1) << ":" << std::to_string(v2);
  }
  return ss.str();
}

Int MultiTypeGraph::getNextAvailID() const {
  return (Int) vertices.size() + 1;
}

bool MultiTypeGraph::findInEdgeString(string edgeString, Int v) {
  string delimiter = ":";
  int position = edgeString.find(delimiter);
  string v1String = edgeString.substr(0, position);
  string v2String = edgeString.substr(position + 1, edgeString.size());
  return (std::to_string(v) == v1String) || (std::to_string(v) == v2String);
}

/* public ******/
void MultiTypeGraph::printVertices() const {
  cout << "<vertexID-type-value>: ";
  for (Int v : vertices) {
    cout << v << "-" << vertexTypeMap.at(v) << "-" << valueMap.at(v) << " ";
  }
  cout << "\n\n";
}

void MultiTypeGraph::printAdjacencyMap() const {
  cout << "adjacency map {\n";
  for (auto pair = adjacencyMap.begin(); pair != adjacencyMap.end(); pair++) {
    auto vertex = pair->first;
    auto neighbors = pair->second;
    cout << "\t" << vertex << " : ";
    for (auto neighbor = neighbors.begin(); neighbor != neighbors.end(); neighbor++) {
      cout << *neighbor << " ";
    }
    cout << "\n";
  }
  cout << "}\n\n";
}

void MultiTypeGraph::printEdgeWeightMap() const {
  cout << "Edge weight map {\n";
  for (auto pair = adjacencyMap.begin(); pair != adjacencyMap.end(); pair++) {
    auto vertex = pair->first;
    auto neighbors = pair->second;
    cout << "\t" << vertex << " : ";
    for (auto neighbor : neighbors) {
      cout << neighbor << "-" << edgeWeightMap.at(getEdgeString(neighbor, vertex)) << " ";
    }
    cout << "\n";
  }
  cout << "}\n\n";
}

MultiTypeGraph::MultiTypeGraph(const Set<Int> &vertexSet) {
  vertices = vertexSet;
  for (Int v : vertexSet) {
    adjacencyMap[v] = Set<Int>();
    vertexTypeMap[v] = "U";
    valueMap[v] = v;
  }
}

MultiTypeGraph::MultiTypeGraph(const Set<Int> &vertexSet, string type) {
  vertices = vertexSet;
  for (Int v : vertexSet) {
    adjacencyMap[v] = Set<Int>();
    vertexTypeMap[v] = type;
    valueMap[v] = v;
  }
}

void MultiTypeGraph::addEdge(Int v1, Int v2, Int edgeWeight) {
  adjacencyMap.at(v1).insert(v2);
  adjacencyMap.at(v2).insert(v1);
  edgeWeightMap[getEdgeString(v1, v2)] = edgeWeight;
}

void MultiTypeGraph::addVertex(Int v, Int value, string type) {
  if (vertices.find(v) != vertices.end()) {
    cout << "Vertex " << std::to_string(v) << " already exists, so overwriting" << std::endl;
  } else {
    vertices.insert(v);
  }
  adjacencyMap[v] = Set<Int>();
  vertexTypeMap[v] = type;
  valueMap[v] = value;
}

Int MultiTypeGraph::addClauseVertex(Int clauseId) {
  Int availID = getNextAvailID();
  addVertex(availID, clauseId, "C");
  return availID;
}

Int MultiTypeGraph::addVarVertex(Int var) {
  Int availID = getNextAvailID();
  addVertex(availID, var, "V");
  return availID;
}

Set<Int>::const_iterator MultiTypeGraph::beginVertices() const {
  return vertices.begin();
}

Set<Int>::const_iterator MultiTypeGraph::endVertices() const {
  return vertices.end();
}

Set<Int>::const_iterator MultiTypeGraph::beginNeighbors(Int v) {
  return adjacencyMap.at(v).begin();
}

Set<Int>::const_iterator MultiTypeGraph::endNeighbors(Int v) {
  return adjacencyMap.at(v).end();
}

void MultiTypeGraph::removeVertex(Int v) {
  vertices.erase(v);

  adjacencyMap.erase(v); // edges from v

  for (std::pair<const Int, Set<Int>> &vertexAndNeighbors: adjacencyMap)
    vertexAndNeighbors.second.erase(v); // edges to v
  
  valueMap.erase(v);
  vertexTypeMap.erase(v);

  // flagging edge weights to remove
  vector<string> tempKeyVec;
  for (std::pair<const string, Int> &keyValue : edgeWeightMap) {
    if (findInEdgeString(keyValue.first, v)) {
      tempKeyVec.push_back(keyValue.first);
    }
  }
  // removing edge weights containing v
  for (string &deleteKey : tempKeyVec) {
    edgeWeightMap.erase(deleteKey);
  }
}

void MultiTypeGraph::removeEdge(Int v1, Int v2) {
  adjacencyMap.at(v1).erase(v2);
  adjacencyMap.at(v2).erase(v1);
  edgeWeightMap.erase(getEdgeString(v1, v2));
}

bool MultiTypeGraph::hasPath(Int from, Int to, Set<Int> &visitedVertices) const {
  if (from == to) return true;

  visitedVertices.insert(from);

  Set<Int> unvisitedNeighbors;
  util::differ(unvisitedNeighbors, adjacencyMap.at(from), visitedVertices);

  for (Int v : unvisitedNeighbors) if (hasPath(v, to, visitedVertices)) return true;

  return false;
}

bool MultiTypeGraph::hasPath(Int from, Int to) const {
  Set<Int> visitedVertices;
  return hasPath(from, to, visitedVertices);
}

Map<Int, Set<Int>> MultiTypeGraph::getVarToClauseMap() {
  Map<Int, Set<Int>> varToClauseMap;
  for (Int vertex : vertices) {
    if (vertexTypeMap[vertex] == "V") {
      for (auto it = beginNeighbors(vertex); it != endNeighbors(vertex); it++) {
        Int neighorVertex = *it;
        if (vertexTypeMap[neighorVertex] == "C") {
          varToClauseMap[valueMap.at(vertex)].insert(valueMap.at(neighorVertex));
        }
      }
    }
  }
  return varToClauseMap;
}
Map<Int, Set<Int>> MultiTypeGraph::getClauseToVarMap() {
  Map<Int, Set<Int>> clauseToVarMap;
  for (Int vertex : vertices) {
    if (vertexTypeMap[vertex] == "C") {
      for (auto it = beginNeighbors(vertex); it != endNeighbors(vertex); it++) {
        Int neighorVertex = *it;
        if (vertexTypeMap[neighorVertex] == "V") {
          clauseToVarMap[valueMap.at(vertex)].insert(valueMap.at(neighorVertex));
        }
      }
    }
  }
  return clauseToVarMap;
}