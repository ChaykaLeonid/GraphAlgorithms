#ifndef NAVIGATOR_SRC_INCLUDES_S21_GRAPH_ALGORITHMS_H_
#define NAVIGATOR_SRC_INCLUDES_S21_GRAPH_ALGORITHMS_H_

#include <algorithm>
#include <climits>
#include <cmath>
#include <iostream>
#include <list>
#include <new>
#include <random>
#include <set>
#include <system_error>
#include <utility>
#include <vector>

#include "libs/s21_queue.h"
#include "libs/s21_stack.h"
#include "s21_graph.h"

const float kAlpha = 1.0;
const float kBeta = 1.0;
const float kInitialPheromones = 0.2;
const float kQ = 4;
const float kRo = 0.25;

using s21::Queue;
using s21::Stack;
using std::set;
using std::vector;
using intention_pair = std::list<std::pair<int, double>>;

namespace s21 {
struct TsmResult {
  std::vector<int> vertices;  // массив с искомым маршрутом (с порядком
                              // обхода вершин). Вместо int* можно использовать
                              // std::vector<int>
  double distance = 0;  // длина этого маршрута
  void Clear(bool is_shortest_path);
};

class GraphAlgorithms {
 public:
  ~GraphAlgorithms();

  vector<int> DepthFirstSearch(Graph &graph, int start_vertex);
  vector<int> BreadthFirstSearch(Graph &graph, int start_vertex);
  // Dijkstra's algorithm
  int GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2);
  // Floyd-Warshall algorithm
  vector<vector<int>> GetShortestPathsBetweenAllVertices(Graph &graph);
  // Prim's algorithm
  vector<vector<int>> GetLeastSpanningTree(Graph &graph);
  // Ant algorithm
  TsmResult SolveTravelingSalesmanProblem(Graph &graph);
  void ResetTsm() { to_reset_matrices_ = true; }

 private:
  int nodes_ = 0;
  double **pheromones_matrix_ = nullptr;
  double **added_pheromones_matrix_ = nullptr;
  bool to_reset_matrices_ = true;
  TsmResult ant_, shortest_path_;
  int **distance_matrix_ = nullptr;

  void PrintPheromonesMatrix();
  void UpdatePheromones();
  void AntGoingRound(int starting_node, Graph &graph, TsmResult &shortest_path);
  void AddPheromones();
  void TravellingSalesmanReset(Graph &graph);
  void FreeMatrix(double **matrix, int size);
  double **AllocMatrix(int size);
  void SquareMatrixSetter(double **matrix, double value, char operation);
  intention_pair ReturnProbabilities(Graph &graph,
                                     std::set<int> &nodes_to_visit,
                                     const int from);
  double ReturnProbability(const int &i, const int &j, int **distance_matrix);
  int ChooseNextNode(intention_pair list);
  double RandomReal();
  std::pair<int, int> CheapestEdgeForMST(const int size,
                                         const set<int> &visited, int **matrix);
  void FloydHelper(const int node, vector<vector<int>> &graph);
};
}  // namespace s21

#endif  // NAVIGATOR_SRC_INCLUDES_S21_GRAPH_ALGORITHMS_H_
