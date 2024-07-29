#ifndef NAVIGATOR_SRC_INCLUDES_S21_GRAPH_H_
#define NAVIGATOR_SRC_INCLUDES_S21_GRAPH_H_

#include <cctype>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

using std::string;

namespace s21 {
class Graph {
 public:
  enum status { kOk, kBadPath, kDirectedGraph, kBadFile, kSmallGraph };

  Graph() : adjacency_matrix_(nullptr), vertex_count_(0) {}
  Graph(int n) { Create(n); }
  ~Graph() { Remove(); }

  int CheckValid(int pre);
  int LoadGraphFromFile(string filename);
  int ExportGraphToFile(string filename);
  void ExportGraphToDot(string filename);

  int** GetMatrix() { return adjacency_matrix_; }
  int GetSize() { return vertex_count_; }

  void Print();

 private:
  int** adjacency_matrix_;
  int vertex_count_;

  void DotFileCompleting(std::ofstream& output);
  string ReadFile(string filename);
  void Create(int n);

  int Parse(string filedata, int col, int row);
  void Remove();
};
}  // namespace s21

#endif  // NAVIGATOR_SRC_INCLUDES_S21_GRAPH_H_
