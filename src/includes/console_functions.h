#ifndef CONSOLE_PROGRAMM_H_
#define CONSOLE_PROGRAMM_H_

#include <iostream>
#include <string>

#include "s21_graph.h"
#include "s21_graph_algorithms.h"

namespace s21 {
class Console {
 public:
  enum Commands {
    kExit,
    kLoadFromFile,
    kBreadTraversal,
    kDepthTraversal,
    kShortestPathTwoVertices,
    kShortestPathAllVertices,
    kMST,
    kSalesmanProblem,
    kPrint,
    kPrintHelp,
  };

  void Menu(int &status);
  void GetValueWithMessage(const string &msg, int &dest);
  void GetStringWithMessage(const string &msg, string &dest);

 private:
  s21::Graph graph_;
  s21::GraphAlgorithms algo_;

  void LoadFromFile();
  void BreadTraversal();
  void DepthTraversal();
  void ShortestPathBetweenTwo();
  void ShortestPathBetweenAll();
  void MST();
  void SolvingSalesmanProblem();
  void PrintGraph();

  template <typename T>
  void Println(T i);
  void PrintInvite();
  void PrintHelp();
  void PrintVectorInt(vector<int> v);
  void PrintMatrix(vector<vector<int>> matrix);
};

}  // namespace s21
#endif  // CONSOLE_PROGRAMM_H_
