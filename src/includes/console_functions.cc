#include "console_functions.h"

#include <climits>

namespace s21 {
void Console::Menu(int &status) {
  switch (status) {
    case s21::Console::kExit:
      break;
    case s21::Console::kLoadFromFile:
      LoadFromFile();
      break;
    case s21::Console::kBreadTraversal:
      BreadTraversal();
      break;
    case s21::Console::kDepthTraversal:
      DepthTraversal();
      break;
    case s21::Console::kShortestPathTwoVertices:
      ShortestPathBetweenTwo();
      break;
    case s21::Console::kShortestPathAllVertices:
      ShortestPathBetweenAll();
      break;
    case s21::Console::kMST:
      MST();
      break;
    case s21::Console::kSalesmanProblem:
      SolvingSalesmanProblem();
      break;
    case s21::Console::kPrint:
      PrintGraph();
      break;
    default:
      PrintHelp();
  }
}

void Console::LoadFromFile() {
  string filename;
  int status = 0;
  PrintInvite();
  GetStringWithMessage("Enter filename:", filename);
  status = graph_.LoadGraphFromFile(filename);
  algo_.ResetTsm();
  PrintInvite();
  switch (status) {
    case 0:
      Println("Graph loaded");
      break;
    case 1:
      Println("Wrong filename");
      break;
    case 2:
      Println("Directed graph loaded");
      break;
    case 3:
      Println("Bad file");
      break;
    default:
      Println("Something's wrong");
  }
}

void Console::BreadTraversal() {
  int vertex = 0;
  GetValueWithMessage("Enter starting vertex", vertex);
  vector<int> answer = algo_.BreadthFirstSearch(graph_, vertex);
  if (answer[0] == -1) {
    Println("No graph loaded yet.");
  } else if (answer[0] == -2) {
    Println("Vertex out of bounds");
  } else {
    for (auto it = answer.begin(), end = answer.end(); it != end; ++it)
      std::cout << *it << "  ";
    std::cout << "\n";
  }
}

void Console::DepthTraversal() {
  int vertex = 0;
  GetValueWithMessage("Enter starting vertex", vertex);
  vector<int> answer = algo_.DepthFirstSearch(graph_, vertex);
  if (answer[0] == -1) {
    Println("No graph loaded yet.");
  } else if (answer[0] == -2) {
    Println("Vertex out of bounds");
  } else {
    for (auto it = answer.begin(), end = answer.end(); it != end; ++it)
      std::cout << *it << "  ";
    std::cout << "\n";
  }
}

void Console::ShortestPathBetweenTwo() {
  int vertex_1 = 0;
  int vertex_2 = 0;
  GetValueWithMessage("Enter starting vertex", vertex_1);
  GetValueWithMessage("Enter final vertex", vertex_2);
  vertex_1 = algo_.GetShortestPathBetweenVertices(graph_, vertex_1, vertex_2);
  if (vertex_1 == -1) {
    Println("No graph loaded yet.");
  } else if (vertex_1 == -2) {
    Println("Vertex out of bounds");
  } else {
    std::cout << vertex_1 << "\n";
  }
}

void Console::ShortestPathBetweenAll() {
  vector<vector<int>> answer;
  answer = algo_.GetShortestPathsBetweenAllVertices(graph_);
  if (answer[0][0] == -1)
    Println("No graph loaded yet.");
  else
    PrintMatrix(answer);
}

void Console::MST() {
  vector<vector<int>> answer = algo_.GetLeastSpanningTree(graph_);
  if (answer[0][0])
    Println("No or directed graph loaded.");
  else
    PrintMatrix(answer);
}

void Console::SolvingSalesmanProblem() {
  TsmResult answer;
  answer = algo_.SolveTravelingSalesmanProblem(graph_);
  if (answer.distance == INT_MAX) {
    Println("Impossible to solve");
  } else if (answer.distance == -1) {
    Println("No graph loaded yet.");
  } else if (answer.distance == -2) {
    Println("Directed graphs are not supported");
  } else {
    std::cout << "Distance: " << answer.distance << "\n"
              << "Path:"
              << "\n";
    for (auto it = answer.vertices.begin(), end = answer.vertices.end();
         it != end; ++it)
      std::cout << *it << "  ";
    std::cout << "\n";
  }
}

void Console::PrintGraph() {
  if (graph_.CheckValid(0) != 1)
    graph_.Print();
  else
    Println("No graph loaded yet.");
}

template <typename T>
void Console::Println(T i) {
  std::cout << i << "\n";
}

void Console::PrintInvite() { std::cout << "SimpleNavigator>$ \n\t"; }

void Console::PrintHelp() {
  std::cout << kLoadFromFile << " - load the graph from a file\n"
            << kBreadTraversal << " - graph traversal in breadth\n"
            << kDepthTraversal << " - graph traversal in depth\n"
            << kShortestPathTwoVertices
            << " - shortest path between two vertices\n"
            << kShortestPathAllVertices
            << " - shortest paths between all pairs of vertices\n"
            << kMST << " - minimal spanning tree in the graph\n"
            << kSalesmanProblem << " - salesman problem\n"
            << kPrint << " - print graph\n"
            << kPrintHelp << " - help\n"
            << kExit << ", or any other key - exit\n";
}

void Console::PrintVectorInt(vector<int> v) {
  for (int i : v) {
    std::cout << i << " ";
  }
  std::cout << std::endl;
}

void Console::PrintMatrix(vector<vector<int>> matrix) {
  unsigned int size = matrix.size();
  if (size != matrix[0].size()) {
    Println("ERROR: Not square matrix");
  } else {
    for (unsigned int i = 0; i < size; ++i) {
      PrintVectorInt(matrix[i]);
    }
  }
}

void Console::GetValueWithMessage(const string &msg, int &dest) {
  Println(msg);
  std::cout << "> ";
  std::cin >> dest;
}

void Console::GetStringWithMessage(const string &msg, string &dest) {
  Println(msg);
  std::cout << "> ";
  std::cin >> dest;
}

}  // namespace s21
