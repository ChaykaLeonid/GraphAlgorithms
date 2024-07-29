#include <gtest/gtest.h>

#include <vector>

#include "./includes/s21_graph.h"
#include "./includes/s21_graph_algorithms.h"

using std::vector;

TEST(PartOneTests, DFS1) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/DFS1.txt");
  s21::GraphAlgorithms algo;
  std::vector<int> result = algo.DepthFirstSearch(graph, 1);
  int expected_values[] = {1, 4, 3, 5, 2};

  for (int i = 0; i < 5; i++) EXPECT_EQ(result[i], expected_values[i]);
}

TEST(PartOneTests, DFS2) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/DFS2.txt");
  s21::GraphAlgorithms algo;
  std::vector<int> result = algo.DepthFirstSearch(graph, 3);
  int expected_values2[] = {3, 4, 1, 2};

  for (int i = 0; i < 4; i++) EXPECT_EQ(result[i], expected_values2[i]);
}

TEST(PartOneTests, DFS3) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/TreeGraph.txt");
  s21::GraphAlgorithms algo;
  std::vector<int> result = algo.DepthFirstSearch(graph, 1);
  int expected_values3[] = {1, 4, 8, 7, 12, 11, 3, 2, 6, 5, 10, 9};

  for (int i = 0; i < 12; i++) EXPECT_EQ(result[i], expected_values3[i]);
}

TEST(PartOneTests, DFSWrongVertex) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/DFS1.txt");
  s21::GraphAlgorithms algo;
  std::vector<int> result = algo.DepthFirstSearch(graph, 999);
  EXPECT_EQ(result.at(0), -2);
}

TEST(PartOneTests, DFSEmptyGraph) {
  s21::Graph empty_graph;
  s21::GraphAlgorithms algo;
  std::vector<int> result = algo.DepthFirstSearch(empty_graph, 9);
  EXPECT_EQ(result.at(0), -1);
}

TEST(PartOneTests, BFS) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/TreeGraph.txt");
  s21::GraphAlgorithms algo;
  std::vector<int> result = algo.BreadthFirstSearch(graph, 1);
  int expected_values3[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

  for (int i = 0; i < 12; i++) EXPECT_EQ(result[i], expected_values3[i]);
}

TEST(PartOneTests, exportGraphToDot) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/example.txt");
  graph.ExportGraphToDot("datasets/aboba.dot");
  std::ifstream file("datasets/aboba.dot");
  char c;
  if (!file.is_open() || !(file >> c)) {
    file.close();
    FAIL();
  }
  file.close();
}

TEST(PartTwoTests, GetShortestPathBetweenVertices) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/example.txt");
  s21::GraphAlgorithms algo;
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, 1, 4), 20);
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, 3, 8), 9);
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, 11, 2), 12);
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, 4, 10), 13);
  EXPECT_EQ(algo.GetShortestPathBetweenVertices(graph, 13, 14), -2);
}

TEST(PartTwoTests, GetShortestPathsBetweenAllVertices1) {
  s21::Graph graph, graph2, graph3;
  s21::GraphAlgorithms algo;
  graph.LoadGraphFromFile("datasets/example2.txt");
  vector<vector<int>> matrix = algo.GetShortestPathsBetweenAllVertices(graph);
  vector<vector<int>> tmp_matrix = {
      {0, 1, 1, 1}, {1, 0, 1, 2}, {1, 1, 0, 2}, {1, 2, 2, 0}};

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      EXPECT_EQ(tmp_matrix[i][j], matrix[i][j]);
    }
  }
}

TEST(PartTwoTests, GetShortestPathsBetweenAllVertices2) {
  s21::Graph graph;
  s21::GraphAlgorithms algo;
  graph.LoadGraphFromFile("datasets/example3.txt");
  vector<vector<int>> matrix = algo.GetShortestPathsBetweenAllVertices(graph);

  vector<vector<int>> pattern = {
      {0, 1, 2, 2}, {1, 0, 1, 1}, {2, 1, 0, 2}, {2, 1, 2, 0}};

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      EXPECT_EQ(matrix[i][j], pattern[i][j]);
    }
  }
}

TEST(PartTwoTests, GetShortestPathsBetweenAllVertices3) {
  s21::Graph graph;
  s21::GraphAlgorithms algo;
  graph.LoadGraphFromFile("datasets/example1.txt");
  vector<vector<int>> matrix = algo.GetShortestPathsBetweenAllVertices(graph);
  vector<vector<int>> pattern = {{0, 29, 20, 20, 16, 31, 100, 12, 4, 31, 18},
                                 {29, 0, 15, 29, 28, 40, 72, 21, 29, 41, 12},
                                 {20, 15, 0, 15, 14, 25, 81, 9, 23, 27, 13},
                                 {20, 29, 15, 0, 4, 12, 92, 12, 24, 13, 25},
                                 {16, 28, 14, 4, 0, 16, 94, 9, 20, 16, 22},
                                 {31, 40, 25, 12, 16, 0, 95, 24, 35, 3, 37},
                                 {100, 72, 81, 92, 94, 95, 0, 90, 101, 98, 84},
                                 {12, 21, 9, 12, 9, 24, 90, 0, 15, 25, 13},
                                 {4, 29, 23, 24, 20, 35, 101, 15, 0, 35, 18},
                                 {31, 41, 27, 13, 16, 3, 98, 25, 35, 0, 38},
                                 {18, 12, 13, 25, 22, 37, 84, 13, 18, 38, 0}};

  for (int i = 0; i < 11; i++) {
    for (int j = 0; j < 11; j++) {
      EXPECT_EQ(matrix[i][j], pattern[i][j]);
    }
  }
}

TEST(PartThreeTests, GetLeastSpanningTree1) {
  s21::Graph graph;
  s21::GraphAlgorithms algo;
  graph.LoadGraphFromFile("datasets/example4.txt");

  vector<vector<int>> matrix = algo.GetLeastSpanningTree(graph);

  vector<vector<int>> tmp_matrix = {
      {-1, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      EXPECT_EQ(tmp_matrix[i][j], matrix[i][j]);
    }
  }
}

TEST(PartThreeTests, GetLeastSpanningTree2) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/example5.txt");
  s21::GraphAlgorithms algo;
  vector<vector<int>> pattern = {{0, 0, 1, 0, 0, 0}, {0, 0, 5, 0, 3, 0},
                                 {1, 5, 0, 0, 0, 4}, {0, 0, 0, 0, 0, 2},
                                 {0, 3, 0, 0, 0, 0}, {0, 0, 4, 2, 0, 0}};
  vector<vector<int>> matrix = algo.GetLeastSpanningTree(graph);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      EXPECT_EQ(matrix[i][j], pattern[i][j]);
    }
  }
}

TEST(TSMTest, AntAlgorithms4x4) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/TSM4x4.txt");
  s21::GraphAlgorithms algo;
  s21::TsmResult result;
  result = algo.SolveTravelingSalesmanProblem(graph);

  EXPECT_LT(result.distance, 95);

  std::cout << "Distance:" << result.distance << "\n";
  std::cout << "Vertices:"
            << "\t";
  for (int i = 0; i < 5; i++) {
    std::cout << result.vertices[i] << "  ";
  }
  std::cout << "\n";
}

TEST(TSMTest, AntAlgorithmsError) {
  s21::Graph graph;
  graph.LoadGraphFromFile("datasets/TSM6x6.txt");
  s21::GraphAlgorithms algo;
  s21::TsmResult result;
  result = algo.SolveTravelingSalesmanProblem(graph);
  EXPECT_EQ(result.distance, -2);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
