#include "s21_graph_algorithms.h"

#include "s21_graph.h"

namespace s21 {
void TsmResult::Clear(bool is_shortest_path) {
  is_shortest_path ? distance = INT_MAX : distance = 0;
  vertices.clear();
}

GraphAlgorithms::~GraphAlgorithms() {
  if (pheromones_matrix_ != nullptr) FreeMatrix(pheromones_matrix_, nodes_);
  if (added_pheromones_matrix_ != nullptr)
    FreeMatrix(added_pheromones_matrix_, nodes_);
}

// -returned_value[0] < 0 - error
vector<int> GraphAlgorithms::DepthFirstSearch(Graph &graph, int start_vertex) {
  vector<int> vertices_vector{};
  int **adjacency_matrix = graph.GetMatrix();

  if (graph.CheckValid(0) == Graph::kBadPath) {
    vertices_vector.push_back(-1);
  } else if (start_vertex < 1 || start_vertex > graph.GetSize()) {
    vertices_vector.push_back(-2);
  } else {
    set<int> visited_vertices;
    Stack<int> stack{};
    stack.push(start_vertex - 1);
    while (stack.size()) {
      int tmp = stack.top();
      if (visited_vertices.find(stack.top()) == visited_vertices.end())
        vertices_vector.push_back(stack.top() + 1);
      visited_vertices.insert(vertices_vector.back() - 1);
      stack.pop();
      for (int i = 0; i < graph.GetSize(); ++i) {
        if ((adjacency_matrix[tmp][i] || adjacency_matrix[i][tmp]) &&
            visited_vertices.find(i) == visited_vertices.end())
          stack.push(i);
      }
    }
  }
  return vertices_vector;
};

// -returned_value[0] < 0 - error
vector<int> GraphAlgorithms::BreadthFirstSearch(Graph &graph,
                                                int start_vertex) {
  vector<int> vertices_vector{};
  int **adjacency_matrix = graph.GetMatrix();

  if (graph.CheckValid(0) == Graph::kBadPath) {
    vertices_vector.push_back(-1);
  } else if (start_vertex < 1 || start_vertex > graph.GetSize()) {
    vertices_vector.push_back(-2);
  } else {
    set<int> set;
    Queue<int> vertices{};
    vertices.push(start_vertex - 1);
    while (vertices.size()) {
      int tmp = vertices.front();
      if (set.find(vertices.front()) == set.end()) {
        vertices_vector.push_back(vertices.front() + 1);
        set.insert(vertices.front());
      }
      vertices.pop();
      for (int i = 0; i < graph.GetSize(); ++i) {
        if ((adjacency_matrix[tmp][i] || adjacency_matrix[i][tmp]) &&
            set.find(i) == set.end())
          vertices.push(i);
      }
    }
  }
  return vertices_vector;
};

// -returned_value < 0 - error
int GraphAlgorithms::GetShortestPathBetweenVertices(Graph &graph, int vertex1,
                                                    int vertex2) {
  int result = 0;
  int size = graph.GetSize();
  int **adjacency_matrix = graph.GetMatrix();
  if (graph.CheckValid(0) == Graph::kBadPath) {
    result = -1;
  } else if (vertex1 < 1 || vertex2 < 1 || vertex1 > size || vertex2 > size) {
    result = -2;
  } else {
    vector<unsigned int> dist(size, UINT_MAX);
    dist[vertex2 - 1] = 0;
    set<int> used{};

    unsigned int min_dist = 0;
    int min_vertex = vertex2 - 1;

    while ((int)used.size() != size) {
      int i = min_vertex;
      used.insert(i);
      for (int j = 0; j < size; ++j)
        if (adjacency_matrix[i][j] &&
            dist[i] + adjacency_matrix[i][j] < dist[j])
          dist[j] = dist[i] + adjacency_matrix[i][j];
      min_dist = 0 - 1;
      for (int j = 0; j < size; ++j)
        if (used.find(j) == used.end() && dist[j] < min_dist) {
          min_dist = dist[j];
          min_vertex = j;
        }
    }
    result = dist[vertex1 - 1];
  }
  return result;
};

// -returned_value[0][0] < 0 - error
vector<vector<int>> GraphAlgorithms::GetShortestPathsBetweenAllVertices(
    Graph &graph) {
  int size = graph.GetSize();
  int **adjacency_matrix = graph.GetMatrix();

  vector<vector<int>> ShortestPathsMatrix(size, vector<int>(size, -1));
  if (graph.CheckValid(0) == Graph::kBadPath || !size) {
    if (!ShortestPathsMatrix.size()) {
      vector<int> tmp(1, -1);
      ShortestPathsMatrix.push_back(tmp);
    }
  } else {
    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < size; ++j) {
        if (adjacency_matrix[i][j])
          ShortestPathsMatrix[i][j] = adjacency_matrix[i][j];
        else if (i == j)
          ShortestPathsMatrix[i][j] = 0;
        else
          ShortestPathsMatrix[i][j] = INT_MAX;
      }
    }
    for (int i = 0; i < size; ++i) {
      FloydHelper(i, ShortestPathsMatrix);
    }
  }
  return ShortestPathsMatrix;
}

// -returned_value[0][0] < 0 - error
vector<vector<int>> GraphAlgorithms::GetLeastSpanningTree(Graph &graph) {
  int size = graph.GetSize();
  vector<vector<int>> least_spanning_tree(size, vector<int>(size, 0));
  int **adjacency_matrix = graph.GetMatrix();
  int status = graph.CheckValid(0);

  if (status == Graph::kBadPath || status == Graph::kDirectedGraph || !size) {
    if (!least_spanning_tree.size()) {
      vector<int> tmp(1, -1);
      least_spanning_tree.push_back(tmp);
    } else {
      least_spanning_tree[0][0] = -1;
    }
  } else {
    set<int> visited_vertices{};
    visited_vertices.insert(0);
    while ((int)visited_vertices.size() != size) {
      std::pair<int, int> edge =
          CheapestEdgeForMST(size, visited_vertices, adjacency_matrix);
      least_spanning_tree[edge.first][edge.second] =
          adjacency_matrix[edge.first][edge.second];
      least_spanning_tree[edge.second][edge.first] =
          adjacency_matrix[edge.first][edge.second];
      visited_vertices.insert(edge.second);
    };
  }
  return least_spanning_tree;
}

// -returned_value.distance < 0 - error
TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(Graph &graph) {
  if (to_reset_matrices_) TravellingSalesmanReset(graph);
  int status = graph.CheckValid(0);

  if (status == Graph::kBadPath) {
    shortest_path_.distance = -1;
  } else if (status != Graph::kOk) {
    shortest_path_.distance = -2;
  } else {
    int counter = 1000;
    while (counter) {
      int actual_distance = shortest_path_.distance;
      for (int i = 0; i < nodes_; ++i) {
        AntGoingRound(i, graph, shortest_path_);
      }

      UpdatePheromones();
      SquareMatrixSetter(added_pheromones_matrix_, 0, '=');

      if (shortest_path_.distance < actual_distance)
        counter = 1000;
      else
        --counter;
    }
  }
  return shortest_path_;
}

void GraphAlgorithms::PrintPheromonesMatrix() {
  std::cout << "\n\n";
  for (int i = 0; i < nodes_; ++i) {
    for (int j = 0; j < nodes_; ++j) {
      i != j ? std::cout << pheromones_matrix_[i][j] << "  "
             : std::cout << "---  ";
    }
    std::cout << std::endl;
  }
}

void GraphAlgorithms::UpdatePheromones() {
  SquareMatrixSetter(pheromones_matrix_, kRo, '*');
  for (int i = 0; i < nodes_; ++i) {
    for (int j = 0; j < nodes_; ++j)
      pheromones_matrix_[i][j] += added_pheromones_matrix_[i][j];
  }
}

void GraphAlgorithms::AntGoingRound(int starting_node, Graph &graph,
                                    TsmResult &shortest_path) {
  int first_node = starting_node;
  std::set<int> nodes_to_visit;
  for (int i = 0; i < nodes_; ++i)
    if (i != starting_node) nodes_to_visit.insert(i);

  while (starting_node != -1) {
    int prev_node = starting_node;
    ant_.vertices.push_back(starting_node);
    nodes_to_visit.erase(starting_node);
    starting_node = ChooseNextNode(
        ReturnProbabilities(graph, nodes_to_visit, starting_node));
    if (starting_node != -1)
      ant_.distance += distance_matrix_[prev_node][starting_node];
  }
  double last_step = distance_matrix_[ant_.vertices.back()][first_node];
  if (nodes_to_visit.empty() && last_step) {
    ant_.distance += last_step;
    ant_.vertices.push_back(first_node);
    AddPheromones();
    if (ant_.distance < shortest_path.distance) {
      shortest_path.distance = ant_.distance;
      for (auto it = ant_.vertices.begin(), end = ant_.vertices.end();
           it != end; ++it)
        *it += 1;
      shortest_path.vertices.swap(ant_.vertices);
    }
  }
  ant_.Clear(false);
};

void GraphAlgorithms::AddPheromones() {
  double pheromones_value = kQ / ant_.distance;
  for (auto it_first = ant_.vertices.begin(), it_second = it_first + 1,
            it_end = ant_.vertices.end();
       it_second != it_end; ++it_first, ++it_second) {
    added_pheromones_matrix_[*it_first][*it_second] += pheromones_value;
    added_pheromones_matrix_[*it_second][*it_first] += pheromones_value;
  }
}

void GraphAlgorithms::TravellingSalesmanReset(Graph &graph) {
  if (pheromones_matrix_ != nullptr) {
    FreeMatrix(pheromones_matrix_, nodes_);
    pheromones_matrix_ = nullptr;
  }
  if (added_pheromones_matrix_ != nullptr) {
    FreeMatrix(added_pheromones_matrix_, nodes_);
    added_pheromones_matrix_ = nullptr;
  }

  nodes_ = graph.GetSize();
  shortest_path_.Clear(true);
  distance_matrix_ = graph.GetMatrix();

  pheromones_matrix_ = AllocMatrix(nodes_);
  added_pheromones_matrix_ = AllocMatrix(nodes_);
  SquareMatrixSetter(pheromones_matrix_, kInitialPheromones, '=');
  SquareMatrixSetter(added_pheromones_matrix_, 0.0, '=');
  ant_.Clear(false);
  to_reset_matrices_ = false;
};

void GraphAlgorithms::FreeMatrix(double **matrix, int size) {
  for (int i = 0; i < size; ++i) {
    delete[] matrix[i];
  }
  delete[] matrix;
}

double **GraphAlgorithms::AllocMatrix(int size) {
  double **matrix = new double *[size];
  for (int i = 0; i < size; ++i) {
    matrix[i] = new double[size];
  }
  return matrix;
}

void GraphAlgorithms::SquareMatrixSetter(double **matrix, double value,
                                         char operation) {
  for (int i = 0; i < nodes_; ++i)
    for (int j = 0; j < nodes_; ++j)
      if (operation == '=')
        matrix[i][j] = value;
      else if (operation == '*')
        matrix[i][j] *= value;
}

intention_pair GraphAlgorithms::ReturnProbabilities(
    Graph &graph, std::set<int> &nodes_to_visit, const int from) {
  intention_pair list_of_intentions;
  double sum_of_intentions = 0;

  for (int node_to_visit = 0, max = graph.GetSize(); node_to_visit < max;
       ++node_to_visit) {
    if (distance_matrix_[from][node_to_visit] &&
        nodes_to_visit.find(node_to_visit) != nodes_to_visit.end()) {
      double P = ReturnProbability(from, node_to_visit, distance_matrix_);
      std::pair<int, double> node_available(node_to_visit, P);
      list_of_intentions.push_back(node_available);
      sum_of_intentions += P;
    }
  }

  for (auto it_begin = list_of_intentions.begin(),
            it_end = list_of_intentions.end();
       it_begin != it_end; ++it_begin) {
    (*it_begin).second /= sum_of_intentions;
  }

  return list_of_intentions;
}

double GraphAlgorithms::ReturnProbability(const int &i, const int &j,
                                          int **distance_matrix) {
  return pow(pheromones_matrix_[i][j], kAlpha) *
         pow(1.0 / distance_matrix[i][j], kBeta);
}

int GraphAlgorithms::ChooseNextNode(intention_pair list) {
  int answer = -1;
  if (!list.empty()) {
    answer = list.front().first;
    double real = RandomReal();
    double level = list.front().second;
    while (level < real) {
      list.pop_front();
      level += list.front().second;
      answer = list.front().first;
    }
  }
  return answer;
}

double GraphAlgorithms::RandomReal() {
  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_real_distribution<> random_real(0.0, 1.0);
  return random_real(rng);
}

std::pair<int, int> GraphAlgorithms::CheapestEdgeForMST(const int size,
                                                        const set<int> &visited,
                                                        int **matrix) {
  int cheapest_edge = INT_MAX;
  std::pair<int, int> minimal_edge(std::make_pair(0, 0));
  for (auto it = visited.cbegin(), end = visited.cend(); it != end; ++it) {
    for (int j = 0; j < size; ++j) {
      if (visited.find(j) == visited.end() && matrix[*it][j] <= cheapest_edge &&
          matrix[*it][j] != 0) {
        cheapest_edge = matrix[*it][j];
        minimal_edge.first = *it;
        minimal_edge.second = j;
      }
    }
  }
  return minimal_edge;
}

void GraphAlgorithms::FloydHelper(const int node, vector<vector<int>> &graph) {
  int size = graph.size();
  for (int i = 0; i < size; ++i) {
    for (int j = 0; j < size; ++j) {
      if (graph[i][node] + graph[node][j] > 0)
        graph[i][j] = std::min(graph[i][j], graph[i][node] + graph[node][j]);
    }
  }
}

}  // namespace s21
