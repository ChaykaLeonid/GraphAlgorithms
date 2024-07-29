#include "s21_graph.h"

#include <cstdio>

namespace s21 {
int Graph::CheckValid(int pre = kOk) {
  int result = pre;

  if (adjacency_matrix_ == nullptr && result == kOk) {
    result = kBadPath;
  } else if (vertex_count_ < 3) {
    result = kSmallGraph;
  } else if (result == kOk) {
    for (int i = 0; i < vertex_count_; ++i)
      for (int j = 0; j < vertex_count_; j++)
        if (i != j && adjacency_matrix_[i][j] != adjacency_matrix_[j][i])
          result = kDirectedGraph;
  }
  return result;
}

int Graph::LoadGraphFromFile(string filename) {
  int error = 0;
  string file_data = "", buffer = "";
  int column_number = 0, line_number = 0;

  file_data = ReadFile(filename);
  if (!file_data.empty()) {
    while (file_data.size() != 0 && !error) {
      if (std::isdigit(file_data[0]))
        buffer.push_back(file_data[0]);
      else if (buffer.size() != 0) {
        error = Parse(buffer, column_number, line_number);
        buffer.erase();
        ++column_number;
      }
      if (file_data[0] == '\n') {
        if (line_number != 0 && column_number != vertex_count_) error = 3;
        column_number = 0;
        ++line_number;
      }
      file_data.erase(0, 1);
    }
  } else {
    error = 1;
  }
  int status = CheckValid(error);
  if (status != 0 && status != 2) Remove();
  return status;
}

int Graph::ExportGraphToFile(string filename) {
  int status = 0;
  if (!adjacency_matrix_) {
    std::ofstream output;
    output.open(filename);
    if (!output.is_open()) {
      status = 1;
    } else {
      output << vertex_count_ << std::endl;
      for (int i = 0; i < vertex_count_; i++) {
        for (int j = 0; j < vertex_count_; j++) {
          output << adjacency_matrix_[i][j] << "\t";
        }
        output << std::endl;
      }
      output.close();
    }
  }
  return status;
}

void Graph::ExportGraphToDot(string filename) {
  char initial_letter = 97;
  std::ofstream output;
  output.open(filename);
  if (!output.fail()) {
    output << "graph graphname {" << std::endl;
    for (int i = 0; i < vertex_count_; i++) {
      output << "  " << initial_letter++ << ";\n";
    }
    DotFileCompleting(output);
    output << "}";
  };
};

void Graph::Print() {
  for (int i = 0; i < vertex_count_; i++) {
    for (int j = 0; j < vertex_count_; j++)
      std::cout << adjacency_matrix_[i][j] << " ";
    std::cout << std::endl;
  }
}

void Graph::DotFileCompleting(std::ofstream &output) {
  char initial_letter = 97;
  string link;
  CheckValid() == 2 ? link = " -> " : link = " -- ";
  for (int i = 0; i < vertex_count_ - 1; i++) {
    for (int j = i; j < vertex_count_; j++) {
      if (adjacency_matrix_[i][j])
        output << "  " << (char)(initial_letter + i) << link
               << (char)(initial_letter + j) << ";\n";
      if (CheckValid() == 2 && adjacency_matrix_[j][i]) {
        output << "  " << (char)(initial_letter + j) << link
               << (char)(initial_letter + i) << ";\n";
      }
    }
  }
}

string Graph::ReadFile(string filename) {
  std::ifstream input;
  string tmp;
  string _buff = "";

  input.open(filename);
  if (input.is_open()) {
    while (!input.eof()) {
      if (std::getline(input, tmp)) {
        _buff.append(tmp);
        _buff.append("\n");
      } else {
        _buff.append("\n");
      }
    }
    _buff.erase(_buff.size() - 1);
    input.close();
  }
  return _buff;
}

void Graph::Create(int n) {
  if (adjacency_matrix_ != NULL) Remove();
  vertex_count_ = n;

  adjacency_matrix_ = new int *[vertex_count_];
  for (int i = 0; i < vertex_count_; i++) {
    adjacency_matrix_[i] = new int[vertex_count_];
    for (int j = 0; j < vertex_count_; j++) adjacency_matrix_[i][j] = 0;
  }
}

int Graph::Parse(string filedata, int col, int row) {
  int error = 0;
  if (row == 0) {
    Create(stoi(filedata));
  } else {
    row -= 1;
    if (col < vertex_count_ && row < vertex_count_) {
      adjacency_matrix_[col][row] = stoi(filedata);
    } else {
      error = 3;
    }
  }
  return error;
}

void Graph::Remove() {
  if (adjacency_matrix_) {
    for (int i = 0; i < vertex_count_; ++i) delete[] adjacency_matrix_[i];
    delete[] adjacency_matrix_;
    adjacency_matrix_ = nullptr;
  }
  vertex_count_ = 0;
}
}  // namespace s21
