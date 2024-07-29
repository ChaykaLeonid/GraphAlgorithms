LIB_NAME     := s21_graph
LIB_ALG_NAME := s21_graph_algorithms
PROGRAM_NAME := navigator

CXX          := g++
CPP_FLAGS    := -std=c++17 -pedantic
WARN_FLAGS   := -Wall -Wextra -Werror
CPP_FLAGS	 += $(WARN_FLAGS)
GTEST_FLAGS	 := -lgtest
GCOV_FLAGS	 := -fprofile-arcs -ftest-coverage

SRCS_PROGRAM := console_menu.cc ./includes/console_functions.cc
HDR_PROGRAM  := ./includes/console_functions.h

LIB_PATH = ./includes/$(LIB_NAME)
LIB_ALG_PATH = ./includes/$(LIB_ALG_NAME)

LIB          = $(LIB_NAME).a
SRC_LIB      := $(LIB_PATH).cc
HDR_LIB      := $(SRC_LIB:.cc=.h)
OBJ_LIB      := $(LIB_NAME).o

LIB_ALG      := $(LIB_ALG_NAME).a
SRC_LIB_ALG  := $(LIB_ALG_PATH).cc
HDR_LIB_ALG  := $(SRC_LIB_ALG:.cc=.h)
OBJ_LIB_ALG  := $(LIB_ALG_NAME).o

HDR_CONT	 := ./includes/libs/s21_list.h \
				./includes/libs/s21_queue.h \
				./includes/libs/s21_stack.h

TEST_NAME	 := test
SRC_TEST	 := $(TEST_NAME).cc

SRCS		 := $(SRC_LIB) $(SRC_LIB_ALG) $(SRCS_PROGRAM) $(SRC_TEST)
HDRS		 := $(HDR_LIB) $(HDR_LIB_ALG) $(HDR_PROGRAM) $(HDR_CONT)

REPORT		 := report_navigator

.PHONY: all test lint clean

all: $(PROGRAM_NAME)

s21_graph.a: $(OBJ_LIB)
	ar rcs  $(LIB) $(OBJ_LIB)

s21_graph_algorithms.a: $(OBJ_LIB_ALG)
	ar rcs  $(LIB_ALG) $(OBJ_LIB_ALG)

$(OBJ_LIB): $(SRC_LIB)
	$(CXX) $(CPP_FLAGS) -c $< -o $@

$(OBJ_LIB_ALG): $(SRC_LIB_ALG)
	$(CXX) $(CPP_FLAGS) -c $< -o $@

test: $(SRC_TEST) $(LIB) $(LIB_ALG)
	$(CXX) $(CPP_FLAGS) $^ -o $(TEST_NAME) $(GTEST_FLAGS)
	./$(TEST_NAME)

gcov_report: $(SRC_TEST) $(LIB) $(LIB_ALG)
	$(CXX) $(CPP_FLAGS) $(GTEST_FLAGS) $(GCOV_FLAGS) $^ -o $(REPORT).out
	./$(REPORT).out
	lcov -t "$(REPORT)" -o $(REPORT).info -c -d ./ --no-external --ignore-errors inconsistent --ignore-errors inconsistent,inconsistent
	genhtml ./$(REPORT).info -o ./report/
	rm -rf ./*.gcno ./*.gcda ./$(REPORT).*
	open ./report/index.html

lint: .clang-format
	@clear
	clang-format -i $(SRCS) $(HDRS)

check: .clang-format
	@clear
	clang-format -n $(SRCS) $(HDRS)

clean:
	rm -rf $(PROGRAM_NAME)
	rm -rf $(LIB)
	rm -rf $(LIB_ALG)
	rm -rf $(OBJ_LIB)
	rm -rf $(OBJ_LIB_ALG)
	rm -rf report
	rm -rf test
	rm -rf ./datasets/aboba.dot
