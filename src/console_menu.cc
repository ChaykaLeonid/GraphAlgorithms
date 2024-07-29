#include "./includes/console_functions.h"

int main() {
  s21::Console console;
  int status = s21::Console::kPrintHelp;

  while (status) {
    console.Menu(status);
    console.GetValueWithMessage("Enter command number", status);
  }
  return 0;
}
