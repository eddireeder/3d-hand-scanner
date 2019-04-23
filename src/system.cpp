#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <array>

#include "system.h"


// Function to convert a string to a char array (C-string)
char * to_char_array(std::string string) {
  char * char_array = new char [string.length() + 1];
  strcpy(char_array, string.c_str());
  return char_array;
}


// Function to execute bash commands with input and output
std::string exec(const std::string string_cmd) {
  char* cmd = to_char_array(string_cmd);
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}