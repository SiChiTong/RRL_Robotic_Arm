#ifndef DEF_TEMPLATE
#define DEF_TEMPLATE

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cctype>
#include <math.h>
#include <map>

double getValue(std::string substring);
double computeFormula(double value1,double value2, std::string function);
std::string extractFormula(std::string substring, std::size_t pos);
double convertFormula(std::string substring);
std::string convertFormulas(std::string converted_template);
std::string convertVariables(std::string string_template, std::map<std::string, double> dictionary);
std::string generateModel(std::string string_template, std::map<std::string, double> dictionary);
std::map<std::string, double> loadDictionary(std::string path);
std::string getFileAsString(std::string path);

#endif
