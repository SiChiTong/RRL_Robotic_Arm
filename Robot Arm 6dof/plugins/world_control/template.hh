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

double getValue(const std::string& substring);
double computeFormula(double value1,double value2, const std::string& function);
std::string extractFormula(const std::string& substring, std::size_t pos);
double convertFormula(const std::string& substring);
std::string convertFormulas(const std::string& converted_template);
std::string convertVariables(const std::string& string_template, std::map<std::string, double>& dictionary);
std::string generateModel(const std::string& string_template, std::map<std::string, double>& dictionary);
std::map<std::string, double> loadDictionary(const std::string& path);
std::string getFileAsString(const std::string& path);

#endif
