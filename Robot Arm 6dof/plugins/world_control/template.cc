#include "template.hh"

double computeFormula(double value1,double value2, const std::string& function)
{
  if( function == "multiply" )
    return value1 * value2;
  else if(function == "divide")
    return value1 / value2;
  else if(function == "add")
    return value1 + value2;
  else if(function == "substract")
    return value1 - value2;
  else if(function == "power")
    return std::pow(value1,value2);
  else if(function == "cos")
    return std::cos(value1);
  else if(function == "sin")
    return std::sin(value1);
  else if(function == "tan")
    return std::tan(value1);
  else if(function == "grad")
    return value1*M_PI/180;
  else
    std::printf("[Error] Unknown Formula %s!\n", function.c_str());
   return 0;
}

std::string extractFormula(const std::string& substring, std::size_t pos)
{
  int count = 1;
  std::size_t posend = pos;
  //go to opening '('
  while(substring[posend] != '(')
  {
    posend++;
  }
  //go to closing ')'
  posend++;
  while(count != 0)
  {
    if(substring[posend] == '(')count++;
    if(substring[posend] == ')')count--;
    posend++;
  }
  return substring.substr(pos,posend-pos);
}

double convertFormula(const std::string& substring)
{
  std::size_t pos = 0;
  std::size_t posit = 0;
  std::string value1 = "";
  std::string value2 = "";
  std::string function = "";
  // Get name of function
  pos++;
  posit = pos;
  while(substring[posit] != '(')
  {
    posit++;
  }
  function = substring.substr(pos,posit-pos);

  // Get value1
  int count = 1;
  posit++;
  std::size_t poscomma = posit;
  while(count != 0)
  {
    if(count == 1 && substring[poscomma] == ',')
    {
      value1 = substring.substr(posit,poscomma-posit);
      posit = poscomma;
    }
    if(substring[poscomma] == '('){count++;}
    if(substring[poscomma] == ')'){count--;}
    poscomma++;
  }

  // Get value2
  value2 = substring.substr(posit+1,poscomma-posit-2);
  return computeFormula(getValue(value1), getValue(value2), function);
}

double getValue(const std::string& substring)
{
  if(substring[0] == '&')
  {
    return convertFormula(substring);
  }else{
    std::printf("[Converting]  %s\n", substring.c_str());
    double valuedouble =  std::stod(substring);
    return valuedouble;
  }
}

std::string convertFormulas(const std::string& converted_template)
{
  std::string copyconverted_template = converted_template;
  std::size_t pos = 0;
  char formulaMark = '&';
  while( (pos = copyconverted_template.find(formulaMark) ) != std::string::npos)
  {
    // Get the entire formula
    std::string stringformula = extractFormula(copyconverted_template, pos);
    //std::printf("[Convert] %s\n", stringformula.c_str());
    // Replace the entire formula by the value as a string
    std::ostringstream strs;
    strs << convertFormula(stringformula);
    const std::string str = strs.str();
    copyconverted_template.replace(pos,stringformula.length(), str);
  }
  return copyconverted_template;
}

std::string convertVariables(const std::string& string_template, std::map<std::string, double>& dictionary)
{
  std::map<std::string, double>::iterator it;
  std::string copystring_template = string_template;
  std::size_t pos = 0;
  for(it = dictionary.begin() ; it != dictionary.end() ; ++it)
  {
    while( (pos = copystring_template.find(it->first) ) != std::string::npos)
    {
      std::ostringstream strs;
      strs << it->second;
      const std::string str = strs.str();
      copystring_template.replace( pos, (it->first).length(), str);
    }
  }
  return copystring_template;
}

std::string generateModel(const std::string& string_template, std::map<std::string, double>& dictionary)
{
  std::string converted_template = convertVariables(string_template, dictionary);
  return convertFormulas(converted_template);
}

std::map<std::string, double> loadDictionary(const std::string& path)
{
  std::ifstream f(path.c_str(), std::ios::in);
  std::map<std::string, double> dictionary;
  std::string line;
  std::string keystring;
  std::string valuestring;
  double valuedouble;
  if(f){
    while(std::getline(f, line))
    {
      //remove spaces
      line.erase(std::remove(line.begin(), line.end(), ' '), line.end());
      //check if syntax ok
      if(line[0] == '$')
      {
        std::size_t pos = line.find('=');
        keystring = line.substr(0,pos);
        valuestring = line.substr(pos+1,line.length()-pos+1);
        valuedouble =  std::stod(valuestring);
        dictionary[keystring] = valuedouble;
      }
    }
    f.close();
  }else{
      std::printf("[Error] while loading file %s!\n", path.c_str());
  }
  return dictionary;
}


std::string getFileAsString(const std::string& path)
{
  std::ifstream f(path.c_str(), std::ios::in);
  std::string buffer;
  std::string line;
  if(f){
    while(std::getline(f, line))
    {
      buffer = buffer + line;
    }
    f.close();
  }else{
      std::printf("[Error] while loading file %s!\n", path.c_str());
  }
  return buffer;
}
