#include "Parameters.hh"

std::map<std::string, double> loadParam(std::string path)
{
  std::ifstream f(path.c_str(), std::ios::in);
  std::map<std::string, double> dictionary;
  std::string line;
  std::string keystring;
  std::string valuestring;
  double valuedouble;
  if(f)
  {
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
        std::printf("[CONTROL] %s - %f \n", keystring.c_str(), valuedouble);
      }
    }
    f.close();
  }else{
      std::printf("[Error] while loading file %s!\n", path.c_str());
  }
  return dictionary;
}
