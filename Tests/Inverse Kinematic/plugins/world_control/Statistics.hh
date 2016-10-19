#ifndef DEF_STATISTICS
#define DEF_STATISTICS

#include <string>
#include <cctype>
#include <iostream>
#include <fstream>
#include <cctype>
#include <map>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ctime>


class Statistics
{
  private:
    // PRIVATE ATTRIBUTES
     unsigned int iterSimu;
     unsigned int nbIterSimu;
     bool printFlag;
     unsigned int waititer;
     unsigned int waititer_i;
     std::map<std::string,std::ofstream> files;
     bool fileFlag;

  public:
    // PUBLIC METHODS
    Statistics();
    ~Statistics();
    void Init(unsigned int nbIterSimu, unsigned int waititer_i);
    void updateSimu();
    bool isSimuOver();
    bool isSimuReady();
    void resetWaititer();
    void SetTimer(std::string name, float seconds);
    bool IsTimerOff(std::string name);
    void CreateFile(std::string filename);
    void WriteInFile(std::string filename, std::string toWrite);
    void WriteInFile(std::string filename, gazebo::math::Vector3 toWrite);
    void WriteInFile(std::string filename, unsigned int toWrite);
    void SetFileFlag(bool flag);
    bool GetFileFlag();
    void CloseFiles();
};

#endif
