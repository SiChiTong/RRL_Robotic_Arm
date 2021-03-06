#include "Statistics.hh"

Statistics::Statistics()
{
  
}

Statistics::~Statistics()
{
  this->CloseFiles();
}

void Statistics::CloseFiles()
{
  typedef std::map<std::string, std::ofstream>::iterator it_type;
  for(it_type iterator = this->files.begin(); iterator != this->files.end(); iterator++) 
  {
     iterator->second.close();
  }
}

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}

void Statistics::CreateFile(std::string filename)
{
  this->files[filename] = std::ofstream();
  this->files[filename].open("Data/" + filename + "(" + currentDateTime() + ").csv");
}

void Statistics::WriteInFile(std::string filename, std::string toWrite)
{
  this->files[filename] << toWrite;
}

void Statistics::WriteInFile(std::string filename, gazebo::math::Vector3 toWrite)
{
  this->files[filename] << toWrite;
}

void Statistics::WriteInFile(std::string filename, unsigned int toWrite)
{
  this->files[filename] << toWrite;
}

void Statistics::SetFileFlag(bool flag)
{
  this->fileFlag = flag;
}

bool Statistics::GetFileFlag()
{
  return this->fileFlag;
}

void Statistics::Init(unsigned int nbIterSimu, unsigned int waititer)
{
  this->nbIterSimu = nbIterSimu;
  this->iterSimu = 1;
  this->printFlag = false;
  this->waititer = waititer;
  this->waititer_i = 0;
  this->fileFlag = false;
}
void Statistics::updateSimu()
{
this->iterSimu++;
std::cout << "SIM ITER: "<< this->iterSimu-1 << std::endl;
}
// Return true if simu over
bool Statistics::isSimuOver()
{
  if(( (this->iterSimu) % (this->nbIterSimu+1) ) == 0)
  {
    if(this->printFlag == false)
    {
      std::cout << std::endl << "SIMULATION DONE " << std::endl;
      std::cout << std::endl << "Number of simulation iter: " << this->nbIterSimu << std::endl;
      this->printFlag = true;
    }
    return true;
  }
  return false;
}

void Statistics::resetWaititer()
{
  this->waititer_i = 0;
}

bool Statistics::isSimuReady()
{
  if((this->waititer_i < this->waititer)) this->waititer_i++;
  return (this->waititer_i >= this->waititer);
}

void Statistics::SetTimer(std::string name, float seconds)
{

}

bool Statistics::IsTimerOff(std::string name)
{
  return true;
}
