#ifndef DEF_WORLDVIEW
#define DEF_WORLDVIEW

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <cctype>
#include <math.h>

class WorldView
{
  private:
    // PRIVATE ATTRIBUTES
    gazebo::physics::WorldPtr world;
    std::map<std::string,gazebo::common::Time> timers;
    // PRIVATE METHODS

  public:
    // PUBLIC METHODS
    WorldView();
    ~WorldView();
    void Init(gazebo::physics::WorldPtr world);
    std::vector<gazebo::math::Vector3> GetBoundingBoxGrabbingPoints(const std::string& objname, double offset);
    std::vector<gazebo::math::Vector3> GetBoxGrabbingPoints(const std::string& objname, double offset);
    void SetTimer(const std::string& name, float seconds);
    unsigned int IsTimerOff(const std::string& name);
    double GetBoxSize(const std::string& objname);
    std::string GetBoxNameInHand(const std::vector<gazebo::math::Vector3>&);


};
#endif
