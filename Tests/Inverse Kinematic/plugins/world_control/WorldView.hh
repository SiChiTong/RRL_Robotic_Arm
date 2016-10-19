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
    // PRIVATE METHODS

  public:
    // PUBLIC METHODS
    WorldView();
    ~WorldView();
    void Init(gazebo::physics::WorldPtr world);
    std::vector<gazebo::math::Vector3> GetBoundingBoxGrabbingPoints(std::string objname, double offset);
    std::vector<gazebo::math::Vector3> GetBoxGrabbingPoints(std::string objname, double offset);



};
#endif
