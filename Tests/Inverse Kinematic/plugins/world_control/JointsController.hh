#ifndef DEF_JOINTSCONTROLLER
#define DEF_JOINTSCONTROLLER

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cctype>
#include <math.h>
#include "ArmStructure.hh"

class JointsController
{
  private:
    // PRIVATE ATTRIBUTES
    std::map<std::string, double> targets;
    gazebo::physics::JointControllerPtr jointController;
    ArmStructure *armStructure;
    double errormax;
    double errorflex;
    double reflex_amp;
    // PRIVATE METHODS

  public:
    // PUBLIC METHODS
    JointsController();
    ~JointsController();
    void Init(gazebo::physics::JointControllerPtr jointController, ArmStructure *armStructure ,std::map<std::string, double> dictionary);
    bool AreTargetsReached(std::string name, bool strict);
    void ReflexToTouch();
    void SetRelativeTargets(std::string name, std::vector<double> targets);
    void SetTargetsToZero();
};
#endif
