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
    gazebo::physics::WorldPtr world;
    double errormax;
    double errorflex;
    double reflex_amp;
    double time_limit;
    double time_random;
    std::string state;
    gazebo::common::Time timer;
    // PRIVATE METHODS
    void initPids(double p, double i, double d, double imax, double cmdMax, double zfactor);
    void SetTimer(double seconds);
    bool IsTimerOff();

  public:
    // PUBLIC METHODS
    JointsController();
    ~JointsController();
    void Init(gazebo::physics::JointControllerPtr jointController, ArmStructure *armStructure ,gazebo::physics::WorldPtr world, std::map<std::string,double>& dictionary);
    bool AreTargetsReached(const std::string& name, bool strict);
    void ReflexToTouch();
    void SetRelativeTargets(const std::string& name, const std::vector<double>& targets);
    void SetTargetsToZero();
};
#endif
