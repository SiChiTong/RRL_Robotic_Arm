#ifndef DEF_ARMCONTROLLER
#define DEF_ARMCONTROLLER

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cctype>
#include <math.h>

#include "InverseKinematics.hh"
#include "JointsController.hh"
#include "GripperController.hh"
#include "Parameters.hh"
#include "Statistics.hh"
#include "ContactsManager.hh"
#include "WorldView.hh"
//
class ArmController
{
  private:
    // PRIVATE ATTRIBUTES
    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;
    ContactsManager contactsManager;
    std::map<std::string,InverseKinematics> inverseKinematics;
    JointsController jointsController;
    GripperController gripperController;
    ArmStructure armStructure;
    Statistics statistics;
    WorldView worldview;
    double box_x;
    double box_y;
    bool reflex;
    bool contact;
    unsigned int state;
    bool init;


    // PRIVATE METHODS
    void UpdateJoints();
    bool checkCollisions();
    void initPids(double p, double i, double d, double imax, double imin, double cmdMax, double cmdMin);
    gazebo::math::Vector3 generateRandomDiskTarget(gazebo::math::Vector3 c,double rmax);
    gazebo::math::Vector3 generateRandomSphereTarget(gazebo::math::Vector3 c,double rmax);

 public:
   // PUBLIC METHODS
   ArmController();
   ~ArmController();
   void Init(gazebo::physics::ModelPtr model);
   void Update();
 };

#endif
