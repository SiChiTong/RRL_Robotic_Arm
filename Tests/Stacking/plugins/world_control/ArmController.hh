#ifndef DEF_ARMCONTROLLER
#define DEF_ARMCONTROLLER

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cctype>
#include <math.h>
#include <string>
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

    bool reflex;
    unsigned int state;
    unsigned int action;
    unsigned int prev_state;
    bool init;

    std::vector<std::vector<unsigned int>> graph;
    std::vector<std::vector<std::string>> param;
    std::map<std::string, double> dictionary;

    // PRIVATE METHODS
    
    bool reflexDo();

    unsigned int waitAction(unsigned int seconds);
    unsigned int placeFingersRelativeToObjectAction(const std::string& objectName,const  std::string& where, bool isControllerPrecise);
    unsigned int placeObjectRelativeToObjectAction(const std::string& objectName,const  std::string& where, bool isControllerPrecise);
    unsigned int putDownObjectAction();
    unsigned int closeHandAction();
    unsigned int openHandAction();
    unsigned int takeHeightAction();
    unsigned int getBackAction();
    unsigned int standStraightAction();

 public:
   // PUBLIC METHODS
   ArmController();
   ~ArmController();
   void Init(gazebo::physics::ModelPtr model);
   void UpdateModel(const std::vector<std::vector<unsigned int>>& graph,const std::vector<std::vector<std::string>>& param);
   unsigned int Update();
 };

#endif
