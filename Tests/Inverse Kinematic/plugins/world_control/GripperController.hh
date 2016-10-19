#ifndef DEF_GRIPPERCONTROLLER
#define DEF_GRIPPERCONTROLLER

#include <string>
#include <cctype>
#include <iostream>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ArmStructure.hh"

class GripperController
{
  private:
    // PRIVATE ATTRIBUTES
     unsigned int state; // 1 = "approach", 2 = "stabilization", 3 = "stable", 4 = "release",  0 = "inactive"
     gazebo::physics::JointControllerPtr jointController;
     ArmStructure *armStructure;
     std::vector<gazebo::physics::JointPtr> joints;
     std::vector<double> targets;
     std::vector<std::vector<gazebo::physics::JointPtr>> captors;
     std::vector<int> signs;
     double captorsensibility;
     double captorsmax;
     double stepsize;
     double targsize;
     double converror;
     unsigned int iter_max;
     unsigned int iter;
     unsigned int freq;
     unsigned int varfreq;

     double getCaptorsLevel(unsigned int i);
     bool approach();
     bool stabilization();
     bool release();


  public:
    // PUBLIC METHODS
    GripperController();
    ~GripperController();
    void Init(gazebo::physics::JointControllerPtr jointController,ArmStructure *armStructure ,std::map<std::string, double> dictionary);
    void Update();
    bool IsClosed();
    bool IsOpen();
    void Close();
    void Open();

};

#endif
