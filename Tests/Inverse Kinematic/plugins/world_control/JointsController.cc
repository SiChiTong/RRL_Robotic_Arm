#include "JointsController.hh"

JointsController::JointsController()
{

}

JointsController::~JointsController()
{

}

void JointsController::Init(gazebo::physics::JointControllerPtr jointController,ArmStructure *armStructure ,std::map<std::string, double> dictionary)
{
  this->errormax = dictionary["$error_max_joints"];
  this->errorflex = dictionary["$error_flex_joints"];
  this->reflex_amp = dictionary["$reflex_amp"];
  this->armStructure = armStructure;
  this->jointController = jointController;
  std::vector<gazebo::physics::JointPtr> joints = this->armStructure->GetAllJoints();
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    this->targets[(*it)->GetName()] = (*it)->GetAngle(0).Radian();
    this->jointController->SetPositionTarget((*it)->GetScopedName(), this->targets[(*it)->GetName()] );
  }
}

void JointsController::SetTargetsToZero()
{
  std::vector<gazebo::physics::JointPtr> joints = this->armStructure->GetAllJoints();
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    this->targets[(*it)->GetName()] = 0;
    this->jointController->SetPositionTarget((*it)->GetScopedName(), this->targets[(*it)->GetName()] );
  }
}


// Set relative target (add to current target)
void JointsController::SetRelativeTargets(std::string name, std::vector<double> targets)
{
  std::vector<gazebo::physics::JointPtr> joints = this->armStructure->GetJoints(name);
  std::vector<std::vector<double>> limits = this->armStructure->GetLimits(name);
  if(joints.size() != targets.size()) std::cout << "[Error] joints and target size not matching in JointsController::setJointsTarget" << std::endl;
  unsigned int i = 0;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    std::string jointname = (*it)->GetName();
    double angle = (*it)->GetAngle(0).Radian();
    double angleLowerLimit = limits[i][0];
    double angleUpperLimit = limits[i][1];
    this->targets[jointname] = angle + targets[i];
    // We avoid the situation where we go a bit farther than the limit and keep this as target
    if(this->targets[jointname] <= (angleLowerLimit) ) {this->targets[jointname] = angleLowerLimit;}
    if(this->targets[jointname] >= (angleUpperLimit) ) {this->targets[jointname] = angleUpperLimit;}
    // Update controller
    this->jointController->SetPositionTarget((*it)->GetScopedName(), this->targets[(*it)->GetName()] );
    i++;
  }
}

// TO IMPROVE: we should avoid using SetForce since it might be overwritten
void JointsController::ReflexToTouch()
{
  std::vector<gazebo::physics::JointPtr> joints = this->armStructure->GetAllJoints();
  std::map<std::string, gazebo::common::PID> PIDs = this->jointController->GetPositionPIDs();
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    double force = PIDs[(*it)->GetScopedName()].GetCmd();
    (*it)->SetForce(0,-force * this->reflex_amp);
  }
}

bool JointsController::AreTargetsReached(std::string name, bool strict)
{
  double errormax = this->errorflex;
  if(strict){ errormax = this->errormax;}
  std::vector<gazebo::physics::JointPtr> joints = this->armStructure->GetJoints(name);
  unsigned int count = joints.size();
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    if( fabs( (*it)->GetAngle(0).Radian() - this->targets[(*it)->GetName()] ) < errormax )
    {
      count--;
    }
  }
  return (count == 0);
}
