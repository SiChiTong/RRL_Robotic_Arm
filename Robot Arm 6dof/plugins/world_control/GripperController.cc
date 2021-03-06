#include "GripperController.hh"

GripperController::GripperController()
{

}

GripperController::~GripperController()
{

}

void GripperController::Init(gazebo::physics::JointControllerPtr jointController,ArmStructure *armStructure ,gazebo::physics::WorldPtr world , std::map<std::string, double>& dictionary)
{
  this->captorsensibility = dictionary["$captors_sensibility"];
  this->captorsmax = dictionary["$captors_max"];
  this->freq = dictionary["$update_freq"];
  this->stepsize = dictionary["$stepsize_gripper"];
  this->targsize = dictionary["$targsize_gripper"];
  this->converror = dictionary["$convergence_error_gripper"];
  this->iter_max = dictionary["$iter_max_gripper"];
  this->gripper_low_angle = dictionary["$gripper_low_angle"];
  this->releasesize = dictionary["$releasesize_gripper"];
  this->recedesize = dictionary["$recedesize_gripper"];
  this->time_limit = dictionary["$time_limit"];
  this->world = world;
  this->iter = 1;
  this->varfreq = 1;
  this->armStructure = armStructure;
  this->jointController = jointController;
  this->captors = this->armStructure->GetCaptors();
  this->signs = this->armStructure->GetFingersSigns();
  this->joints =  this->armStructure->GetFingers();
  this->state = 0;
  for(auto it = this->joints.begin(); it != this->joints.end(); it++)
  {
    this->targets.push_back(0);
  }
}

void GripperController::SetTimer(double seconds)
{
  this->timer = this->world->GetSimTime() + gazebo::common::Time(seconds,0);
}

bool GripperController::IsTimerOff()
{
  if(this->world->GetSimTime() > this->timer)
  {
    return true;
  }
  return false;
}

void GripperController::Update()
{
  if(this->varfreq == this->freq )
  {
    // 1 = "approach", 2 = "stabilization", 3 = "stable", 4 = "release",  5 = "",  0 = "inactive"
    switch(this->state)
    {
      case 1:
        if(this->approach() || this->IsTimerOff()) {this->state = 2;this->iter = 1;}
        break;
      case 2:
        if(this->stabilization() || this->IsTimerOff()) {this->state = 3;this->iter = 1;}
        break;
      case 4:
        if(this->release() || this->IsTimerOff()) {this->state = 5;this->iter = 1;}
        break;
      case 6:
        if(this->recede() || this->IsTimerOff()) {this->state = 0;this->iter = 1;}
        break;
    }
    this->varfreq = 1;
  }else{
    this->varfreq++;
  }
  this->iter++;
}

bool GripperController::IsClosed()
{
  if(this->state == 3)
  {
    return true;
  }
  return false;
}

bool GripperController::IsReleased()
{
  if(this->state == 5)
  {
    return true;
  }
  return false;
}

bool GripperController::IsOpen()
{
  if(this->state == 0)
  {
    return true;
  }
  return false;
}

void GripperController::Close()
{
  //std::cout << "Closing gripper" << std::endl;
  this->state = 1;
  for(unsigned int i = 0; i < this->joints.size(); i++)
  {
    this->targets[i] = 0;
  }
  this->SetTimer(this->time_limit);
  //std::cout << "  --> Appraoch" << std::endl;
}

void GripperController::Release()
{
  this->state = 4;
  this->SetTimer(this->time_limit);
}

void GripperController::Open()
{
  //std::cout << "Opening gripper" << std::endl;
  this->state = 6;
  this->SetTimer(this->time_limit);
}

bool GripperController::approach()
{
  unsigned int count = 0;
  unsigned int limits = 0;
  for(unsigned int i = 0; i < this->joints.size(); i++)
  {
    if(this->getCaptorsLevel(i) < this->captorsensibility)
    {
      this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(), this->joints[i]->GetAngle(0).Radian() + this->signs[i] * this->stepsize );
      count++;
    }else{
      this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(), this->joints[i]->GetAngle(0).Radian());
    }
    limits += std::fabs(this->joints[i]->GetAngle(0).Radian()) > this->gripper_low_angle; // increase still in range
  }
  if(count == 0 || limits == 0)
  {
    for(unsigned int i = 0; i < this->joints.size(); i++)
    {
      this->targets[i] = this->joints[i]->GetAngle(0).Radian() + this->signs[i] * this->stepsize;
    }
    return true;
  }
  return false;
}

bool GripperController::stabilization()
{
  /*for(unsigned int i = 0; i < this->joints.size(); i++)
  {
    std::cout << i << "    " << this->getCaptorsLevel(i) << std::endl;
  }*/
  unsigned int count = 0;
  unsigned int limits = 0;
  for(unsigned int i = 0; i < this->joints.size(); i++)
  {
    if(this->getCaptorsLevel(i) < this->captorsmax)
    {
      {
        this->targets[i] += this->signs[i] * this->targsize;
        this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(),  this->targets[i] );
      }
      count++;
    }else{
      this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(),  this->targets[i] );
    }
    limits += std::fabs(this->joints[i]->GetAngle(0).Radian()) > this->gripper_low_angle; // increase still in range
  }

  if(count == 0 || limits == 0)
  {
    /*for(unsigned int i = 0; i < this->joints.size(); i++)
    {
      std::cout << "gripper::target "<< i << ": " << this->targets[i] << std::endl;
    }*/
    return true;
  }
  return false;
}

bool GripperController::release()
{
  unsigned int count = 0;
  for(unsigned int i = 0; i < this->joints.size(); i++)
  {
    if(this->getCaptorsLevel(i) > this->captorsensibility)
    {
      this->targets[i] -=  this->signs[i] * this->releasesize;
      this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(), this->targets[i] );
      count++;
    }else{
      this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(), this->targets[i]);
    }
  }
  if(count == 0)
  {
    return true;
  }
  return false;
}

bool GripperController::recede()
{
  for(unsigned int i = 0; i < this->joints.size(); i++)
  {
    this->jointController->SetPositionTarget(this->joints[i]->GetScopedName(), this->joints[i]->GetAngle(0).Radian() - this->signs[i] * this->recedesize);
  }
  return true;
}

double GripperController::getCaptorsLevel(unsigned int i)
{
  double level = 0;
  for(auto it = this->captors[i].begin(); it != this->captors[i].end(); it++)
  {
    level += std::fabs( (*it)->GetAngle(0).Radian() );
  }
  return ( level / this->captors[i].size() );
}
