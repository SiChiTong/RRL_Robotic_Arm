#include "ArmStructure.hh"

ArmStructure::ArmStructure()
{
  this->maxRank = 0;
}

ArmStructure::~ArmStructure()
{
 
}

void ArmStructure::Init(gazebo::physics::JointPtr rootJoint, double limitoffset)
{
  this->limitoffset = limitoffset;
  this->buildStructure(rootJoint, 0);
  for(unsigned int i = 0; i < this->fingers.size(); i++)
  {
    this->findCaptors(this->fingers[i],i);
  }

  for(unsigned int i = 0; i<this->fingers.size(); i++)
  {
    std::cout << "Finger " << i << ":  " << this->fingers[i]->GetName() << std::endl << "Touch captors: " << std::endl;
    for(unsigned int j = 0; j<this->captors[i].size(); j++)
    {
      std::cout << "   Captor " << j << ":  " << this->captors[i][j]->GetName() << std::endl;
    }
    std::cout << "Signs:   " << this->GetFingersSigns()[i] << std::endl;
  }
}

void ArmStructure::findCaptors(gazebo::physics::JointPtr rootJoint, unsigned int i)
{
  std::vector<gazebo::physics::JointPtr> nextJoints = rootJoint->GetChild()->GetChildJoints();
  for(auto it = nextJoints.begin(); it != nextJoints.end(); it++)
  {
    if( (*it)->GetName().find("#touch") != std::string::npos )
    {
      this->captors[i].push_back(*it);
    }
    this->findCaptors(*it,i);
  }
}

void ArmStructure::buildStructure(gazebo::physics::JointPtr rootJoint, unsigned int indice)
{
  // Update next ones
  std::vector<gazebo::physics::JointPtr> nextJoints = rootJoint->GetChild()->GetChildJoints();
  std::vector<std::string>::size_type i = 0;
  while ( i < nextJoints.size() )
  {
    if(nextJoints[i]->GetName().find("#joint") == std::string::npos)
    {
        nextJoints.erase( nextJoints.begin() + i );
    } else {
        i++;
    }
  }

  if(rootJoint->GetName().find("#joint") != std::string::npos)
  {
    // Add a linkage
    struct linkage newlink;
    newlink.rank = indice;
    newlink.nbchild = nextJoints.size();
    newlink.jointName = rootJoint->GetName();
    newlink.linkName = rootJoint->GetChild()->GetName();
    newlink.jointScopedName = rootJoint->GetScopedName();
    newlink.linkScopedName = rootJoint->GetChild()->GetScopedName();
    newlink.joint = rootJoint;
    newlink.link = rootJoint->GetChild();
    // Map keeps link between name and position in joints and linkages
    this->map.insert(std::pair<std::string,unsigned int>(newlink.jointName,this->joints.size()));
    this->linkages.push_back(newlink);
    this->joints.push_back(rootJoint);
    // Update max rnk if needed
    if(this->maxRank < indice) this->maxRank = indice;
  }

  // Add joint in fingers if this is the case
  if( rootJoint->GetName().find("#finger") != std::string::npos )
  {
    this->fingers.push_back(rootJoint);
    this->captors.push_back(std::vector<gazebo::physics::JointPtr>{});
  }
  for(auto it = nextJoints.begin(); it != nextJoints.end(); it++)
  {
    buildStructure(*it, indice+1);
  }
}

void ArmStructure::CreateChain(std::string name,unsigned int min,unsigned int max)
{
  std::vector<gazebo::physics::JointPtr> joints;
  for(auto it = this->linkages.begin(); it != this->linkages.end(); it++)
  {
    if(it->rank >= min && it->rank <= max)
    {
      joints.push_back(it->joint);
    }
  }
  this->chains.insert(std::pair<std::string,std::vector<gazebo::physics::JointPtr>>(name,joints));
  joints = this->chains[name];
  this->buildChildren(name);

  // For verification
  std::cout << std::endl << std::endl << "[ARM STRUCTURE :: CHAIN CREATION]" << std::endl;
  std::cout << "Chain " << name << " created:" << std::endl;
  for(unsigned int i = 0; i < this->chains[name].size(); i++)
  {
    std::cout << i << " - " << this->chains[name][i]->GetName() << "  children = ";
    for(unsigned int j = 0; j < this->chainsChildren[name][i].size(); j++)
    {
      std::cout << " " << this->chainsChildren[name][i][j] << " " ;
    }
    std::cout << std::endl;
    std::cout << "Lower limit: " << this->chains[name][i]->GetLowerLimit(0).Radian() << "  adding offset:" << this->limitoffset << std::endl;
    std::cout << "Higher limit: " << this->chains[name][i]->GetUpperLimit(0).Radian() << "  adding offset:" << -this->limitoffset << std::endl;
  }
}
//
void ArmStructure::buildChildren(std::string name)
{
  std::vector<gazebo::physics::JointPtr> joints = this->chains[name];
  std::vector<std::vector<unsigned int>> childrenall;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    std::vector<unsigned int> children;
    std::vector<gazebo::physics::JointPtr> nextJoints = (*it)->GetChild()->GetChildJoints();
    unsigned int num = 0;
    for(auto tmp = joints.begin(); tmp != joints.end(); tmp++)
    {
      for(auto nxt = nextJoints.begin(); nxt != nextJoints.end(); nxt++)
      {
        if( (*tmp) == (*nxt) )
        {
          children.push_back(num);
        }
      }
      num++;
    }
    childrenall.push_back(children);
  }
  this->chainsChildren.insert(std::pair<std::string,std::vector<std::vector<unsigned int>>>(name,childrenall));
}

std::vector<gazebo::physics::JointPtr> ArmStructure::GetJoints(std::string name)
{
  return this->chains[name];
}

std::vector<gazebo::physics::JointPtr> ArmStructure::GetAllJoints()
{
  return this->joints;
}

std::vector<gazebo::math::Vector3> ArmStructure::GetAxis(std::string name)
{
  std::vector<gazebo::physics::JointPtr> joints = this->chains[name];
  std::vector<gazebo::math::Vector3> axis;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    axis.push_back((*it)->GetLocalAxis(0));
  }
  return axis;
}

std::vector<gazebo::math::Pose> ArmStructure::GetPoses(std::string name)
{
  std::vector<gazebo::physics::JointPtr> joints = this->chains[name];
  std::vector<gazebo::math::Pose> poses;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    poses.push_back((*it)->GetWorldPose());
  }
  return poses;
}


std::vector<std::vector<double>> ArmStructure::GetRelativeLimits(std::string name, bool fixFingers)
{
  std::vector<gazebo::physics::JointPtr> joints = this->chains[name];
  std::vector<std::vector<double>> limits;
  unsigned int i = 0;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    // Create sub std::vector
    std::vector<double> duo;
    limits.push_back(duo);
    // Get angle and scale it to [-pi,pi]
    double angle = (*it)->GetAngle(0).Radian();
    int scale =  floor(angle/(2*M_PI));
    angle = angle - scale * (2*M_PI);
    if(angle > M_PI){angle = angle-2*M_PI;}
    double angleLowerLimit = (*it)->GetLowerLimit(0).Radian() + this->limitoffset;
    double angleUpperLimit = (*it)->GetUpperLimit(0).Radian() - this->limitoffset;
    // If axis on z, not limit, else (-pi/2,pi/2)
    if( (*it)->GetLocalAxis(0) == gazebo::math::Vector3(0,0,1) )
    {
      limits[i].push_back(std::numeric_limits<double>::lowest());
      limits[i].push_back(std::numeric_limits<double>::max());
    }else{
      limits[i].push_back( -(angle + (-angleLowerLimit)) );
      limits[i].push_back(angleUpperLimit - angle);
    }
    // If we want to fix the fingers
    if(fixFingers)
    {
      for(auto at = this->fingers.begin(); at != this->fingers.end(); at++)
      {
        if( (*it) == (*at) )
        {
          limits[i][0] = 0;
          limits[i][1] = 0;
        }
      }
    }
    i++;
  }
  return limits;
}

std::vector<std::vector<double>> ArmStructure::GetLimits(std::string name)
{
  std::vector<gazebo::physics::JointPtr> joints = this->chains[name];
  std::vector<std::vector<double>> limits;
  unsigned int i = 0;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    // Create sub std::vector
    std::vector<double> duo;
    limits.push_back(duo);
    double angleLowerLimit = (*it)->GetLowerLimit(0).Radian() + this->limitoffset;
    double angleUpperLimit = (*it)->GetUpperLimit(0).Radian() - this->limitoffset;
    // If axis on z, not limit, else (-pi/2,pi/2)
    if( (*it)->GetLocalAxis(0) == gazebo::math::Vector3(0,0,1) )
    {
      limits[i].push_back(std::numeric_limits<double>::lowest());
      limits[i].push_back(std::numeric_limits<double>::max());
    }else{
      limits[i].push_back(angleLowerLimit);
      limits[i].push_back(angleUpperLimit);
    }
    i++;
  }
  return limits;
}

std::vector<gazebo::physics::JointPtr> ArmStructure::GetFingers()
{
  return this->fingers;
}

std::vector<int> ArmStructure::GetFingersSigns()
{
  gazebo::math::Vector3 center(0,0,0);
  for(unsigned int i = 0; i < this->fingers.size(); i++)
  {
    center += this->fingers[i]->GetWorldPose().pos;
  }
  center = center / this->fingers.size();

  std::vector<int> signs;
  for(unsigned int i = 0; i < this->fingers.size(); i++)
  {
    gazebo::math::Vector3 centerDir = center - this->fingers[i]->GetWorldPose().pos;
    gazebo::math::Vector3 jointAxis = this->fingers[i]->GetGlobalAxis(0);
    gazebo::math::Vector3 fingerDir = this->fingers[i]->GetChild()->GetWorldPose().pos - this->fingers[i]->GetWorldPose().pos;
    if( (fingerDir.Cross(jointAxis)).Dot(centerDir) > 0)
    {
      signs.push_back(-1);
    }else{
      signs.push_back(1);
    }
  }
  return signs;
}

std::vector<std::vector<gazebo::physics::JointPtr>> ArmStructure::GetCaptors()
{
  return this->captors;
}

std::vector<gazebo::math::Vector3> ArmStructure::GetCaptorsEndEffectors()
{
  std::vector<gazebo::math::Vector3> endEff;
  for(unsigned int i = 0; i < this->captors.size(); i++)
  {
    gazebo::math::Vector3 vect(0,0,0);
    for(auto it = this->captors[i].begin(); it != this->captors[i].end(); it++)
    {
      vect += (*it)->GetChild()->GetWorldPose().pos;
    }
    vect = vect / this->captors[i].size();
    endEff.push_back(vect);
  }
  return endEff;
}

std::vector<std::vector<unsigned int>> ArmStructure::GetChildren(std::string name)
{
  return this->chainsChildren[name];
}

std::vector<unsigned int> ArmStructure::GetRanks(std::string name)
{
  std::vector<gazebo::physics::JointPtr> joints = this->chains[name];
  std::vector<unsigned int> ranks;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    ranks.push_back(this->linkages[this->map[(*it)->GetName()]].rank);
  }
  return ranks;
}

std::vector<gazebo::math::Vector3> ArmStructure::GetMovedSingleTarget(std::string name, gazebo::math::Vector3 scale)
{
  gazebo::physics::JointPtr joint = this->chains[name].back();
  joint = joint->GetChild()->GetChildJoints().front();
  return std::vector<gazebo::math::Vector3>{joint->GetWorldPose().pos + scale};
}

std::vector<gazebo::math::Vector3> ArmStructure::GetNaturalSingleEndEffectors(std::string name)
{
  gazebo::physics::JointPtr joint = this->chains[name].back();
  std::vector<gazebo::physics::JointPtr> joints = joint->GetChild()->GetChildJoints();
  gazebo::math::Vector3 toReturn(0,0,0) ;
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    toReturn += (*it)->GetWorldPose().pos;
  }
  toReturn = toReturn / joints.size();
  return std::vector<gazebo::math::Vector3>{toReturn};
}
