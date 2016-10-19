#include "InverseKinematics.hh"

InverseKinematics::InverseKinematics()
{

}

InverseKinematics::~InverseKinematics()
{

}

void InverseKinematics::Init(const std::vector<gazebo::math::Vector3>& axis, const std::vector<std::vector<unsigned int>>& children, const std::vector<unsigned int>& endEffJoints, std::map<std::string, double>& dictionary)
{
  if(axis.size() != children.size() )
  {
    std::cout << "[Error] axis and subchains sizes do not match in InverseKinematics::Init" << std::endl;
    return;
  }
  this->axis = axis;
  this->children = children;
  this->jointsNumber = axis.size();
  this->errormax = dictionary["$error_max_ik"];
  this->converror = dictionary["$convergence_error_ik"];
  this->itermax = dictionary["$iter_max_ik"];
  this->endEffJoints = endEffJoints;
  this->targetsNumber = endEffJoints.size();
  for(unsigned int i=0; i<this->targetsNumber; i++)
  {
    this->endEffectors.push_back(gazebo::math::Vector3(0,0,0));
    this->targets.push_back(gazebo::math::Vector3(0,0,0));
  }
  for(unsigned int i=0; i<this->jointsNumber; i++)
  {
    this->finalAngles.push_back(0);
  }
  this->jointsToEndEff = std::vector<std::vector<unsigned int>>(this->jointsNumber);
  this->initJointsToEndEff(0);

  // For information
  std::cout << std::endl << std::endl << "[INVERSE KINEMATICS :: INIT]" << std::endl;
  for(unsigned int i=0; i<this->jointsNumber; i++)
  {
    std::cout << "Joint " << i << ": " << std::endl;
    std::cout << "-> axis: " << this->axis[i] << std::endl;
    std::cout << "-> children: ";
    for(auto const& k: this->children[i])
    {
      std::cout << " " << k;
    }
    std::cout << std::endl;
    std::cout << "-> connected end effectors: ";
    for(auto const& k: this->jointsToEndEff[i])
    {
      std::cout << " " << k << "/" << this->endEffJoints[k];
    }
    std::cout << std::endl;
  }
}

void InverseKinematics::initJointsToEndEff(unsigned int i)
{
  // Initialize jointsToEndEff[i] empty
  this->jointsToEndEff[i] = std::vector<unsigned int>(0);

  // Wait for subchildren to be treated-
  for(unsigned int j = 0; j < this->children[i].size(); j++)
  {
    unsigned int k = this->children[i][j];
    this->initJointsToEndEff(k);
    this->jointsToEndEff[i] = this->uniqueConcat(this->jointsToEndEff[i], this->jointsToEndEff[k]);
  }
  // If defined as a end effector, we add it to the list
  for(unsigned int j = 0; j < this->targetsNumber; j++)
  {
    if(this->endEffJoints[j] == i)
    {
      this->jointsToEndEff[i].push_back(j);
    }
  }
}

bool InverseKinematics::updateKinematicParameters(const std::vector<gazebo::math::Pose>& poses, const std::vector<std::vector<double>>& jointsLimits, const std::vector<gazebo::math::Vector3>& endEffectors, const std::vector<gazebo::math::Vector3>& targets)
{
  if(poses.size() != this->jointsNumber || jointsLimits.size() != this->jointsNumber || endEffectors.size() != this->targetsNumber || targets.size() != this->targetsNumber)
  {
    std::cout << "[Error] sizes do not match in InverseKinematics::updateKinematicParameters" << std::endl;
    return false;
  }
  this->targets = targets;
  this->poses = poses;
  this->jointsLimits = jointsLimits;
  //Compute where the endEffectors are
  for(unsigned int i = 0; i < this->targetsNumber; i++)
  {
    this->endEffectors[i] = endEffectors[i];
    //std::cout << "EndEffetor " << i << ": " << this->endEffectors[i] << "    offset:" << endEffectorOffsets[i] << std::endl;
    //std::cout << "Target " << i << ": " << this->targets[i] << std::endl;
  }
  return true;
}



std::vector<double> InverseKinematics::CCD(const std::vector<gazebo::math::Pose>& poses, const std::vector<std::vector<double>>& jointsLimits, const std::vector<gazebo::math::Vector3>& endEffectors, const std::vector<gazebo::math::Vector3>& targets)
{
  // Set kinematic values from gazebo simulation
  this->updateKinematicParameters(poses,  jointsLimits, endEffectors, targets);
  unsigned int iter = 0;
  double previous_error_mean = std::numeric_limits<double>::max();
  double previous_error_max = std::numeric_limits<double>::max();
  this->resetFinalAngles();

  while( iter < 20 || ( ( this->getError("max",this->endEffectors) > this->errormax ) && ( iter < this->itermax )  && ( previous_error_mean > (this->getError("mean",this->endEffectors) + this->converror) || previous_error_mean < this->getError("mean",this->endEffectors) || previous_error_max > (this->getError("max",this->endEffectors) + this->converror) || previous_error_max > this->getError("max",this->endEffectors)) ) )
  {
    previous_error_mean = this->getError("mean",this->endEffectors);
    previous_error_max = this->getError("max",this->endEffectors);
    // Optimize end eE after each other
    for(unsigned int j = 0; j<this->targetsNumber; j++)
    {
      for(int i = (this->jointsNumber-1); i>=0; i--)
      {
        double angle = 0;
        for(auto const& chain: this->jointsToEndEff[i])
        {
          if(j == chain)
          {
            angle += this->computeDescent(i, this->endEffectors[chain], this->targets[chain]);
          }
        }
        angle = angle / this->jointsToEndEff[i].size();
        angle = this->regularizeAngle(i, angle);
        this->forwardKinematic(i, angle);
      }
    }
    iter++;
    //std::cout << "Iter: " << iter << "    Error mean:" << this->getError("mean",this->endEffectors) << "    Error max:" << this->getError("max",this->endEffectors) << std::endl;
  }
  //std::cout << "Iter: " << iter << "    Error mean:" << this->getError("mean",this->endEffectors) << "    Error max:" << this->getError("max",this->endEffectors) << std::endl;
  this->finalAngles = this->normalizeAngles(this->finalAngles);
  return this->finalAngles;
}

double InverseKinematics::computeDescent(unsigned int i, const gazebo::math::Vector3& endEffector, const gazebo::math::Vector3& target)
{
  // Optimize the angle for the connected end effector
  // Some definitions
  gazebo::math::Vector3 o = endEffector; // actual position of endEffector on the rotation circle
  gazebo::math::Vector3 j = this->poses[i].pos; // position of the joint
  gazebo::math::Vector3 jo = o - j; // vector from j to o
  gazebo::math::Vector3 t = target; // position of the target
  gazebo::math::Vector3 jc_vec = this->poses[i].rot.RotateVector(this->axis[i]); //direction to center of circle of rotation of end effector ( sign is missing )
  // Get the center of the circle decribed by the end effector under rotation of the joint
  double posign = jc_vec.Dot(jo); // get the sens of jc
  int signfactor = ( (posign >= 0) - (posign < 0) ); // get the sign factor for jc
  gazebo::math::Vector3 jc_dir = jc_vec * signfactor; // vector directing the rotation of the endEffector around the axis of the joint
  double ojc =  this->get3DAngle(jo, jc_dir); // get angle  ojc
  gazebo::math::Vector3 jc = jc_dir * jo.GetLength() * std::cos(ojc); // find vector jc
  gazebo::math::Vector3 c = j + jc; // compute the position c of the centre or rotation of oc
  // Find the projection of target on the circle's plan
  double d = (-jc_dir).Dot(c); // Get the d parameter of the plan equation
  double tp_dist = std::fabs(jc_dir.Dot(t)+d)/jc_dir.GetLength(); // get the dist of projection
  gazebo::math::Vector3 tc = c - t; //Get the sens of projection
  posign = jc_dir.Dot(tc); // idem
  signfactor = ( (posign >= 0) - (posign < 0) ); //idem
  gazebo::math::Vector3 p = t + signfactor * tp_dist * jc_dir; // get the actual projection
  // Find projection of p on the circle
  gazebo::math::Vector3 cp = (p-c); // compute cp
  gazebo::math::Vector3 co = (o-c); // compute co
  gazebo::math::Vector3 cn = cp*co.GetLength()/cp.GetLength(); // compute cn
  gazebo::math::Vector3 n = c + cn;

  // Update joint angle
  double angle = this->get3DAngle(co,cn); // get absolute angle
  gazebo::math::Vector3 signvec = co.Cross(cn); // find in which direction it turns
  double angle_sign = signvec.Dot(jc_vec); // get the sign of the angle with respect to the sens of the gazebo's axis of rotation
  angle_sign = ( (angle_sign >= 0) - (angle_sign < 0) ); // idem
  angle = angle_sign * angle; // multiply by sign factor
  //std::cout << angle << " " << this->jointsLimits[i][0] << " " << this->jointsLimits[i][1] << std::endl;
  // Joint limit
  // this->angles[i] is in rnge [-pi, pi]
  // this->jointsLimits[i] are in range [-pi, pi]

  return angle;
}

double InverseKinematics::regularizeAngle(unsigned int i, double angle)
{
  double lmin = this->jointsLimits[i][0]; // lower limit [-2p - 0] if not infinity
  double lmax = this->jointsLimits[i][1]; // upper limit  [0 - 2pi] if not infinity
  double med = (lmin + lmax)/2; // median of limits [-pi , pi]
  double dist = M_PI - (lmax - med); // absolute distance to go after one limits before falling back in the ohter
  if(angle <= lmin) // if lower
  {
    //std::cout << "Angle to far: " << i << " " << angle << " " << lmin << " " << lmax << " " << dist << std::endl;
    if(angle >= (lmin - dist) )
    {
      angle = lmin;
    }else{
      angle = lmax;
    }
    return angle;
  }

  if(angle > lmax)
  {
    if(angle <= (lmax + dist) )
    {
      angle = lmax;
    }else{
      angle = lmin;
    }
    return angle;
  }
  return angle;
}

std::vector<double> InverseKinematics::normalizeAngles(const std::vector<double>& angles)
{
  std::vector<double> newAngles = angles;
  for(unsigned int i = 0; i<angles.size(); i++)
  {
    // Get angle and scale it to [-pi,pi]
    double angle = angles[i];
    int scale =  floor(angle/(2*M_PI));
    angle = angle - scale * (2*M_PI);
    if(angle > M_PI){angle = angle-2*M_PI;}
    newAngles[i] = angle;
  }
  return angles;
}

// FOR ONE JOINT, APPLY TO ALL SUBCHILDREN
// Forward kinematic
// Update poses, finalAngles and jointsLimits with respect to angles;
void InverseKinematics::forwardKinematic(unsigned int i, double angle)
{
  //Update axis and position
  // Compute quaternion for relative rotation of joint i
  gazebo::math::Vector3 jc_vec = this->poses[i].rot.RotateVector(this->axis[i]);
  gazebo::math::Quaternion quater(jc_vec,angle);
  this->forwardPass(i, i,quater);
  // Update jointsLimits
  this->jointsLimits[i][0] = this->jointsLimits[i][0] - angle;
  this->jointsLimits[i][1] = this->jointsLimits[i][1] - angle;
  // Update final angles
  this->finalAngles[i] += angle;
  // Update end Effectors
  this->endEffectors = this->computeEndEffectors(i, angle);
}

void InverseKinematics::forwardPass(unsigned int i, unsigned int j, const gazebo::math::Quaternion& quater)
{
  // Update pos
  gazebo::math::Vector3 ij = this->poses[j].pos - this->poses[i].pos;
  ij = quater.RotateVector(ij);
  this->poses[j].pos = this->poses[i].pos + ij;
  //Update rot
  this->poses[j].rot = quater * this->poses[j].rot;
  // Update children
  for(unsigned int k = 0; k < this->children[j].size(); k++)
  {
    this->forwardPass(i, this->children[j][k],quater);
  }

}

// Compute end effectors given one joint rotation
std::vector<gazebo::math::Vector3> InverseKinematics::computeEndEffectors(unsigned int i,double angle)
{
  // Then compute new end effectors positions
  std::vector<gazebo::math::Vector3> endEffectors;
  gazebo::math::Vector3 jointPos = this->poses[i].pos;
  gazebo::math::Vector3 jointAxi = this->poses[i].rot.RotateVector(this->axis[i]);
  for(unsigned int j = 0; j < this->targetsNumber; j++)
  {
    endEffectors.push_back(this->endEffectors[j]);
    for(auto const& chain: this->jointsToEndEff[i])
    {
      if(j == chain)
      {
        gazebo::math::Vector3 vec = this->endEffectors[chain] - jointPos;
        gazebo::math::Quaternion quater(jointAxi,angle);
        endEffectors.back() = jointPos + quater.RotateVector(vec);
      }
    }
  }
  return endEffectors;
}


std::vector<unsigned int> InverseKinematics::uniqueConcat(const std::vector<unsigned int>& A, const std::vector<unsigned int>& B)
{
  std::vector<unsigned int> copyA = A;
  std::vector<unsigned int> copyB = B;
  if(copyA.size() == 0)
  {
    return copyB;
  }

  for(auto const& it: copyA)
  {
    unsigned int i = 0;
    while( i < copyB.size() )
    if(it == copyB[i])
    {
        copyB.erase( copyB.begin() + i );
    } else {
        i++;
    }
  }

  copyA.insert( copyA.end(), copyB.begin(), copyB.end() );
  return copyA;
}

// Set all angles to zero
void InverseKinematics::resetFinalAngles()
{
  for(unsigned int i=0; i<this->jointsNumber; i++)
  {
    this->finalAngles[i] = 0;
  }
}
// Get absolute angle in 3D between 0 and pi
double InverseKinematics::get3DAngle(const gazebo::math::Vector3& v1, const gazebo::math::Vector3& v2)
{
  if(v1.GetLength() == 0 || v2.GetLength() == 0)
  {
    return 0;
  }
  double cos = v1.Dot(v2) / (v1.GetLength() * v2.GetLength());
  if(cos > 1){cos = 1;}
  if(cos < -1){cos = -1;}
  return std::acos(cos);
}

double InverseKinematics::getError(const std::string& errortype, const std::vector<gazebo::math::Vector3>& endEffectors)
{
  if(errortype == "mean")
  {
    return this->getMeanError(endEffectors);
  }else if(errortype == "max")
  {
    return this->getMaxError(endEffectors);
  }else{
    return -1;
  }
}

double InverseKinematics::getMaxError(const std::vector<gazebo::math::Vector3>& endEffectors)
{
  double error = 0;
  for(unsigned int i=0; i<this->targetsNumber; i++)
  {
    if(error < fabs((this->targets[i]-endEffectors[i]).GetLength()))
    {
      error = fabs((this->targets[i]-endEffectors[i]).GetLength());
    }
  }
  return error;
}

double InverseKinematics::getMeanError(const std::vector<gazebo::math::Vector3>& endEffectors)
{
  double error = 0;
  for(unsigned int i=0; i<this->targetsNumber; i++)
  {
    error += fabs((this->targets[i]-endEffectors[i]).GetLength());
  }
  error = error / (double)this->targetsNumber;
  return error;
}
