#include "WorldView.hh"



WorldView::WorldView()
{

}

WorldView::~WorldView()
{

}

void WorldView::Init(gazebo::physics::WorldPtr world)
{
  this->world = world;
}

void WorldView::SetTimer(const std::string& name, float seconds)
{
  this->timers[name] = this->world->GetSimTime() + gazebo::common::Time(seconds,0);
  /*std::cout << "Setting timer to: " << this->timers[name] << "  with current SimTime at: " << this->world->GetSimTime() << std::endl;*/
}

unsigned int WorldView::IsTimerOff(const std::string& name)
{
  std::map<std::string,gazebo::common::Time>::iterator it;
  it=this->timers.find(name);
  if ( it == this->timers.end() ) {
    return 2;
  } else {
    if(this->world->GetSimTime() > this->timers[name])
    {
      this->timers.erase(it);
      return 1;
    }
    return 0;
  }
  return 2;
}

double WorldView::GetBoxSize(const std::string& objname)
{
  return this->world->GetModel(objname)->GetCollisionBoundingBox().GetZLength();
}

std::string WorldView::GetBoxNameInHand(const std::vector<gazebo::math::Vector3>& endE)
{
  std::vector<gazebo::physics::ModelPtr> models = this->world->GetModels();
  double result;
  unsigned int k = 0;
  double min = std::numeric_limits<double>::max();
  for(unsigned int i=0; i < models.size(); i++)
  {
    if(models[i]->GetName() != "arm")
    {
      result = 0;
      for(unsigned int j=0; j < endE.size(); j++)
      {
        result += (models[i]->GetWorldPose().pos - endE[j]).GetLength();
      }
      if(result < min)
      {
        min = result;
        k = i;
      }
    }
  }
  return models[k]->GetName();
}

std::vector<gazebo::math::Vector3> WorldView::GetBoxGrabbingPoints(const std::string& objname, double offset)
{
  // TO REMOVE
  /*if(this->world->GetModel(objname)->GetWorldPose().pos.z < 1)
  {
    double xmin = -0.6;
    double xmax = 0.6;
    double ymin = 0.5;
    double ymax = 1;
    double qmin = 0;
    double qmax = 2*M_PI;
    double x = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
    double y = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
    double q = (qmax - qmin) * ( (double)rand() / (double)RAND_MAX ) + qmin;
    this->world->GetModel(objname)->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(x,y,1.04),gazebo::math::Quaternion(0,0,q) ),std::string("boxlink"));
  }*/

  gazebo::physics::ModelPtr obj = this->world->GetModel(objname);
  gazebo::math::Pose objpose = obj->GetWorldPose();
  gazebo::math::Vector3 objpos = objpose.pos; // absolute centroid position of obj
  gazebo::math::Quaternion objrot = objpose.rot; // rot of the obj

  std::vector<std::vector<gazebo::math::Vector3>> points;
  std::vector<gazebo::math::Vector3> axis = std::vector<gazebo::math::Vector3>{gazebo::math::Vector3(1,0,0), gazebo::math::Vector3(0,1,0), gazebo::math::Vector3(0,0,1)};
  std::vector<double> scores;
  // Compute possible grabbing points
  for(unsigned int i=0; i < 3; i++)
  {
    std::vector<gazebo::math::Vector3> apo;
    apo.push_back(  objrot.RotateVector( axis[i] * offset ) );
    apo.push_back( objrot.RotateVector( -axis[i] * offset ) );
    points.push_back(apo);
    gazebo::math::Vector3 diff = points[i][1]-points[i][0];
    if( std::fabs(diff.z) < ( std::fabs(diff.x) + std::fabs(diff.y) ) )
    {
      scores.push_back( std::fabs(diff.Dot(objpos)) );
    }else{
      scores.push_back(std::numeric_limits<double>::max());
    }
  }

  // Find the best points-pair
  double min = scores[0];
  unsigned int k = 0;
  for(unsigned int i=1; i < scores.size(); i++)
  {
    if(scores[i] < min)
    {
      min = scores[i];
      k = i;
    }
  }

  // Return found points
  std::vector<gazebo::math::Vector3> grabpoints;
  grabpoints.push_back(objpos + points[k][0]);
  grabpoints.push_back(objpos + points[k][1]);
  return grabpoints;

}

std::vector<gazebo::math::Vector3> WorldView::GetBoundingBoxGrabbingPoints(const std::string& objname, double offset)
{
  gazebo::physics::ModelPtr obj = this->world->GetModel(objname);
  gazebo::math::Pose objpose = obj->GetWorldPose();
  gazebo::math::Vector3 objpos = objpose.pos; // absolute centroid position of obj
  gazebo::math::Box box = obj->GetCollisionBoundingBox(); // Bounding box of the object
  gazebo::math::Vector3 mincorner = box.min; // Minimum point of the bounding box

  // Get orientation to arm base in 2D (x,y, 0)
  gazebo::math::Vector3 orientation = mincorner;
  orientation.z = 0;
  // Get axis of direction from arm to box
  gazebo::math::Vector3 axis = objpos;
  axis.z = 0;

  double sign = (orientation.Cross(axis)).Dot(gazebo::math::Vector3(0,0,1));
  sign = sign / std::fabs(sign);

  // Get two rotations
  double len = box.GetYLength(); // Not sure of this, can't find proper doc
  gazebo::math::Vector3 quarteraxis = gazebo::math::Vector3(0,0,1);
  gazebo::math::Vector3 vec = orientation - axis;  // From centroid to min corner, in 2D
  vec = vec * len / (2*vec.GetLength()); // We scale it to the len of the box
  gazebo::math::Quaternion rota1 =  gazebo::math::Quaternion(quarteraxis, sign * M_PI/4);
  gazebo::math::Quaternion rota2 =  gazebo::math::Quaternion(quarteraxis, -1 * sign * 3*M_PI/4);

  // Relative poses
  gazebo::math::Vector3 pos1 = rota1.RotateVector(vec);
  gazebo::math::Vector3 pos2 = rota2.RotateVector(vec);

  // Adding offset
  pos1 = pos1 + pos1 * offset / pos1.GetLength();
  pos2 = pos2 + pos2 * offset / pos2.GetLength();

  // Generating absolute pos
  std::vector<gazebo::math::Vector3> points;
  points.push_back(objpos + pos1);
  points.push_back(objpos + pos2);

  return points;
}
