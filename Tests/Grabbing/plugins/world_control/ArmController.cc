#include "ArmController.hh"

ArmController::ArmController()
{
  std::srand((unsigned)std::time(NULL));
  this->reflex = true;
}

ArmController::~ArmController()
{

}

void ArmController::Init(gazebo::physics::ModelPtr model)
{
  // Store the pointer to the model
  this->model = model;
  // Store the world
  this->world = this->model->GetWorld();
  // Load external parameters
  std::map<std::string, double> dictionary = loadParam("Models/control.dic");
  this->reflex_xyz = dictionary["$reflex_xyz"];
  this->reflex_z = dictionary["$reflex_z"];
  // Init Stats
  this->statistics.Init(dictionary["$iter_simu"], dictionary["$wait_iter"]);
  this->statistics.CreateFile("grabbox");
  this->statistics.WriteInFile("grabbox", "SPECIAL\n");
  // Init the structure of the arm
  this->armStructure.Init(this->model->GetJoint("shoulder_#joint"), dictionary["$limit_offset"]);
  // Create a IK chain
  this->armStructure.CreateChain("IK1",0,2);
  this->armStructure.CreateChain("IK2",0,6);
  this->armStructure.CreateChain("IK2.1",0,5);
  // Init PIDs
  this->initPids(dictionary["$p"], dictionary["$i"], dictionary["$d"], dictionary["$imax"], dictionary["$imin"], dictionary["$max"], dictionary["$min"]);
  // Init the joint controller
  this->jointsController.Init(this->model->GetJointController(),&(this->armStructure),this->world, dictionary);
  // Init the gripper controller
  this->gripperController.Init(this->model->GetJointController(),&(this->armStructure),dictionary, &(this->statistics));
  // Init Contacts Manager
  this->contactsManager.Init(this->world->GetPhysicsEngine()->GetContactManager(),this->world->GetName(), dictionary["$update_freq"],&(this->statistics));
  // Init kinematic class (axis of rotation of the joints and control prameters)
  this->inverseKinematics["IK1"] = InverseKinematics();
  this->inverseKinematics["IK1"].Init(this->armStructure.GetAxis("IK1"), this->armStructure.GetChildren("IK1"), std::vector<unsigned int>{2}, dictionary, &(this->statistics));
  this->inverseKinematics["IK2"] = InverseKinematics();
  this->inverseKinematics["IK2"].Init(this->armStructure.GetAxis("IK2"), this->armStructure.GetChildren("IK2"), std::vector<unsigned int>{6,7}, dictionary, &(this->statistics));
  this->inverseKinematics["IK2.1"] = InverseKinematics();
  this->inverseKinematics["IK2.1"].Init(this->armStructure.GetAxis("IK2.1"), this->armStructure.GetChildren("IK2.1"), std::vector<unsigned int>{5}, dictionary, &(this->statistics));
  // Init worldview
  this->worldview.Init(this->world);
  // Init state
  this->state = 1;
  this->init = false;
}

//
void ArmController::Update()
{
  // is simulation stil running
  if(this->statistics.isSimuReady() == true && this->statistics.isSimuOver() == false)
  {
    switch(this->state)
    {
      //this->statistics.WriteInFile("ik", endEffectors[0]);this->statistics.WriteInFile("ik", ";");
      //this->statistics.SetFileFlag(true);
      
      // If contact, we start again
      case 0:
        if(!this->init)
        {
          this->contact = false; 
          std::vector<gazebo::math::Vector3> targets = this->armStructure.GetMovedSingleTarget("IK1", gazebo::math::Vector3( (double)std::rand() * this->reflex_xyz / (RAND_MAX),(double)std::rand() * this->reflex_xyz / (RAND_MAX),(double)std::rand() * this->reflex_xyz / (RAND_MAX) + this->reflex_z));
          std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1");
          std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1", false);
          std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1");
          std::vector<double> angles = this->inverseKinematics["IK1"].CCD(poses,jointsLimits, endEffectors, targets);
          this->jointsController.SetRelativeTargets("IK1",angles);
          this->init = true;
          std::cout << "Target set for reflex: " << targets.front() << std::endl;
          this->statistics.WriteInFile("grabbox", "reflex;");this->statistics.WriteInFile("grabbox",targets.front());this->statistics.WriteInFile("grabbox", "\n");
          this->worldview.SetTimer("reflex",1);
        }

        if(this->worldview.IsTimerOff("reflex"))
        {
          this->contact = true;
        }

        //Nothing
        if(this->jointsController.AreTargetsReached("IK1", false))
        {
          this->state = 4;
          this->init = false;
          this->contact = true; 
        }
        break;

      // Init for simu
      case 1:
        {
          double xmin = -0.6;
          double xmax = 0.6;
          double ymin = 0.5;
          double ymax = 1;
          double qmin = 0;
          double qmax = 2*M_PI;

          double xred = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
          double yred = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
          double qred = (qmax - qmin) * ( (double)rand() / (double)RAND_MAX ) + qmin;
          this->world->GetModel("boxred")->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(xred,yred,1.04),gazebo::math::Quaternion(0,0,qred) ),std::string("boxlink"));
          this->statistics.WriteInFile("grabbox", "newiter\n");
          this->statistics.WriteInFile("grabbox", "boxpos;");this->statistics.WriteInFile("grabbox",gazebo::math::Vector3(xred,yred,1.04));this->statistics.WriteInFile("grabbox", "\n");
          this->statistics.WriteInFile("grabbox", "boxquater;");this->statistics.WriteInFile("grabbox",gazebo::math::Quaternion(0,0,qred));this->statistics.WriteInFile("grabbox", "\n");
          this->state = 2;
        }
        break;
        
        case 2:
        if(!this->init)
        {
          this->worldview.SetTimer("waitforbox",1);
          this->init = true;
        }

        //Nothing
        if(this->worldview.IsTimerOff("waitforbox"))
        {
          this->state = 4;
          this->init = false;
        }
        break;
      // Place hand around redbox
      case 4:
        if(!this->init)
        {
          this->contact = true;
          std::vector<gazebo::math::Vector3> targets;
          targets = this->worldview.GetBoxGrabbingPoints("boxred", 0.16);
          std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK2");
          std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK2",false);
          std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetCaptorsEndEffectors();
          this->statistics.SetFileFlag(true);
          std::vector<double> angles = this->inverseKinematics["IK2"].CCD(poses,jointsLimits, endEffectors, targets);
          this->statistics.SetFileFlag(false);
          this->jointsController.SetRelativeTargets("IK2",angles);
          this->init = true;
          std::cout << "Target set to red box:  " << targets[0] << " - " << targets[1] << std::endl;
          this->statistics.WriteInFile("grabbox", "target1;");this->statistics.WriteInFile("grabbox",targets[0]);this->statistics.WriteInFile("grabbox", "\n");
          this->statistics.WriteInFile("grabbox", "target2;");this->statistics.WriteInFile("grabbox",targets[1]);this->statistics.WriteInFile("grabbox", "\n");
        }

        //Nothing

        if(this->jointsController.AreTargetsReached("IK2", true))
        {
          double angle1 = this->model->GetJoint("left#finger_#joint")->GetAngle(0).Radian();
          double angle2 = this->model->GetJoint("right#finger_#joint")->GetAngle(0).Radian();
          this->statistics.WriteInFile("grabbox", "fingers;");this->statistics.WriteInFile("grabbox",angle1);this->statistics.WriteInFile("grabbox", ";");
          this->statistics.WriteInFile("grabbox",angle2);this->statistics.WriteInFile("grabbox", "\n");
          this->state = 5;
          this->init = false;
        }
        break;
        
      // Close hand
      case 5:
      if(!this->init)
      {
        this->contact = false;
        this->gripperController.Close();
        this->init = true;
        this->statistics.SetFileFlag(true);
      }

      this->gripperController.Update();

      if(this->gripperController.IsClosed())
      {
        this->statistics.SetFileFlag(false);
        this->state = 6;
        this->init = false;
      }
      break;

      //  Get some hight
      case 6:
        if(!this->init)
        {
          this->contact = false;
          std::vector<gazebo::math::Vector3> targets = std::vector<gazebo::math::Vector3>{gazebo::math::Vector3(0,0,2.55)};
          std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK2.1");
          std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK2.1", false);
          std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK2.1");
          std::vector<double> angles = this->inverseKinematics["IK2.1"].CCD(poses,jointsLimits, endEffectors, targets);
          this->jointsController.SetRelativeTargets("IK2.1",angles);
          this->init = true;
          std::cout << "Get some hight: " << gazebo::math::Vector3(0,0,2.55) << std::endl;
        }
        
        //Nothing

        if(this->jointsController.AreTargetsReached("IK1", false))
        {
          this->state = 7;
          this->init = false;
        }
        break;
        
      case 7:
        if(!this->init)
        {
          this->worldview.SetTimer("wait",2);
          this->init = true;
        }

        //Nothing
        if(this->worldview.IsTimerOff("wait"))
        {
          this->statistics.WriteInFile("grabbox", "boxfinalpos;");this->statistics.WriteInFile("grabbox",this->world->GetModel("boxred")->GetWorldPose().pos);this->statistics.WriteInFile("grabbox", "\n");
          this->state = 1;
          this->init = false;
          this->statistics.updateSimu();
        }
        break;
     }
  }else if(this->statistics.isSimuOver() == true){
    this->statistics.CloseFiles();
  }
  this->UpdateJoints();
}

void ArmController::UpdateJoints()
{
  if(this->contact)
  {
    this->statistics.SetFileFlag(true);
  }
  if( this->contactsManager.checkCollisions() && this->contact )
  {
    this->state = 0;
    this->init = false;
    this->jointsController.ReflexToTouch();
    this->contact = false;
  }else{
    this->model->GetJointController()->Update();
  }
  this->statistics.SetFileFlag(false);
}

gazebo::math::Vector3 ArmController::generateRandomDiskTarget(gazebo::math::Vector3 c,double rmax)
{
  double r = ((double) std::rand() / (RAND_MAX)) * rmax;
  double phi = ((double) std::rand() / (RAND_MAX)) * 2*M_PI;
  double x = r * std::sin(phi);
  double z = r * std::cos(phi);
  return gazebo::math::Vector3(x,0,z)+c;
}

gazebo::math::Vector3 ArmController::generateRandomSphereTarget(gazebo::math::Vector3 c,double rmax)
{
  double r = ((double) std::rand() / (RAND_MAX)) * rmax;
  double phi = ((double) std::rand() / (RAND_MAX)) * 2*M_PI;
  double theta = ((double) std::rand() / (RAND_MAX)) * M_PI;
  double x = r * std::sin(phi) * std::cos(theta);
  double y = r * std::sin(phi) * std::sin(theta);
  double z = r * std::cos(phi);
  return gazebo::math::Vector3(x,y,z)+c;
}

void ArmController::initPids(double p, double i, double d, double imax, double imin, double cmdMax, double cmdMin)
{
  // For each one of these joints
  std::vector<gazebo::physics::JointPtr> joints = this->armStructure.GetAllJoints();
  for(auto it = joints.begin(); it != joints.end(); it++)
  {
    gazebo::common::PID PID = gazebo::common::PID(p,i,d,imax,imin,cmdMax,cmdMin);
    this->model->GetJointController()->SetPositionPID((*it)->GetScopedName(),PID);
  }
}
