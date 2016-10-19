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
  // Load external parameters
  std::map<std::string, double> dictionary = loadParam("Models/control.dic");
  // Init Stats
  this->statistics.Init(dictionary["$iter_simu"], dictionary["$wait_iter"]);
  this->statistics.CreateFile("ik");
  this->statistics.WriteInFile("ik", "TARGET;INITIAL_POSITION;IK_SOLUTION;NB_ITERATIONS\n");
  // Store the pointer to the model
  this->model = model;
  // Store the world
  this->world = this->model->GetWorld();
  // Init the structure of the arm
  this->armStructure.Init(this->model->GetJoint("shoulder_#joint"), dictionary["$limit_offset"]);
  // Create a IK chain
  this->armStructure.CreateChain("IK1",0,2);
  this->armStructure.CreateChain("IK2",0,6);
  this->armStructure.CreateChain("IK2.1",0,5);
  // Init PIDs
  this->initPids(dictionary["$p"], dictionary["$i"], dictionary["$d"], dictionary["$imax"], dictionary["$imin"], dictionary["$max"], dictionary["$min"]);
  // Init the joint controller
  this->jointsController.Init(this->model->GetJointController(),&(this->armStructure),dictionary);
  // Init the gripper controller
  this->gripperController.Init(this->model->GetJointController(),&(this->armStructure),dictionary);
  // Init Contacts Manager
  this->contactsManager.Init(this->world->GetPhysicsEngine()->GetContactManager(),this->world->GetName(), dictionary["$update_freq"]);
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
  this->state = 0;
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
      // Move to random init place
      case 0:
        if(!this->init)
        {
          this->statistics.updateSimu();
          // Generate random target
          std::vector<gazebo::math::Vector3> targets;
          // Arm_base is 1.5 meter high and max 1.05 meters from base to hand joint
          targets.push_back( this->generateRandomSphereTarget(gazebo::math::Vector3(0,0,1.5),1.05) );
          std::cout << "Random sphere: "  << targets[0] << std::endl;
          std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK2.1");
          std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK2.1", false);
          std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK2.1");
          std::vector<double> angles = this->inverseKinematics["IK2.1"].CCD(poses,jointsLimits, endEffectors, targets);
          this->jointsController.SetRelativeTargets("IK2.1",angles);
          this->init = true;
        }

        //Nothing
        
        if(this->jointsController.AreTargetsReached("IK2.1", false))
        {
          this->state = 1;
          this->init = false;
        }
        break;

      // Go to random position with y=0
      case 1:
        if(!this->init)
        {
          // Generate random target
          std::vector<gazebo::math::Vector3> targets;
          // Arm_base is 1.5 meter high and max 1.25 meters from base to hand joint
          targets.push_back( this->generateRandomDiskTarget(gazebo::math::Vector3(0,0,1.5),1.25) );
          std::cout << "Random disk: "  << targets[0] << std::endl;
          this->statistics.WriteInFile("ik", targets[0]);this->statistics.WriteInFile("ik", ";");
          std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK2.1");
          std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK2.1", false);
          std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK2.1");
          this->statistics.WriteInFile("ik", endEffectors[0]);this->statistics.WriteInFile("ik", ";");
          this->statistics.SetFileFlag(true);
          std::vector<double> angles = this->inverseKinematics["IK2.1"].CCD(poses,jointsLimits, endEffectors, targets);
          this->statistics.SetFileFlag(false);
          this->jointsController.SetRelativeTargets("IK2.1",angles);
          this->init = true;
        }

        //Nothing

        if(this->jointsController.AreTargetsReached("IK2", true))
        {
          this->state = 2;
          this->init = false;
        }
        break;
        
      // Go to random position with y=0
      case 2:
        if(!this->init)
        {
          this->statistics.SetTimer("wait",2);
          this->init = true;
        }

        //Nothing
        if(this->statistics.IsTimerOff("wait"))
        {
          this->state = 0;
          this->init = false;
        }
        break;
      // Close hand
     }
  }else if(this->statistics.isSimuOver() == true){
    this->statistics.CloseFiles();
  }
  this->UpdateJoints();
}

void ArmController::UpdateJoints()
{
  if( this->contactsManager.checkCollisions() && this->contact )
  {
    this->state = 0;
    this->init = false;
  }else{
    this->model->GetJointController()->Update();
  }
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
