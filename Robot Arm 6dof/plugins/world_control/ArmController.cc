#include "ArmController.hh"

ArmController::ArmController()
{
  std::srand((unsigned)std::time(NULL));
  this->reflex = false;
}

ArmController::~ArmController()
{

}

void ArmController::Init(gazebo::physics::ModelPtr model)
{
  // Load external parameters
  this->graph = std::vector<std::vector<unsigned int>>{std::vector<unsigned int>{0,0}};
  this->param = std::vector<std::vector<std::string>>{};
  this->dictionary = loadParam("Models/control.dic");
  this->dictionary["$reflex_xyz"];
  this->dictionary["$reflex_z"];
  // Store the pointer to the model
  this->model = model;
  // Store the world
  this->world = this->model->GetWorld();
  // Init the structure of the arm
  this->armStructure.Init(this->model->GetJoint("shoulder_#joint"), this->dictionary["$limit_offset"]);
  // Create a IK chain
  this->armStructure.CreateChain("IK1",0,1);
  this->armStructure.CreateChain("IK1.2",0,2);
  this->armStructure.CreateChain("IK2",0,6);
  this->armStructure.CreateChain("IK2.1",0,5);
  // Init the joint controller
  this->jointsController.Init(this->model->GetJointController(),&(this->armStructure),this->world, this->dictionary);
  // Init the gripper controller
  this->gripperController.Init(this->model->GetJointController(),&(this->armStructure),this->world,this->dictionary);
  // Init Contacts Manager
  this->contactsManager.Init(this->world->GetPhysicsEngine()->GetContactManager(),this->world->GetName(), this->dictionary["$update_freq"]);
  // Init kinematic class (axis of rotation of the joints and control prameters)
  this->inverseKinematics["IK1"] = InverseKinematics();
  this->inverseKinematics["IK1"].Init(this->armStructure.GetAxis("IK1"), this->armStructure.GetChildren("IK1"), std::vector<unsigned int>{1}, this->dictionary);
  this->inverseKinematics["IK1.2"] = InverseKinematics();
  this->inverseKinematics["IK1.2"].Init(this->armStructure.GetAxis("IK1.2"), this->armStructure.GetChildren("IK1.2"), std::vector<unsigned int>{2}, this->dictionary);
  this->inverseKinematics["IK2"] = InverseKinematics();
  this->inverseKinematics["IK2"].Init(this->armStructure.GetAxis("IK2"), this->armStructure.GetChildren("IK2"), std::vector<unsigned int>{6,7}, this->dictionary);
  // Init worldview
  this->worldview.Init(this->world);
  // Init Stats
  this->statistics.Init(this->dictionary["$iter_simu"], this->dictionary["$wait_iter"]);
  this->statistics.CreateFile("Log");
  this->statistics.WriteInFile("Log", "SIMULATION LOG:\n");
  // Init state
  this->state = std::numeric_limits<unsigned int>::max();
  this->action = std::numeric_limits<unsigned int>::max();
  this->prev_state = 0;
  this->init = false;
}

//
unsigned int ArmController::Update()
{
  // is simulation stil running
  if(this->statistics.isSimuReady() == true)
  {
    unsigned int status = 0;
    switch(this->state)
    {
      // Initialization
      case 0:
      {
        status = 1;
        break;
      }
      // Place the hand above an object
      case 1:
      {
        status = this->placeFingersRelativeToObjectAction(this->param[this->action][0],"above", false);
        break;
      }
      // Place hand around an object
      case 2:
      {
        status = this->placeFingersRelativeToObjectAction(this->param[this->action][0], "around", false);
        break;
      }
      // Place hand on around an object
      case 3:
      {
        status = this->placeFingersRelativeToObjectAction(this->param[this->action][0], "on", true);
        break;
      }
      // Close hand
      case 4:
      {
        status = this->closeHandAction();
        break;
      }
      //  Get some hight
      case 5:
      {
        status = this->takeHeightAction();
        break;
      }
      // Move an object above another
      case 6:
      {
        status = this->placeObjectRelativeToObjectAction(this->param[this->action][0], "above", true);
        break;
      }
      // Put down an object
      case 7:
      {
        status = this->putDownObjectAction();
        break;
      }
      //  Open hand
      case 8:
      {
        status = this->openHandAction();
        break;
      }
      // Reset pos
      case 10:
      {
        status = this->standStraightAction();
        break;
      }
      //Get Back Action
      case 11:
      {
        status = this->getBackAction();
        break;
      }
      // Move an object in far front of another
      case 12:
      {
        status = this->placeObjectRelativeToObjectAction(this->param[this->action][0], "farfrontabove", false);
        break;
      }
      // Place the fingers far, in front and above an object
      case 13:
      {
        status = this->placeFingersRelativeToObjectAction(this->param[this->action][0],"farfrontabove", false);
        break;
      }
      // Reset Simulation
      case std::numeric_limits<unsigned int>::max():
      {
        this->state = 0;
        this->action = 0;
        break;
      }
    }
    if(status != 0 && graph[this->action][status] != std::numeric_limits<unsigned int>::max())
    {
      //std::cout << "ACTION: " << this->action << "   STATE: " << this->state <<std::endl;
      this->action = graph[this->action][status];
      this->state = graph[this->action][0];
      //std::cout << "ACTION: " << this->action << "   STATE: " << this->state <<std::endl;
    }
    return this->state;
  }
  return std::numeric_limits<unsigned int>::max();
}

  void ArmController::UpdateModel(const std::vector<std::vector<unsigned int>>& graph,const  std::vector<std::vector<std::string>>& param)
  {
    this->graph = graph;
    this->param = param;
  }

  unsigned int ArmController::standStraightAction()
  {
    // INIT
    if(!this->init)
    {
      this->jointsController.SetTargetsToZero();
      this->init = true;
      this->statistics.WriteInFile("Log", "standStraightAction\n");
    }
    // LOOP
    if(this->reflex)
    {
      this->reflexDo();
    }else{
      if( this->contactsManager.checkCollisions(true))
      {
        this->init = false;
        this->reflex = true;
        this->reflexDo();
        this->jointsController.ReflexToTouch();
      }
      this->model->GetJointController()->Update();
    }
    // END
    if(this->jointsController.AreTargetsReached("IK2", false))
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  unsigned int ArmController::openHandAction()
  {
    // INIT
    if(!this->init)
    {
      this->gripperController.Open();
      this->init = true;
      this->worldview.SetTimer("open",1);
      this->statistics.WriteInFile("Log", "openHandAction\n");
    }
    // LOOP
    this->gripperController.Update();
    this->model->GetJointController()->Update();
    // END
    if(this->gripperController.IsOpen() && this->worldview.IsTimerOff("open")==1)
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  unsigned int ArmController::getBackAction()
  {
    // INIT
    if(!this->init)
    {
      double distance = this->dictionary["$getback_offset"];
      std::vector<gazebo::math::Vector3> eE = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      gazebo::math::Vector3 offset = distance * ( eE[0] - gazebo::math::Vector3(0,0, eE[0].z) ) / ( eE[0] - gazebo::math::Vector3(0,0, eE[0].z) ).GetLength();
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetMovedSingleTarget("IK1.2", -offset );
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1.2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1.2", false);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<double> angles = this->inverseKinematics["IK1.2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1.2",angles);
      this->init = true;
      this->statistics.WriteInFile("Log", "getBacktAction: offset = ");
      this->statistics.WriteInFile("Log", offset);
      this->statistics.WriteInFile("Log", "\n");
    }
    // LOOP
    if(this->reflex)
    {
      this->reflexDo();
    }else{
      if( this->contactsManager.checkCollisions(false) )
      {
        this->init = false;
        this->reflex = true;
        this->reflexDo();
        this->jointsController.ReflexToTouch();
      }
      this->model->GetJointController()->Update();
    }
    // END
    if(this->jointsController.AreTargetsReached("IK1.2", false))
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  unsigned int ArmController::takeHeightAction()
  {
    // INIT
    if(!this->init)
    {
      // Move second link above
      double height = this->dictionary["$takeheight_offset"];
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetMovedSingleTarget("IK1.2", gazebo::math::Vector3( 0,0, height) );
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1.2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1.2", false);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<double> angles = this->inverseKinematics["IK1.2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1.2",angles);
      //Move first link above
      height = this->dictionary["$takeheight_offset"];
      targets = this->armStructure.GetMovedSingleTarget("IK1", gazebo::math::Vector3( 0,0, height) );
      poses = this->armStructure.GetPoses("IK1");
      jointsLimits = this->armStructure.GetRelativeLimits("IK1", false);
      endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1");
      angles = this->inverseKinematics["IK1"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1",angles);

      this->init = true;

      this->statistics.WriteInFile("Log", "takeHeightAction: height = ");
      this->statistics.WriteInFile("Log", height);
      this->statistics.WriteInFile("Log", "\n");
    }
    // LOOP
    if(this->reflex)
    {
      this->reflexDo();
    }else{
      if( this->contactsManager.checkCollisions(false))
      {
        this->init = false;
        this->reflex = true;
        this->reflexDo();
        this->jointsController.ReflexToTouch();
      }
      this->model->GetJointController()->Update();
    }
    // END
    if(this->jointsController.AreTargetsReached("IK1.2", false))
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  unsigned int ArmController::putDownObjectAction()
  {
    // INIT
    if(!this->init)
    {
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<gazebo::math::Vector3> captors = this->armStructure.GetCaptorsEndEffectors();
      std::string boxHandName = this->worldview.GetBoxNameInHand(captors);
      targets[0] -= gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(boxHandName)/3 );
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1.2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1.2",true);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<double> angles = this->inverseKinematics["IK1.2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1.2",angles);
      this->gripperController.Release();
      this->init = true;
      this->statistics.WriteInFile("Log", "putDownObjectAction");
      this->statistics.WriteInFile("Log", "\n");
      this->worldview.SetTimer("putdown",2);
    }
    // LOOP
    this->model->GetJointController()->Update();
    this->gripperController.Update();

    if(this->gripperController.IsClosed())
    {
      this->gripperController.Release();
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<gazebo::math::Vector3> captors = this->armStructure.GetCaptorsEndEffectors();
      std::string boxHandName = this->worldview.GetBoxNameInHand(captors);
      //std::vector<gazebo::math::Vector3> offset =  this->worldview.GetBoxGrabbingPoints(boxHandName, this->worldview.GetBoxSize(boxHandName)/2);
      targets[0] -= gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(boxHandName)/4);
      /*for(unsigned int i = 0; i < captors.size(); i++)
      {
        targets[0] -= ( gazebo::math::Vector3(captors[i].x,captors[i].y, 0) - gazebo::math::Vector3(offset[i].x,offset[i].y, 0) )/captors.size();
      }*/
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1.2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1.2",true);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<double> angles = this->inverseKinematics["IK1.2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1.2",angles);
    }

    // END
    if( this->gripperController.IsReleased() && this->worldview.IsTimerOff("putdown")==1)
    {
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<gazebo::math::Vector3> captors = this->armStructure.GetCaptorsEndEffectors();
      std::string boxHandName = this->worldview.GetBoxNameInHand(captors);
      targets[0] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(boxHandName)/4);
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1.2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1.2",true);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<double> angles = this->inverseKinematics["IK1.2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1.2",angles);
      this->init = false;
      return 1;
    }

    if(this->gripperController.IsReleased())
    {
      this->gripperController.Close();
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<gazebo::math::Vector3> captors = this->armStructure.GetCaptorsEndEffectors();
      std::string boxHandName = this->worldview.GetBoxNameInHand(captors);
      std::vector<gazebo::math::Vector3> offset =  this->worldview.GetBoxGrabbingPoints(boxHandName, this->worldview.GetBoxSize(boxHandName)/2);
      targets[0] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(boxHandName)/4);
      for(unsigned int i = 0; i < captors.size(); i++)
      {
        targets[0] += ( gazebo::math::Vector3(captors[i].x,captors[i].y, 0) - gazebo::math::Vector3(offset[i].x,offset[i].y, 0) )*1.5/captors.size();
      }
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1.2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1.2",true);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1.2");
      std::vector<double> angles = this->inverseKinematics["IK1.2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1.2",angles);
    }
    return 0;
  }


  unsigned int ArmController::placeObjectRelativeToObjectAction(const std::string& objectName,const std::string& where, bool isControllerPrecise)
  {
    // INIT
    if(!this->init)
    {
      std::vector<gazebo::math::Vector3> targets;
      std::vector<gazebo::math::Vector3> captors = this->armStructure.GetCaptorsEndEffectors();
      std::string boxHandName = this->worldview.GetBoxNameInHand(captors);
      double offset = (captors[0] - captors[1]).GetLength()/2;
      if(where == "above")
      {
        targets =  this->worldview.GetBoxGrabbingPoints(objectName, offset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(objectName)/2 + (this->worldview.GetBoxSize(boxHandName)*std::sqrt(2)/2) + this->worldview.GetBoxSize(boxHandName)/7 );
        }
      }else if(where == "front")
      {
        targets =  this->worldview.GetBoxGrabbingPoints(objectName, offset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] -= ( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) )/( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) ).GetLength() * (this->worldview.GetBoxSize(objectName)*std::sqrt(2)/2 + (this->worldview.GetBoxSize(boxHandName)*std::sqrt(2)/2) + this->dictionary["$grabbing_offset"]);
        }
      }else if(where == "farfrontabove")
      {
        targets =  this->worldview.GetBoxGrabbingPoints(objectName, offset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] -= ( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) )/( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) ).GetLength() * (this->worldview.GetBoxSize(boxHandName));
          targets[i] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(objectName)/2 + (this->worldview.GetBoxSize(boxHandName)*std::sqrt(2)/2) + this->worldview.GetBoxSize(boxHandName) );
        }
      }
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK2",true);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetCaptorsEndEffectors();
      std::vector<double> angles = this->inverseKinematics["IK2"].CCD(poses,jointsLimits, endEffectors, targets);
      angles.pop_back(); angles.pop_back(); // for on the number of fingers
      this->jointsController.SetRelativeTargets("IK2.1",angles);
      this->init = true;
      this->statistics.WriteInFile("Log", "placeObjectRelativeToObjectAction: objectName = " + objectName + ", where = "  );
      this->statistics.WriteInFile("Log", where);
      this->statistics.WriteInFile("Log", ", boxInHand = ");
      this->statistics.WriteInFile("Log", boxHandName);
      this->statistics.WriteInFile("Log", ", isControllerPrecise = ");
      this->statistics.WriteInFile("Log", isControllerPrecise);
      this->statistics.WriteInFile("Log", "\n");
    }
    // LOOP
    if(this->reflex)
    {
      this->reflexDo();
    }else{
      if( this->contactsManager.checkCollisions(false))
      {
        this->init = false;
        this->reflex = true;
        this->reflexDo();
        this->jointsController.ReflexToTouch();
      }
      this->model->GetJointController()->Update();
    }
    // END
    if( this->jointsController.AreTargetsReached("IK2.1", isControllerPrecise) )
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  unsigned int ArmController::closeHandAction()
  {
    // INIT
    if(!this->init)
    {
      this->gripperController.Close();
      this->init = true;
      this->statistics.WriteInFile("Log", "closeHandAction\n" );
    }
    // LOOP
    this->gripperController.Update();
    this->model->GetJointController()->Update();
    // END
    if(this->gripperController.IsClosed())
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  unsigned int ArmController::placeFingersRelativeToObjectAction(const std::string& objectName, const std::string& where, bool isControllerPrecise)
  {
    // INIT
    if(!this->init)
    {
      std::vector<gazebo::math::Vector3> targets;
      if(where == "around")
      {
        double grabbingOffset = this->worldview.GetBoxSize(objectName)/2 + this->dictionary["$grabbing_offset"]/2;
        targets = this->worldview.GetBoxGrabbingPoints(objectName, grabbingOffset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(objectName)/3);
        }
      }else if(where == "on")
      {
        double grabbingOffset = this->worldview.GetBoxSize(objectName)/2 +this->dictionary["$grabbing_offset"]/4;
        targets = this->worldview.GetBoxGrabbingPoints(objectName, grabbingOffset);
      }else if(where == "above")
      {
        double grabbingOffset = this->worldview.GetBoxSize(objectName)/2 + this->dictionary["$grabbing_offset"];
        targets = this->worldview.GetBoxGrabbingPoints(objectName, grabbingOffset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(objectName) + this->worldview.GetBoxSize(objectName)/2);
        }
      }else if(where == "front")
      {
        double grabbingOffset = this->worldview.GetBoxSize(objectName)/2 + this->dictionary["$grabbing_offset"];
        targets = this->worldview.GetBoxGrabbingPoints(objectName, grabbingOffset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] -= ( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) )/( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) ).GetLength() * (this->worldview.GetBoxSize(objectName) + this->dictionary["$grabbing_offset"]*2);
        }
      }else if(where == "frontabove")
      {
        double grabbingOffset = this->worldview.GetBoxSize(objectName)/2 + this->dictionary["$grabbing_offset"]*2;
        targets = this->worldview.GetBoxGrabbingPoints(objectName, grabbingOffset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] -= ( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) )/( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) ).GetLength() * (this->worldview.GetBoxSize(objectName) + this->dictionary["$grabbing_offset"]*2);
          targets[i] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(objectName));
        }
      }else if(where == "farfrontabove")
      {
        double grabbingOffset = this->worldview.GetBoxSize(objectName)/2 + this->dictionary["$grabbing_offset"]*2;
        targets =  this->worldview.GetBoxGrabbingPoints(objectName, grabbingOffset);
        for(unsigned int i = 0; i < targets.size(); i++)
        {
          targets[i] -= ( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) )/( targets[i] - gazebo::math::Vector3(0,0, targets[i].z) ).GetLength() * (this->dictionary["$getback_offset"]/2+this->worldview.GetBoxSize(objectName));
          targets[i] += gazebo::math::Vector3(0,0,this->worldview.GetBoxSize(objectName)*2 );
        }
      }
      std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK2");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK2",false);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetCaptorsEndEffectors();
      std::vector<double> angles = this->inverseKinematics["IK2"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK2",angles);
      this->init = true;
      this->statistics.WriteInFile("Log", "placeFingersRelativeToObjectAction: objectName = " + objectName + ", where = "  );
      this->statistics.WriteInFile("Log", where);
      this->statistics.WriteInFile("Log", ", isControllerPrecise = ");
      this->statistics.WriteInFile("Log", isControllerPrecise);
      this->statistics.WriteInFile("Log", "\n");
    }
    // LOOP
    if(this->reflex)
    {
      if(this->reflexDo())
      {
        return 2;
      }
    }else{
      if( (this->contactsManager.checkCollisions(true) && where != "on") || this->contactsManager.checkCollisions(false))
      {
        this->init = false;
        this->reflex = true;
        this->reflexDo();
        this->jointsController.ReflexToTouch();
      }
      this->model->GetJointController()->Update();
    }
    // END
    if(this->jointsController.AreTargetsReached("IK2", isControllerPrecise))
    {
      this->init = false;
      return 1;
    }
    return 0;
  }

  void ArmController::waitDo(unsigned int seconds)
  {
    // INIT
    if(!this->init)
    {
      this->init = true;
      this->worldview.SetTimer("wait",seconds);
      this->statistics.WriteInFile("Log", "waitDo: seconds = ");
      this->statistics.WriteInFile("Log", seconds);
      this->statistics.WriteInFile("Log", "\n");
    }
    // LOOP
    this->model->GetJointController()->Update();
    // END
    if(this->worldview.IsTimerOff("wait"))
    {
      this->init = false;
    }
  }

  bool ArmController::reflexDo()
  {
    // INIT
    if(!this->init)
    {
      std::vector<gazebo::math::Vector3> targets = this->armStructure.GetMovedSingleTarget("IK1", gazebo::math::Vector3( (double)std::rand() * this->dictionary["$reflex_xyz"] / (RAND_MAX),(double)std::rand() * this->dictionary["$reflex_xyz"] / (RAND_MAX),(double)std::rand() * this->dictionary["$reflex_xyz"] / (RAND_MAX) + this->dictionary["$reflex_z"]));          std::vector<gazebo::math::Pose> poses = this->armStructure.GetPoses("IK1");
      std::vector<std::vector<double>> jointsLimits = this->armStructure.GetRelativeLimits("IK1", false);
      std::vector<gazebo::math::Vector3> endEffectors = this->armStructure.GetNaturalSingleEndEffectors("IK1");
      std::vector<double> angles = this->inverseKinematics["IK1"].CCD(poses,jointsLimits, endEffectors, targets);
      this->jointsController.SetRelativeTargets("IK1",angles);
      this->init = true;
      this->statistics.WriteInFile("Log", "reflexDo \n");
      this->worldview.SetTimer("reflex",1);
    }
    // LOOP
    this->model->GetJointController()->Update();
    if(this->worldview.IsTimerOff("reflex") >= 1)
    {
      if( this->contactsManager.checkCollisions(false))
      {
        this->init = false;
        this->jointsController.ReflexToTouch();
      }
    }
    //END
    if(this->jointsController.AreTargetsReached("IK1", false))
    {
        this->init = false;
        this->reflex = false;
        return true;
    }
    return false;
  }
