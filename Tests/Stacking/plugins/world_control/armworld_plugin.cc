#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <string>
#include "template.hh"
#include "ArmController.hh"
#include "Statistics.hh"

namespace gazebo
{
  class ArmWolrdPlugin : public WorldPlugin
  {
    // Pointer to the world
    private: physics::WorldPtr world;
    // To remove soon
    private: unsigned int step;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    // Arm Controller
    private: ArmController armController;

    private: unsigned long int iteration;

    private: std::vector<std::vector<std::string>> sequence;
    
    Statistics statistics;

    public: ArmWolrdPlugin() : WorldPlugin()
    {
      printf("Starting ArmWolrdPlugin Plugin!\n");
    }
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
    // Store the world
      this->world = _parent;
      this->iteration = 0;
      
      // Load dictionary to generate files from templates
      std::map<std::string, double> dictionary = loadDictionary("Models/dictionary.dic");
      this->statistics.Init(5001, 0);
      this->statistics.CreateFile("Stacking");
      std::vector<std::string> listModels = std::vector<std::string>{"Models/table.sdf.template", "Models/box_red.sdf.template", "Models/arm.sdf.template"};
      for(unsigned int i = 0; i < listModels.size(); i++)
      {
        sdf::SDF modelSDF;
        std::string model_template = getFileAsString(std::string(listModels[i]));
        std::string modelString = generateModel(model_template, dictionary);
        modelSDF.SetFromString(modelString);
        _parent->InsertModelSDF(modelSDF);
      }
      
      for(unsigned int i = 1; i <= 9; i++)
      {
        sdf::SDF modelSDF;
        std::string model_template = getFileAsString("Models/box_red.sdf.template");
        std::string modelString = generateModel(model_template, dictionary);
        modelSDF.SetFromString(modelString);
        sdf::ElementPtr model = modelSDF.root->GetElement("model");
        std::string name = std::string("boxred")+std::to_string(i);
        model->GetAttribute("name")->SetFromString(name);
        _parent->InsertModelSDF(modelSDF);
      }
      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ArmWolrdPlugin::OnUpdate, this, _1));
      std::cout << "Plugin loaded !" << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Wait for gazebo environment to be ready
      if(this->iteration == 2)
      {
        // Create the rm controller
        this->armController.Init(this->world->GetModel("arm"));
        this->sequence = this->Stacking(9);
        this->step = this->sequence.size();
        for(unsigned int i = 0; i < this->sequence.size(); i++)
        {
          for(unsigned int j = 0; j < this->sequence[i].size(); j++)
          {
            std::cout << this->sequence[i][j] << " ";
          }
          std::cout << std::endl;
        }
      }
      // Main loop of the simulation
      else if( (this->iteration > 2) && (this->statistics.isSimuOver() == false) )
      {
        unsigned int state = this->armController.Update();
        if(state == 0)
        {
          if(this->step >= this->sequence.size())
          {
            this->InitSimulation();
            this->step = 0;
          }
          if(this->sequence[this->step][0] == "MOVE")
          {
            this->initBoxAction(this->sequence[this->step][1]);
            this->MoveAonB(this->sequence[this->step][1], this->sequence[this->step][2]);
            if(this->step != 0)
            {
              this->checkBox(this->sequence[this->step][2]);
            }
          }
          this->step++;
          if(this->statistics.isSimuOver() == true)
          {
            this->statistics.CloseFiles();
          }
        }
      }
      this->iteration++;
    }
    
    private: void checkBox(std::string boxname)
    {
      gazebo::physics::ModelPtr obj = this->world->GetModel(boxname);
      gazebo::math::Pose objpose = obj->GetWorldPose();
      gazebo::math::Vector3 objpos = objpose.pos; // absolute centroid position of obj
      std::string integer = boxname.substr(boxname.length() - 1,1);
      unsigned int k = std::stoul(integer);
      double size = this->world->GetModel(boxname)->GetCollisionBoundingBox().GetZLength();
      double height = size/2 + k*size + 1.03;
      if(objpos.z > height+size/10 || objpos.z < height-size/10)
      {
        this->statistics.WriteInFile("Stacking", boxname);
        this->statistics.WriteInFile("Stacking", ";");
        this->statistics.WriteInFile("Stacking", k);
        this->statistics.WriteInFile("Stacking", ";");
        this->statistics.WriteInFile("Stacking", objpos);
        this->statistics.WriteInFile("Stacking", "\n");

        this->InitSimulation();
        this->step = 0;
        this->initBoxAction(this->sequence[this->step][1]);
        this->MoveAonB(this->sequence[this->step][1], this->sequence[this->step][2]);
      }
    }
    
    private: void initBoxAction(std::string boxname)
    {
      double xmin = 0.2;
      double xmax = 0.6;
      double ymin = 0.5;
      double ymax = 1;
      double qmin = 0;
      double qmax = 2*M_PI;
      double xred = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
      double yred = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
      double qred = (qmax - qmin) * ( (double)rand() / (double)RAND_MAX ) + qmin;
      this->world->GetModel(boxname)->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(xred,yred,1.04),gazebo::math::Quaternion(0,0,qred) ),std::string("boxredlink"));
    }

    public: void MoveAonB(const std::string& A, const std::string& B)
    {
      std::vector<std::vector<unsigned int>> graph;
      std::vector<std::vector<std::string>> param;
      graph.push_back(std::vector<unsigned int>{0,1}); param.push_back(std::vector<std::string>{A});// Init state
      graph.push_back(std::vector<unsigned int>{15,2}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{13,3,1}); param.push_back(std::vector<std::string>{A});//
      graph.push_back(std::vector<unsigned int>{1,4,1}); param.push_back(std::vector<std::string>{A});//
      graph.push_back(std::vector<unsigned int>{2,5,1}); param.push_back(std::vector<std::string>{A});
      graph.push_back(std::vector<unsigned int>{3,6,1}); param.push_back(std::vector<std::string>{A});
      graph.push_back(std::vector<unsigned int>{4,7}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{5,8}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{11,9}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{5,10}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{12,11}); param.push_back(std::vector<std::string>{B});
      graph.push_back(std::vector<unsigned int>{6,12}); param.push_back(std::vector<std::string>{B});
      graph.push_back(std::vector<unsigned int>{7,13}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{8,14}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{5,15}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{11,16}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{14,17}); param.push_back(std::vector<std::string>{A});
      graph.push_back(std::vector<unsigned int>{std::numeric_limits<unsigned int>::max()}); param.push_back(std::vector<std::string>{});// End state
      this->armController.UpdateModel(graph, param);
    }
    
    public: std::vector<std::vector<std::string>> Stacking(unsigned int n)
    {
      std::vector<std::vector<std::string>> sequence;
      for(unsigned int i = 1; i <= n; i++)
      {
        std::vector<std::string> move = std::vector<std::string>{"MOVE", "boxred"+std::to_string(i), "boxred"+std::to_string(i-1)};
        sequence.push_back(move);
      }
      return sequence;
    }

    public: void InitSimulation()
    {
      this->statistics.updateSimu();
      double offset = 0;
      for(unsigned int i = 1; i < 9; i++)
      {
        this->world->GetModel("boxred"+std::to_string(i))->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(3,3,3 + offset),gazebo::math::Quaternion(0,0,0) ),std::string("boxredlink"));
        offset += this->world->GetModel("boxred"+std::to_string(i))->GetCollisionBoundingBox().GetXLength() + 0.10;
      }
      
      double xmin = -0.6;
      double xmax = -0.2;
      double ymin = 0.5;
      double ymax = 1;
      double qmin = 0;
      double qmax = 2*M_PI;
      double xred = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
      double yred = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
      double qred = (qmax - qmin) * ( (double)rand() / (double)RAND_MAX ) + qmin;
      this->world->GetModel("boxred0")->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(xred,yred,1.04),gazebo::math::Quaternion(0,0,qred) ),std::string("boxredlink"));
      
      this->statistics.WriteInFile("Stacking", gazebo::math::Vector3(xred,yred,1.04));
      this->statistics.WriteInFile("Stacking", ";");
      this->statistics.WriteInFile("Stacking", gazebo::math::Quaternion(0,0,qred));
      this->statistics.WriteInFile("Stacking", ";");
      
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ArmWolrdPlugin)
}
