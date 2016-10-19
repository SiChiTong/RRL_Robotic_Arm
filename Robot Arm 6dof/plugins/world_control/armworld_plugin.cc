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

    public: ArmWolrdPlugin() : WorldPlugin()
    {
      printf("Starting ArmWolrdPlugin Plugin!\n");
    }
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Load dictionary to generate files from templates
      std::map<std::string, double> dictionary = loadDictionary("Models/dictionary.dic");

      std::vector<std::string> listModels = std::vector<std::string>{"Models/table.sdf.template", "Models/box_red.sdf.template"/*, "Models/box_blue.sdf.template"*/,"Models/box_green.sdf.template","Models/box_yellow.sdf.template","Models/plate_left.sdf.template","Models/plate_middle.sdf.template","Models/plate_right.sdf.template", "Models/arm.sdf.template"};
      for(unsigned int i = 0; i < listModels.size(); i++)
      {
        sdf::SDF modelSDF;
        std::string model_template = getFileAsString(std::string(listModels[i]));
        std::string modelString = generateModel(model_template, dictionary);
        modelSDF.SetFromString(modelString);
        _parent->InsertModelSDF(modelSDF);
      }

      // Store the world
      this->world = _parent;
      this->iteration = 0;
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
        this->MoveAonB("boxyellow", "plateright");
        std::vector<std::string> D = std::vector<std::string>{"plateleft",/*"boxblue",*/ "boxred", "boxgreen", "boxyellow"};
        std::vector<std::string> A = std::vector<std::string>{"plateright"};
        std::vector<std::string> I = std::vector<std::string>{"platemiddle"};
        this->sequence = this->Hanoi(3,D,A,I);
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
      else if(this->iteration > 2)
      {
        unsigned int state = this->armController.Update();
        if(state == 0)
        {
          if(this->step == this->sequence.size())
          {
            this->InitSimulation();
            this->step = 0;
          }
          if(this->sequence[this->step][0] == "MOVE")
          {
            this->MoveAonB(this->sequence[this->step][1], this->sequence[this->step][2]);
          }
          this->step++;
        }
      }
      this->iteration++;
    }

    public: void MoveAonB(const std::string& A, const std::string& B)
    {
      std::vector<std::vector<unsigned int>> graph;
      std::vector<std::vector<std::string>> param;
      graph.push_back(std::vector<unsigned int>{0,1}); param.push_back(std::vector<std::string>{});// Init state
      graph.push_back(std::vector<unsigned int>{13,2,1}); param.push_back(std::vector<std::string>{A});//
      graph.push_back(std::vector<unsigned int>{1,3,1}); param.push_back(std::vector<std::string>{A});//
      graph.push_back(std::vector<unsigned int>{2,4,1}); param.push_back(std::vector<std::string>{A});
      graph.push_back(std::vector<unsigned int>{3,5,1}); param.push_back(std::vector<std::string>{A});
      graph.push_back(std::vector<unsigned int>{4,6}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{5,7}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{11,8}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{5,9}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{12,10}); param.push_back(std::vector<std::string>{B});
      graph.push_back(std::vector<unsigned int>{6,11}); param.push_back(std::vector<std::string>{B});
      graph.push_back(std::vector<unsigned int>{7,12}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{8,13}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{5,14}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{11,15}); param.push_back(std::vector<std::string>{});
      graph.push_back(std::vector<unsigned int>{std::numeric_limits<unsigned int>::max()}); param.push_back(std::vector<std::string>{});// End state
      this->armController.UpdateModel(graph, param);
    }

    public: std::vector<std::vector<std::string>> Hanoi(unsigned int n,std::vector<std::string>& D, std::vector<std::string>& A, std::vector<std::string>& I)
    {
      std::vector<std::vector<std::string>> sequence;
      if(n != 0)
      {
        // First recursion
        sequence = this->Hanoi(n-1,D,I,A);
        // Move D to A
        std::vector<std::string> move = std::vector<std::string>{"MOVE", D.back(), A.back()};
        A.push_back(D.back());
        D.pop_back();
        sequence.push_back(move);
        // Last recursion
        std::vector<std::vector<std::string>> nextSequence = this->Hanoi(n-1,I,A,D);
        sequence.insert( sequence.end(), nextSequence.begin(), nextSequence.end() );
      }
      return sequence;
    }


    public: void InitSimulation()
    {
      std::vector<std::string> listModels = std::vector<std::string>{/*"boxblue",*/ "boxred", "boxgreen", "boxyellow"};
      double offset = 0;
      for(unsigned int i = 0; i < listModels.size(); i++)
      {
        this->world->GetModel(listModels[i])->SetLinkWorldPose(gazebo::math::Pose(gazebo::math::Vector3(-0.45,0.9,1.16 + offset),gazebo::math::Quaternion(0,0,0) ),std::string(listModels[i] + "link"));
        offset += this->world->GetModel(listModels[i])->GetCollisionBoundingBox().GetXLength() + 0.10;
      }
    }
  };

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ArmWolrdPlugin)
}
