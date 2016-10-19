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
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    // Arm Controller
    private: ArmController armController;

    private: unsigned long int iteration;

    public: ArmWolrdPlugin() : WorldPlugin()
    {
      printf("Starting ArmWolrdPlugin Plugin!\n");
    }
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Load dictionary to generate files from templates
      std::map<std::string, double> dictionary = loadDictionary("Models/dictionary.dic");

      // Insert table
      sdf::SDF tableSDF;
      std::string table_template = getFileAsString(std::string("Models/table.sdf.template"));
      std::string tableString = generateModel(table_template, dictionary);
      tableSDF.SetFromString(tableString);
      _parent->InsertModelSDF(tableSDF);

      // Insert cube red
      sdf::SDF cuberedSDF;
      std::string boxred_template = getFileAsString(std::string("Models/box_red.sdf.template"));
      std::string cuberedString = generateModel(boxred_template, dictionary);
      cuberedSDF.SetFromString(cuberedString);
      //sdf::ElementPtr model = sphereSDF.root->GetElement("model");
      //model->GetAttribute("name")->SetFromString("unique_sphere");
      _parent->InsertModelSDF(cuberedSDF);

      // Insert cube blue
      sdf::SDF cubeblueSDF;
      std::string boxblue_template = getFileAsString(std::string("Models/box_blue.sdf.template"));
      std::string cubeblueString = generateModel(boxblue_template, dictionary);
      cubeblueSDF.SetFromString(cubeblueString);
      _parent->InsertModelSDF(cubeblueSDF);

      // Insert arm
      sdf::SDF armSDF;
      std::string arm_template = getFileAsString(std::string("Models/arm.sdf.template"));
      std::string armString = generateModel(arm_template, dictionary);
      armSDF.SetFromString(armString);
      _parent->InsertModelSDF(armSDF);

      // Store the world
      this->world = _parent;
      this->iteration = 0;
      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ArmWolrdPlugin::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      if(this->iteration == 2)
      {
        std::cout << this->world->GetModels().size() << std::endl;
        // Create the rm controller
        this->armController.Init(this->world->GetModel("arm"));
      }else if(this->iteration > 2){
        this->armController.Update();
      }
      this->iteration++;
      //std::cout << this->iteration << std::endl;

      //std::cout << this->world->GetSimTime() << std::endl;
    }

  };

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ArmWolrdPlugin)
}
