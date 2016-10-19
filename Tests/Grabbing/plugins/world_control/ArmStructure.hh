#ifndef DEF_ARMSTRUCTURE
#define DEF_ARMSTRUCTURE

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cctype>
#include <math.h>
#include <map>

/*
To build and use chains, we need to keep in mind that the tree is build (warning not implemented) top-down -> left-right.
*/
//

struct linkage
{
  unsigned int rank;
  unsigned int nbchild;
  std::string jointName;
  std::string linkName;
  std::string jointScopedName;
  std::string linkScopedName;
  gazebo::physics::JointPtr joint;
  gazebo::physics::LinkPtr link;
  std::vector<unsigned int> children;
};

class ArmStructure
{
  private:
    // PRIVATE ATTRIBUTES
    double limitoffset;
    std::map<std::string, unsigned int> map;
    std::vector<struct linkage> linkages;
    std::vector<gazebo::physics::JointPtr> joints;
    std::map<std::string,std::vector<gazebo::physics::JointPtr>> chains;
    std::map<std::string,std::vector<std::vector<unsigned int>>> chainsChildren;
    std::vector<gazebo::physics::JointPtr> fingers;
    std::vector<std::vector<gazebo::physics::JointPtr>> captors;

    unsigned int maxRank;
    // PRIVATE METHODS
    void buildStructure(gazebo::physics::JointPtr rootJoint, unsigned int indice);
    void buildChildren(std::string name);
    void findCaptors(gazebo::physics::JointPtr rootJoint, unsigned int i);

  public:
    // PUBLIC METHODS
    ArmStructure();
    ~ArmStructure();
    void Init(gazebo::physics::JointPtr rootJoint, double limitoffset);
    void CreateChain(std::string name,unsigned int min,unsigned int max);
    std::vector<gazebo::physics::JointPtr> GetJoints(std::string name);
    std::vector<gazebo::physics::JointPtr> GetAllJoints();
    std::vector<gazebo::math::Vector3> GetAxis(std::string name);
    std::vector<gazebo::math::Pose> GetPoses(std::string name);
    std::vector<std::vector<double>> GetRelativeLimits(std::string name, bool fixFingers);
    std::vector<std::vector<double>> GetLimits(std::string name);
    std::vector<std::vector<unsigned int>> GetChildren(std::string name);
    std::vector<unsigned int> GetRanks(std::string name);
    std::vector<gazebo::physics::JointPtr> GetFingers();
    std::vector<int> GetFingersSigns();
    std::vector<std::vector<gazebo::physics::JointPtr>> GetCaptors();
    std::vector<gazebo::math::Vector3> GetCaptorsEndEffectors();
    std::vector<gazebo::math::Vector3> GetMovedSingleTarget(std::string name, gazebo::math::Vector3 scale);
    std::vector<gazebo::math::Vector3> GetNaturalSingleEndEffectors(std::string name);


};
#endif
