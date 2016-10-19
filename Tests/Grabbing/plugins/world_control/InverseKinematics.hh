#ifndef DEF_INVERSEKINEMATICS
#define DEF_INVERSEKINEMATICS

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "Statistics.hh"

class InverseKinematics
{
  private:
    // PRIVATE ATTRIBUTES

     // Define orientation, and tree structure (fixed from begining)-
     std::vector<gazebo::math::Vector3> axis;
     std::vector<std::vector<unsigned int>> children;
     std::vector<unsigned int> endEffJoints;
     Statistics *statistics;


     // Build from previous structure
     unsigned int targetsNumber;
     unsigned int jointsNumber;
     std::vector<std::vector<unsigned int>> jointsToEndEff;

     // Used for IK algo
     std::vector<gazebo::math::Pose> poses;
     std::vector<std::vector<double>> jointsLimits;

     std::vector<double> finalAngles;
     std::vector<gazebo::math::Vector3> endEffectors;
     std::vector<gazebo::math::Vector3> targets;



     // Algorithm param
     double errormax;
     double converror;
     unsigned int itermax;

     // PRIVATE METHODS
     void initJointsToEndEff(unsigned int i);
     double computeDescent(unsigned int i, gazebo::math::Vector3 endEffector, gazebo::math::Vector3 target);
     void forwardKinematic(unsigned int i, double angle);
     double regularizeAngle(unsigned int i, double angle);
     void resetFinalAngles();
     double get3DAngle(gazebo::math::Vector3 v1, gazebo::math::Vector3 v2);
     double getError(std::string errortype, std::vector<gazebo::math::Vector3> endEffectors);
     double getMaxError(std::vector<gazebo::math::Vector3> endEffectors);
     double getMeanError(std::vector<gazebo::math::Vector3> endEffectors);
     std::vector<unsigned int> uniqueConcat(std::vector<unsigned int> A, std::vector<unsigned int> B);
     void forwardPass(unsigned int i, unsigned int j, gazebo::math::Quaternion quater);
     bool updateKinematicParameters(std::vector<gazebo::math::Pose> poses,std::vector<std::vector<double>> jointsLimits, std::vector<gazebo::math::Vector3> endEffectors, std::vector<gazebo::math::Vector3> targets);
     std::vector<gazebo::math::Vector3> computeEndEffectors(unsigned int i, double angle);
     std::vector<double> normalizeAngles(std::vector<double> angles);

   public:
     // PUBLIC METHODS
     InverseKinematics();
     ~InverseKinematics();
     void Init(std::vector<gazebo::math::Vector3> axis, std::vector<std::vector<unsigned int>> children, std::vector<unsigned int> endEffJoints, std::map<std::string, double> dictionary, Statistics *statistics);
     std::vector<double> CCD(std::vector<gazebo::math::Pose> poses, std::vector<std::vector<double>> jointsLimits, std::vector<gazebo::math::Vector3> endEffectors, std::vector<gazebo::math::Vector3> targets);
     std::vector<double> FAB(std::vector<gazebo::math::Pose> poses, std::vector<std::vector<double>> jointsLimits, std::vector<gazebo::math::Vector3> endEffectors, std::vector<gazebo::math::Vector3> targets);
};

#endif
