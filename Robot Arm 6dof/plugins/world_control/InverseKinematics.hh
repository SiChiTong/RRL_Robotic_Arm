#ifndef DEF_INVERSEKINEMATICS
#define DEF_INVERSEKINEMATICS

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

class InverseKinematics
{
  private:
    // PRIVATE ATTRIBUTES

     // Define orientation, and tree structure (fixed from begining)-
     std::vector<gazebo::math::Vector3> axis;
     std::vector<std::vector<unsigned int>> children;
     std::vector<unsigned int> endEffJoints;


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
     double computeDescent(unsigned int i, const gazebo::math::Vector3& endEffector, const gazebo::math::Vector3& target);
     void forwardKinematic(unsigned int i, double angle);
     double regularizeAngle(unsigned int i, double angle);
     void resetFinalAngles();
     double get3DAngle(const gazebo::math::Vector3& v1, const gazebo::math::Vector3& v2);
     double getError(const std::string& errortype, const std::vector<gazebo::math::Vector3>& endEffectors);
     double getMaxError(const std::vector<gazebo::math::Vector3>& endEffectors);
     double getMeanError(const std::vector<gazebo::math::Vector3>& endEffectors);
     std::vector<unsigned int> uniqueConcat(const std::vector<unsigned int>& A, const std::vector<unsigned int>& B);
     void forwardPass(unsigned int i, unsigned int j, const gazebo::math::Quaternion& quater);
     bool updateKinematicParameters(const std::vector<gazebo::math::Pose>& poses,const std::vector<std::vector<double>>& jointsLimits, const std::vector<gazebo::math::Vector3>& endEffectors, const std::vector<gazebo::math::Vector3>& targets);
     std::vector<gazebo::math::Vector3> computeEndEffectors(unsigned int i, double angle);
     std::vector<double> normalizeAngles(const std::vector<double>& angles);

   public:
     // PUBLIC METHODS
     InverseKinematics();
     ~InverseKinematics();
     void Init(const std::vector<gazebo::math::Vector3>& axis, const std::vector<std::vector<unsigned int>>& children, const std::vector<unsigned int>& endEffJoints, std::map<std::string, double>& dictionary);
     std::vector<double> CCD(const std::vector<gazebo::math::Pose>& poses, const std::vector<std::vector<double>>& jointsLimits, const std::vector<gazebo::math::Vector3>& endEffectors, const std::vector<gazebo::math::Vector3>& targets);
     std::vector<double> FAB(const std::vector<gazebo::math::Pose>& poses, const std::vector<std::vector<double>>& jointsLimits, const std::vector<gazebo::math::Vector3>& endEffectors, const std::vector<gazebo::math::Vector3>& targets);
};

#endif
