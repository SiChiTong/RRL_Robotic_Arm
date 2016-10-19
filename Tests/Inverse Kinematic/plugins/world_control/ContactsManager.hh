#ifndef DEF_CONTACTSMANAGER
#define DEF_CONTACTSMANAGER

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <cctype>
#include <math.h>

class ContactsManager
{
  private:
    // PRIVATE ATTRIBUTES
    gazebo::physics::ContactManager* contactManager;
    bool isContact;
    gazebo::transport::SubscriberPtr contactSub;
    gazebo::transport::NodePtr contactNod;
    unsigned int freq;
    unsigned int varfreq;
    // PRIVATE METHODS
    void contactsCallBack(ConstWorldStatisticsPtr &_msg);


  public:
    // PUBLIC METHODS
    ContactsManager();
    ~ContactsManager();
    void Init(gazebo::physics::ContactManager* contactManager, std::string worldName, unsigned int freq);
    bool checkCollisions();
};
#endif
