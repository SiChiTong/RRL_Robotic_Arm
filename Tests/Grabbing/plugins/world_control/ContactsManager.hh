#ifndef DEF_CONTACTSMANAGER
#define DEF_CONTACTSMANAGER

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <cctype>
#include <math.h>
#include "Statistics.hh"

class ContactsManager
{
  private:
    // PRIVATE ATTRIBUTES
    Statistics *statistics;
    gazebo::physics::ContactManager* contactManager;
    bool isContact;
    std::string name1;
    std::string name2;
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
    void Init(gazebo::physics::ContactManager* contactManager, std::string worldName, unsigned int freq,Statistics *statistics);
    bool checkCollisions();
};
#endif
