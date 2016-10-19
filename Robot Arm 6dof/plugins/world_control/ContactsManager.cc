#include "ContactsManager.hh"

ContactsManager::ContactsManager()
{

}

ContactsManager::~ContactsManager()
{
  gazebo::transport::fini();
}

void ContactsManager::Init(gazebo::physics::ContactManager* contactManager, const std::string& worldName, unsigned int freq)
{
  this->contactManager = contactManager;
  this->isContact = 0;
  this->freq = freq;
  this->varfreq = 1;
  // Subscribe to Contact topic
  this->contactNod = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->contactNod->Init(worldName);
  this->contactSub = this->contactNod->Subscribe("~/physics/contacts", &ContactsManager::contactsCallBack, this);

}

bool ContactsManager::checkCollisions(bool touch)
{
  //std::cout << this->varfreq << "/" << this->freq << std::endl;
  if(this->varfreq == this->freq )
  {
    //std::cout << "ok" << std::endl;
    this->varfreq = 1;
    this->isContact = 0;
    unsigned int count = this->contactManager->GetContactCount();
    std::vector<gazebo::physics::Contact*> contacts = this->contactManager->GetContacts();
    unsigned int nbContact = 0;
    unsigned int nbTouch = 0;
    //std::cout  << std::endl;
    for(unsigned int i=0; i < count; i++)
    {
      std::string name1 = contacts[i]->collision1->GetName();
      std::string name2 = contacts[i]->collision2->GetName();
      if( (name1.substr(0,3) != "arm") != (name2.substr(0,3) != "arm") && (name1 != "collision" && name2 != "collision") )
      {
        this->isContact = 1;
        nbContact++;
        if ( (name1.find("touch") == std::string::npos) || (name2.find("touch") == std::string::npos) )
        {
          nbTouch++;
        }
        //std::cout << name1 << "   " << name2 << "   " << nbContact << "    "  << nbTouch << std::endl;
      }
    }
    //std::cout << std::endl;
    if(this->isContact == 1 && nbTouch == nbContact)
    {
      this->isContact = 2;
    }
    //std::cout << this->isContact << std::endl;
  }else{
    this->varfreq++;
  }
  //std::cout << "Contact: " << this->isContact << std::endl;
  //std::cout << "Touch: " << touch << std::endl;
  if( (this->isContact == 1) || ( this->isContact == 2 && touch == true) )
  {
      std::cout << "Contact : isContact = " << this->isContact << "  touch = " << touch << std::endl;
      this->isContact = 0;
      return true;
  }
  return false;
}


// Using call back mess up, (cllback not terminated when checkCollison called)
void ContactsManager::contactsCallBack(ConstWorldStatisticsPtr &_msg)
{

}
