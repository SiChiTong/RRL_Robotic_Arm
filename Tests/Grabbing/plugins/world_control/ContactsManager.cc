#include "ContactsManager.hh"

ContactsManager::ContactsManager()
{

}

ContactsManager::~ContactsManager()
{
  gazebo::transport::fini();
}

void ContactsManager::Init(gazebo::physics::ContactManager* contactManager, std::string worldName, unsigned int freq,Statistics *statistics)
{
  this->statistics = statistics;
  this->contactManager = contactManager;
  this->isContact = false;
  this->freq = freq;
  this->varfreq = 1;
  // Subscribe to Contact topic
  this->contactNod = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->contactNod->Init(worldName);
  this->contactSub = this->contactNod->Subscribe("~/physics/contacts", &ContactsManager::contactsCallBack, this);

}

bool ContactsManager::checkCollisions()
{
  if(this->isContact == true)
  {
      this->isContact = false;
      //std::cout << "CONTACT !" << std::endl;
      if(this->statistics->GetFileFlag())
      {
        this->statistics->WriteInFile("grabbox", "contact;");this->statistics->WriteInFile("grabbox",this->name1 + ";");this->statistics->WriteInFile("grabbox", this->name2 + "\n");
      }
      return true;
  }
  return false;
}

void ContactsManager::contactsCallBack(ConstWorldStatisticsPtr &_msg)
{
  //std::cout << this->varfreq << "/" << this->freq << std::endl;
  if(this->varfreq == this->freq )
  {
    //std::cout << "ok" << std::endl;
    this->varfreq = 1;
    unsigned int count = this->contactManager->GetContactCount();
    std::vector<gazebo::physics::Contact*> contacts = this->contactManager->GetContacts();
    for(unsigned int i=0; i < count; i++)
    {
      std::string name1 = contacts[i]->collision1->GetName();
      std::string name2 = contacts[i]->collision2->GetName();
      //std::cout << name1 << "   " << name2 << std::endl;
      if( (name1.substr(0,3) != "arm") != (name2.substr(0,3) != "arm") && (name1 != "collision" && name2 != "collision") /*&& (name1.find("touch") == std::string::npos) && (name2.find("touch") == std::string::npos)*/ )
      {
        //std::cout << "[contact] " << i << ": " << name1 << " -- " << name2 << std::endl;
        this->isContact = true;
        this->name1 = name1;
        this->name2 = name2;
        return;
      }
    }
  }else{
    this->varfreq++;
  }
}
