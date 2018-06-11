
#include <vigir_footstep_planning_rviz_plugin/status_prompt.h>

namespace vigir_footstep_planning_rviz_plugin {

StatusPrompt::StatusPrompt(QWidget *parent) :
    QTextEdit(parent)
{
  initialize();
}

StatusPrompt::~StatusPrompt()
{

}


void StatusPrompt::initialize()
{
  this->setReadOnly(true);
}

void StatusPrompt::displayConnection(QString action_server_name, bool success)
{
  if(success)
  {
    displaySuccess("> Connected to Action Server:\n" + action_server_name + "\n");
  }
  else
    displayWarn("> Could not connect to Action Server:\n" + action_server_name + "\n");
}


void StatusPrompt::displayMessage(QString message)
{
  this->setTextColor(QColor("black"));
  this->append(message);
}

void StatusPrompt::displayError(QString message)
{
  this->setTextColor(QColor("red"));
  this->append(message);
}

void StatusPrompt::displaySuccess(QString message)
{
  this->setTextColor(QColor("green"));
  this->append(message);
}

void StatusPrompt::displayWarn(QString message)
{
  this->setTextColor(QColor("orange"));
  this->append(message);
}


}// End namespace vigir_footstep_planning_rviz_plugin
