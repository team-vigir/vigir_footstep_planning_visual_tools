#include <vigir_footstep_planning_rviz_plugin/parameter_set_combo_box.h>


namespace vigir_footstep_planning_rviz_plugin
{

ParameterSetComboBox::ParameterSetComboBox(QWidget *parent) :
    QComboBox(parent)
  , parameter_set_ac("/johnny5/footstep_planning/params/get_all_parameter_sets", true)
{
  this->addItem("default");
  this->setCurrentIndex(0);
}

ParameterSetComboBox::~ParameterSetComboBox()
{

}

void ParameterSetComboBox::initialize()
{
  if(parameter_set_ac.waitForServer(ros::Duration(1,0)))
    Q_EMIT(actionClientConnected( "/johnny5/footstep_planning/params/get_all_parameter_sets", true));
  else
    Q_EMIT(actionClientConnected( "/johnny5/footstep_planning/params/get_all_parameter_sets", false));

  updateParameterSetNames();
}

void ParameterSetComboBox::updateParameterSetNames()
{
  if(parameter_set_ac.isServerConnected())
  {
    vigir_generic_params::GetAllParameterSetsGoal goal;
    parameter_set_ac.sendGoal(goal,
                          boost::bind(&ParameterSetComboBox::getParameterSetsCallback, this, _1, _2),
                              GetAllParameterSetsActionClient::SimpleActiveCallback(),
                              GetAllParameterSetsActionClient::SimpleFeedbackCallback());
  }
  else
  {
    ROS_WARN("get_all_parameter_sets not available! Please activate an \"//johnny5/footstep_planning/params/get_all_parameter_sets\" action server.");

    this->clear();
    this->addItem("default");
    this->setCurrentIndex(0);
  }
}


void ParameterSetComboBox::getParameterSetsCallback(const actionlib::SimpleClientGoalState& state, const vigir_generic_params::GetAllParameterSetsResultConstPtr& result)
{
  // remove all
  this->clear();

  for(int i = 0; i < result->param_sets.size() ; i++)
  {
    QString name = QString::fromStdString(result->param_sets[i].name.data);
    this->addItem(name);
  }

  // if no parameter sets available set default:
  if(this->count() == 0)
  {
    this->addItem("default");
    this->setCurrentIndex(0);
  }
}

}// End namespace vigir_footstep_planning_rviz_plugin
