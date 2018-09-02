#ifndef PARAMETER_SET_COMBO_BOX_H
#define PARAMETER_SET_COMBO_BOX_H

#ifndef Q_MOC_RUN

#include <qcombobox.h>
#include <actionlib/client/simple_action_client.h>
#include <vigir_generic_params/generic_params_msgs.h>

#endif

typedef actionlib::SimpleActionClient<vigir_generic_params::GetAllParameterSetsAction > GetAllParameterSetsActionClient;

namespace vigir_footstep_planning_rviz_plugin
{
class ParameterSetComboBox : public QComboBox
{
    Q_OBJECT
public:
    ParameterSetComboBox(QWidget *parent = 0);
    ~ParameterSetComboBox();
    void initialize();

Q_SIGNALS:
    void actionClientConnected(QString name, bool connected);

public Q_SLOTS:
    void updateParameterSetNames();

private:
    GetAllParameterSetsActionClient parameter_set_ac;
    void getParameterSetsCallback(const actionlib::SimpleClientGoalState& state, const vigir_generic_params::GetAllParameterSetsResultConstPtr& result);

};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PARMETER_SET_COMBO_BOX_H
