#ifndef PROPERTY_WIDGET_H
#define PROPERTY_WIDGET_H

#include <qtreewidget.h>
#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

namespace vigir_footstep_planning_rviz_plugin
{
class PropertyWidget : public QTreeWidget
{
    Q_OBJECT
public:
    PropertyWidget(QWidget *parent = 0);
    ~PropertyWidget();

    void initialize();

    void addStep(const unsigned int& index);
    void addStep(const vigir_footstep_planning_msgs::Step& index);

public Q_SLOTS:
    void displayStepPlanProperties(vigir_footstep_planning_msgs::StepPlan step_plan);

private:
    void addProperty(QTreeWidgetItem *parent, const QString& property_name, const QString& property_value);

};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PROPERTY_WIDGET_H
