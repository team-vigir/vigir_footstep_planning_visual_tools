#ifndef WIDGET_BASE_H
#define WIDGET_BASE_H

#include <QWidget>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <vigir_footstep_planning_msgs/PlanningFeedback.h>
#include <vigir_footstep_planning_msgs/StepPlan.h>
#include <vigir_footstep_planning_rviz_plugin/planning_request_handler.h>

namespace vigir_footstep_planning_rviz_plugin
{

//class RequestHandlerBase;


class WidgetBase : public QWidget
{
    Q_OBJECT

public:
    WidgetBase(QWidget *parent = 0);
    ~WidgetBase();
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // WIDGET_BASE_H
