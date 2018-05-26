#ifndef PLANNINGWIDGET_H
#define PLANNINGWIDGET_H

#include <QWidget>
#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_planning_widget.h"
#include <vigir_footstep_planning_rviz_plugin/planning_request_handler.h>
#include <vigir_footstep_planning_rviz_plugin/widget_base.h>

namespace rviz
{
class Config;
}

namespace vigir_footstep_planning_rviz_plugin
{
// Handels user interface for planning
class PlanningWidget : public WidgetBase
{
    Q_OBJECT
public:
    explicit PlanningWidget(QWidget *parent = 0);
    ~PlanningWidget();
    Ui::PlanningWidget *ui;
    PlanningRequestHandler* request_handler_;
    // save and load ui
    void save( rviz::Config config ) const;
    void load( const rviz::Config& config );
    // set request setting from display
    void setFrameID(const QString frame_id);


private Q_SLOTS:
    void on_planningComputeCommandLinkButton_clicked();
    void setParameterSet(QString parameter_set_name);

public Q_SLOTS:
// invoked from Display
    void setCurrentStepPlan(vigir_footstep_planning_msgs::StepPlan step_plan); // update step plan if computed in other request handler
    void startPoseRequested();
    void handleGoalPose(Ogre::Vector3 position, Ogre::Quaternion orientation);
    void setFeedbackRequested(bool requested);
// invoked from Foot Visuals
    void setLastStep(int last_index);
    void replanToIndex(int index);
    // invoked from panel
    void resetValues();
    void abort();
// emit signals
    void emitFeetToolActivated(bool activated);

Q_SIGNALS:
  void feetToolActivated(bool active);
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PLANNINGFORM_H
