#ifndef PLANNINGWIDGET_H
#define PLANNINGWIDGET_H

#include <QWidget>
#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_planning_widget.h"
#include <vigir_footstep_planning_rviz_plugin/planning_request_handler.h>

namespace rviz
{
class Config;
}

namespace vigir_footstep_planning_rviz_plugin
{
// Handels user interface for planning
class PlanningWidget : public QWidget
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
    void initialize();

private Q_SLOTS:
    void on_planningComputeCommandLinkButton_clicked();

public Q_SLOTS:
// invoked from Display
    void startPoseRequested();
    void updateGoalPose(vigir_footstep_planning_msgs::Feet goal_pose);
    void setFeedbackRequested(bool requested);
// invoked from Foot Visuals
    void setLastStep(int last_index);
    void replanToIndex(int index);
    // invoked from panel
    void abort();
    void setParameterSet(QString parameter_set_name);

Q_SIGNALS:
  void setGoalActivated(bool active);
  void changed();
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PLANNINGFORM_H
