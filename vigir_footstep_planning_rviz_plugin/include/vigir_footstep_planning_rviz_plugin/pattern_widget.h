#ifndef PATTERNWIDGET_H
#define PATTERNWIDGET_H

#include "../../../../../build/vigir_footstep_planning_rviz_plugin/ui_pattern_planning.h"
#include <vigir_footstep_planning_rviz_plugin/pattern_request_handler.h>
#include <QWidget>

namespace rviz
{
class Config;
}

namespace vigir_footstep_planning_rviz_plugin
{
// Handels user interface for pattern planning
class PatternWidget : public QWidget
{
    Q_OBJECT
public:
    explicit PatternWidget(QWidget *parent = 0);
    ~PatternWidget();
    Ui::PatternWidget *ui;
    PatternRequestHandler* request_handler_;

    void initialize();

    // save and load ui
    void save( rviz::Config config ) const;
    void load( const rviz::Config& config );
    // set request setting from display
    void setFrameID(const QString frame_id);


public Q_SLOTS:
    // invoked from panel
    void abort();
    //invoked from display
    void startPoseRequested();
    //invoked from StepVisual
    void setLastStep(int last_index);
    void setParameterSet(QString parameter_set_name);

Q_SIGNALS:
    void changed();

private Q_SLOTS:
    void on_patternForwardPushButton_clicked();
    void on_patternBackwardPushButton_clicked();
    void on_patternRotateLeftPushButton_clicked();
    void on_patternRotateRightPushButton_clicked();
    void on_patternStrafeLeftPushButton_clicked();
    void on_patternStrafeRightPushButton_clicked();
    void on_patternStepUpPushButton_clicked();
    void on_patternStepOverPushButton_clicked();
    void on_patternStepDownPushButton_clicked();
    void on_patternWideStancePushButton_clicked();
    void on_patternCenterLeftPushButton_clicked();
    void on_patternCenterFeetPushButton_clicked();
    void on_patternCenterRightPushButton_clicked();
    void on_patternSamplingPushButton_clicked();
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // PLANNINGFORM_H
