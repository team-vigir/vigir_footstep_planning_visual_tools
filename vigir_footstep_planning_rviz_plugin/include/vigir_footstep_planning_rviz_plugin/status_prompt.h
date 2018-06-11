#ifndef STATUS_PROMPT_H
#define STATUS_PROMPT_H

#ifndef Q_MOC_RUN

#include <QTextEdit>
#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#endif

namespace vigir_footstep_planning_rviz_plugin
{
class StatusPrompt : public QTextEdit
{
    Q_OBJECT
public:
    StatusPrompt(QWidget *parent = 0);
    ~StatusPrompt();

    void initialize();

public Q_SLOTS:
    void displayConnection(QString action_server_name, bool success);

private Q_SLOTS:
    void displayMessage(QString message);
    void displayError(QString message);
    void displaySuccess(QString message);
    void displayWarn(QString message);
};

} // end namespace vigir_footstep_planning_rviz_plugin
#endif // STATUS_PROMPT_H
