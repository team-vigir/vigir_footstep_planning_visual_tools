#include <vigir_footstep_planning_rviz_plugin/property_widget.h>


typedef vigir_footstep_planning_msgs::Step StepMsg;
typedef vigir_footstep_planning_msgs::StepPlan StepPlanMsg;

namespace vigir_footstep_planning_rviz_plugin {

PropertyWidget::PropertyWidget(QWidget *parent) :
    QTreeWidget(parent)
{
  initialize();
}

PropertyWidget::~PropertyWidget()
{

}

void PropertyWidget::initialize()
{
  this->setColumnCount(2);
  QStringList labels;
  labels << "Step" << "Properties";
  setHeaderLabels(labels);
}

void PropertyWidget::addProperty(QTreeWidgetItem *parent, const QString& property_name, const QString& property_value)
{
  QTreeWidgetItem *property_item = new QTreeWidgetItem(parent);
  property_item->setText(0, property_name);
  property_item->setText(1, property_value);
  // QTreeWidgetItem::addChild(QTreeWidgetItem * child)
  parent->addChild(property_item);
}

void PropertyWidget::addStep(const StepMsg& step)
{


  QTreeWidgetItem *step_item = new QTreeWidgetItem(this);
  step_item->setText(0, "Step " + QString::number(step.step_index));

  QBrush brush_red(Qt::red);
  if(!step.valid || step.colliding)
    step_item->setBackground(0, brush_red);

  addProperty(step_item, "Cost", QString::number(step.cost));
  addProperty(step_item, "Risk", QString::number(step.risk));
  addProperty(step_item, "Valid", step.valid ? "YES" : "NO (not reached from predecessor)");
  addProperty(step_item, "Colliding", step.colliding ? "YES" : "NO");
}

void PropertyWidget::displayStepPlanProperties(StepPlanMsg step_plan)
{
  // todo: remove old properties
  clear();
  for(unsigned int i = 0; i < step_plan.steps.size(); ++i)
  {
    addStep(step_plan.steps[i]);
  }


}

}// End namespace vigir_footstep_planning_rviz_plugin
