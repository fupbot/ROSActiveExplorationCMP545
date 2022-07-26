#include <QApplication>
#include <QIcon>
#include "ros_mapping_gui.h"

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "ros_mapping_gui");
  QApplication a(argc, argv);

  RosMappingGUI w;

  // set the window title as the node name
  w.setWindowTitle(QString::fromStdString(
                      ros::this_node::getName()));

  //icone
  QIcon icon(":/icons/robot.svg");
  w.setWindowIcon(icon);

  w.show();
  return a.exec();
}
