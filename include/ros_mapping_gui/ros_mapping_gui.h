#ifndef ROS_MAPPING_GUI_H
#define ROS_MAPPING_GUI_H

#include <QWidget>
#include <QtGui>
#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsPixmapItem>
#include <QtCore>
#include <QPainter>
#include <ros/ros.h>
#include <qtimer.h>
#include <math.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

namespace Ui {
class RosMappingGUI;
}

class RosMappingGUI : public QWidget
{
  Q_OBJECT

public:
  explicit RosMappingGUI(QWidget *parent = nullptr);
  void posePosition(const nav_msgs::Odometry::ConstPtr& msg);
  ~RosMappingGUI();

public slots:
  void spinOnce();

private:
  Ui::RosMappingGUI *ui;

  QTimer *ros_timer;
  ros::NodeHandlePtr nh_;
  ros::Subscriber pose_sub_;

  QGraphicsScene *scene;
  QGraphicsRectItem *rectangle;
  QGraphicsEllipseItem *ellipse;
  QGraphicsPixmapItem *arrow;
  QGraphicsPolygonItem *triangle;
};

#endif // ROS_MAPPING_GUI_H
