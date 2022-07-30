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
#include <QGraphicsPathItem>
#include <QHBoxLayout>
#include <QListWidget>
#include <QGraphicsView>
#include <QtCore>
#include <QPainter>
#include <QImage>
#include <ros/ros.h>
#include <qtimer.h>
#include <math.h>
#include <tuple>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>


//struct to store occupation data
struct Obstacle {
  bool occupied;      //whether it is occupied or not
  float prob_occ;     //occupation probability - increased upon revisits
};

//global variables (x,y) and rotation (w)
extern double x_pos;
extern double y_pos;
extern double yaw_rot;
extern Obstacle world[500][500];  //world or map
extern int img_side;              //side of image or scene - always square
extern double scale_factor;
extern int grid_size;

namespace Ui {
class RosMappingGUI;
}

class RosMappingGUI : public QWidget
{
  Q_OBJECT

public:
  explicit RosMappingGUI(QWidget *parent = nullptr);
  void   posePosition(const nav_msgs::Odometry::ConstPtr& msg);           //function for odometry readings
  std::tuple<double, double, double> getPose(const nav_msgs::Odometry::ConstPtr &msg);
  void   sonarObstacles(const sensor_msgs::PointCloudConstPtr& son);       //function for sonar readinds


  //print map on the scene
  void generateMap();
  ~RosMappingGUI();

public slots:
  void spinOnce();

private:
  Ui::RosMappingGUI *ui;

  QTimer *ros_timer;
  ros::NodeHandlePtr nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber sonar_sub_;

  QGraphicsScene *scene;
  QGraphicsRectItem *rectangle;
  QGraphicsEllipseItem *ellipse;
  QGraphicsPixmapItem *arrow;
  QGraphicsPolygonItem *triangle;


  //map image
  QImage *map;
};

#endif // ROS_MAPPING_GUI_H
