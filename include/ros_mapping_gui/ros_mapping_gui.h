#ifndef ROS_MAPPING_GUI_H
#define ROS_MAPPING_GUI_H

#include <QWidget>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QtGui>
#include <QDialog>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsPathItem>
#include <QProgressBar>
#include <QCursor>
#include <QMouseEvent>
#include <QCheckBox>
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
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>


//struct to store occupation data
struct Obstacle {
  float prob_occ;     //occupation probability - increased upon revisits
  int himm_occ;       //himm occupation probability
  bool occupied;      //whether it is occupied or not
  bool obst_goal;     //if it is goal or obstacle, do not change potential
  bool path_cell;     //to determine whether it is a cell that was in the path of robot
  bool unused_var;    //just to keep off the annoying padding warning
  float harm_pot;     //harmonic potential
  float angle_pot;    //angle to plot field
};

//global variables and general program parameters
extern double x_pos;
extern double y_pos;
extern double yaw_rot;
extern Obstacle world[800][800];  //world or map
extern int img_side;              //side of image or scene - always square
extern double scale_factor;
extern int grid_size;
extern double sonar_readings[8][2];      //array to store sonar readings in the point cloud format - 8 (x,y) readings
extern bool btn_BAYES;
extern bool btn_HIMM;
extern bool btn_GOAL;
extern std::pair<int,int> goal;
extern bool pot_calculated;
extern bool field_calculated;
extern bool show_field;
extern bool btn_GS;
extern bool btn_SOR;
extern bool sensors_on_off;
extern bool goal_cross_mov;
extern bool first_movement;
extern bool exp_goal_reached;

//namespace
namespace Ui {
class RosMappingGUI;
}

class RosMappingGUI : public QWidget
{
  Q_OBJECT

public:
  explicit RosMappingGUI(QWidget *parent = nullptr);
  void  posePosition(const nav_msgs::Odometry::ConstPtr& msg);           //function for odometry readings
  void  sonarObstacles(const sensor_msgs::PointCloudConstPtr& son);      //function for sonar readinds
  void  bayesProb(int which_sonar);                                      //function for bayes probability of obstacles
  void  HIMMProb(int which_sonar);                                       //function for HIMM probability of obstacles
  void  markOccupied();
  std::pair<int, int> convertCoord(double scale, double angle, double trans_x, double trans_y, double x, double y);
  void calcHarmonicPot();
  void navGoal();
  void exploreWorld();
  const double ROBOT_FOOTPRINT_RADIUS = 0.40;
  const double ROBOT_SONAR_ERROR      = 0.05;

  //print map on the scene
  void generateMap();
  ~RosMappingGUI() override;

public slots:
  void spinOnce();
  void onBayesButtonClicked();
  void onHIMMButtonClicked();
  void onGoalButtonClicked();
  void onNavButtonClicked();
  void onGOButtonClicked();
  void onFieldCheckboxClicked();
  void onGSButtonClicked();
  void onSORButtonClicked();
  void onEXPButtonClicked();
  void onTrajectoryCheckboxClicked();

protected:
  void mousePressEvent(QMouseEvent *event) override;

private:
  Ui::RosMappingGUI *ui;

  //ros variables
  QTimer *ros_timer;
  ros::NodeHandlePtr nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber sonar_sub_;
  ros::Publisher  pub_topic_;

  //scene variables
  QGraphicsScene       *scene;
  //QGraphicsRectItem    *rectangle;
  //QGraphicsEllipseItem *ellipse;
  //QGraphicsPixmapItem  *arrow;
  QPolygonF            *Triangle;
  QGraphicsPathItem    *block;
  QPolygonF            *Goal_Cross;
  QGraphicsPathItem    *cross;

  //button variabless
  QPushButton  *pbtn_BAYES;
  QPushButton  *pbtn_HIMM;
  QCheckBox    *checkBox;
  QProgressBar *progressBar;

  //map image
  QImage *map_img;
};

#endif // ROS_MAPPING_GUI_H
