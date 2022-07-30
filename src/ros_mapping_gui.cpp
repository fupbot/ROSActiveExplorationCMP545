#include "ros_mapping_gui.h"
#include "ui_ros_mapping_gui.h"

//global variables (x,y) and rotation (w)
double x_pos;
double y_pos;
double yaw_rot;
int img_side = 500;                 //to alter img size, change here and on the declaration of world below
Obstacle world[500][500]{};         //world
double scale_factor = 20.0;         //scale factor
int grid_size = 6;                 //square pixels of minimum grid size

RosMappingGUI::RosMappingGUI(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::RosMappingGUI)
{
  ui->setupUi(this);

  //ROS config ------------------------------------------
  nh_.reset(new ros::NodeHandle("~"));

  // setup the timer for ros to make things happen
  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 1000ms

  // setup subscriber
  std::string listen_topic, listen_topic2;
  nh_->param<std::string>("listen_topic",listen_topic,"/RosAria/pose");
  pose_sub_  = nh_->subscribe<nav_msgs::Odometry>(listen_topic, 1, &RosMappingGUI::posePosition, this);
  nh_->param<std::string>("listen_topic2", listen_topic2, "/RosAria/sonar");
  sonar_sub_ = nh_->subscribe<sensor_msgs::PointCloud>(listen_topic2, 1, &RosMappingGUI::sonarObstacles, this);

  //GraphicsView config ---------------------------------
  scene = new QGraphicsScene(this);
  ui->mapa->setScene(scene);
}

RosMappingGUI::~RosMappingGUI()
{
  delete ui;
}

void RosMappingGUI::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
   }
  else {
    QApplication::quit();
  }
}

static QPixmap QPixmapFromItem(QGraphicsItem *item){
    QPixmap pixmap(item->boundingRect().size().toSize());
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    QStyleOptionGraphicsItem opt;
    item->paint(&painter, &opt);
    return pixmap;
}

//function to draw map
void RosMappingGUI::generateMap(){
  QImage image(img_side, img_side, QImage::Format_RGB32);
  QRgb value;
  QListWidget listWidget;

  //iterates X image to plot obstacles
  for (int i=0;i<img_side;++i) {                          //iterate x
    //iterates Y image to plot obstacles
    for (int j=0;j<img_side;++j) {                        //iterate y

      //if it is occupied, then paint the square black
      if (world[i][j].occupied == 1){

        int lower_x = int(floor(i/grid_size)*grid_size);      //get the grid square boundaries that the
        int upper_x = lower_x + grid_size;                    //occupied pixel is i
        int lower_y = int(floor(j/grid_size)*grid_size);    //get the grid square boundaries
        int upper_y = lower_y + grid_size;

        for (int k = lower_x; k < upper_x+1; k++){
          for (int l = lower_y; l < upper_y+1; l++){
            value =  QColor("black").rgba();
            image.setPixel(k, l, value);
           }
         }
       }

       else{
        value = QColor("white").rgba();
        image.setPixel(i, j, value);
       }
     }
   }


  //GRID lines - only plots once
  if (image.pixel(0,0) == 4278190080){            //if first pixel is of white color
    for (int r=0; r<img_side; r+=grid_size){
      for (int t=0; t<img_side; t++){
        value =  QColor("lightGray").rgba();
        image.setPixel(r, t, value);
      }
    }
    for (int r=0; r<img_side; r+=grid_size){
      for (int t=0; t<img_side; t++){
        value =  QColor("lightGray").rgba();
        image.setPixel(t, r, value);
      }
    }
  }

  //creates world pixmap
  scene->addPixmap(QPixmap::fromImage(image));


  //robot triangle
  QPolygonF Triangle;
  Triangle.append(QPointF(-5.,0));
  Triangle.append(QPointF(0.,-15));
  Triangle.append(QPointF(5.,0));
  Triangle.append(QPointF(-5.,0));

  double ang_deg = yaw_rot * (180/3.1415);
  if (ang_deg < 0){
    ang_deg = ang_deg + 360;
  }

  //updates robot position and rotation
  QPainterPath p;
  p.addPolygon(Triangle);
  QGraphicsPathItem *block = scene->addPath(p);
  block->setBrush(QBrush("green"));
  block->setFlag(QGraphicsItem::ItemIsMovable);
  block->setFlag(QGraphicsItem::ItemIsSelectable);
  block->setX(x_pos*scale_factor + (img_side/2));
  block->setY(-y_pos*scale_factor + (img_side/2));
  block->setRotation(-ang_deg+90);

  //add robot triangle to scene
  QPixmap robot = QPixmapFromItem(block);
  scene->addPixmap(robot);
}



//odometry function
void RosMappingGUI::posePosition(const nav_msgs::Odometry::ConstPtr &msg){
  //set global variables
  x_pos = msg->pose.pose.position.x;
  y_pos = msg->pose.pose.position.y;

  //convert quaternion to RPY
  tf::Quaternion q(
         msg->pose.pose.orientation.x,
         msg->pose.pose.orientation.y,
         msg->pose.pose.orientation.z,
         msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   yaw_rot = yaw;
}

/* I really tried not to use global variables.
//values to be used in next function
 std::tuple<double, double, double> RosMappingGUI::getPose(const nav_msgs::Odometry::ConstPtr &msg){
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double w = msg->pose.pose.orientation.w;

  return {x, y, w};   //returns tuple
}
*/

//sonar function
void RosMappingGUI::sonarObstacles(const sensor_msgs::PointCloudConstPtr& son){
  //yaw rotation in degrees 0 to 360 CCW
  double ang_deg = yaw_rot * (180/3.1415);
  if (ang_deg < 0){
    ang_deg = ang_deg + 360;
  }

  //stores readings in x, y format. Calculates euclid dist, rotates and offsets point cloud.
  //values that yield euclidian distance less than 2m are positive doubles, others are zero.
  double sonar_readings[8][2]{};
  for (unsigned long i=0; i<8; i++){
    double dist = sqrt(pow(son->points[i].x,2) + pow(son->points[i].y,2));  //euclid dist

    //if euclid distance less than 2m, store that value
    if (dist < 2.0){
      //y readings are inverted in order to be transfered to image coordinate
      double x_rotated = (son->points[i].x * cos(ang_deg*(3.1415/180))) - (son->points[i].y * sin(ang_deg*(3.1415/180)));  //rotation conversion
      double y_rotated = (son->points[i].x * sin(ang_deg*(3.1415/180))) + (son->points[i].y * cos(ang_deg*(3.1415/180)));

      //rotation, scaling and translation
      sonar_readings[i][0] = scale_factor*x_rotated + scale_factor*x_pos + (img_side/2);
      sonar_readings[i][1] = -scale_factor*y_rotated - scale_factor*y_pos + (img_side/2);
    }
    //else, store as zero
    else {
      sonar_readings[i][0] = 0.0;
      sonar_readings[i][1] = 0.0;
    }
  }

  //robot initialization offset is at the center of picture (0,0) becomes ((img_side/2),(img_side/2))
  //renders world
  for (int i = 0; i < img_side; i++){     //x coordinate of image
    for (int j=0; j < img_side; j++){     //y coordinate of image
      for (int k=0; k < 8; k++){
        if (int(sonar_readings[k][0]) == i &&
            int(sonar_readings[k][1]) == j){
          world[i][j].occupied = 1;                //if reading is valid, mark obstacle
        }
        else if(world[i][j].occupied == 1){
          world[i][j].occupied = 1;                //if there is already an obstacle, do nothing
        }
        else{
          world[i][j].occupied = 0;                //otherwise, mark as free (0)
        }
      }
    }
  }

  //calls method to plot map
  generateMap();
};
