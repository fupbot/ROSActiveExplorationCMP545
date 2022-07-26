#include "ros_mapping_gui.h"
#include "ui_ros_mapping_gui.h"

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
  ros_timer->start(500);  // set the rate to 1000ms

  // setup subscriber
  std::string listen_topic;
  nh_->param<std::string>("listen_topic",listen_topic,"/RosAria/pose");
  pose_sub_ = nh_->subscribe<nav_msgs::Odometry>(listen_topic, 1, &RosMappingGUI::posePosition, this);


  //GraphicsView config ---------------------------------
  scene = new QGraphicsScene(this);
  ui->mapa->setScene(scene);
  scene->setSceneRect(0,0,600,600);

  QBrush redBrush(Qt::red);
  QBrush blueBrush(Qt::blue);
  QBrush greenBrush(Qt::green);
  QPen blackpen(Qt::black);
  QPen redpen(Qt::red);
  blackpen.setWidth(1);

  //Creates GRID
  // add vertical lines - color GRAY
  for (int x=0; x<=600; x+=10)
    scene->addLine(x,0,x,600, QPen(Qt::gray));

  // add GRAY horizontal lines
  for (int y=0; y<=600; y+=10)
      scene->addLine(0,y,600,y, QPen(Qt::gray));

  //triangle to represent the robot
  QPolygonF Triangle;
  Triangle.append(QPointF(-10.,0));
  Triangle.append(QPointF(0.,-30));
  Triangle.append(QPointF(10,0));
  Triangle.append(QPointF(-10.,0));

  rectangle = scene->addRect(300,300,10,10,blackpen,redBrush);
  ellipse = scene->addEllipse(300, 300, 3, 3, redpen, redBrush);
  triangle = scene->addPolygon(Triangle, blackpen, greenBrush);

  rectangle->setFlags(QGraphicsItem::ItemIsMovable);
  ellipse->setFlags(QGraphicsItem::ItemIsMovable);
  triangle->setFlags(QGraphicsItem::ItemIsMovable);
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


//main function that is called every time there is an event in ROS
void RosMappingGUI::posePosition(const nav_msgs::Odometry::ConstPtr &msg){
  auto qstring_x = QString::fromStdString(std::to_string(msg->pose.pose.position.x));
  auto qstring_y = QString::fromStdString(std::to_string(msg->pose.pose.position.y));
  ui->x_pos->setText(qstring_x);
  ui->y_pos->setText(qstring_y);

  //robot representing the robot
  float x = ui->x_pos->text().toFloat();
  float y = ui->y_pos->text().toFloat();
  //rectangle->setPos(x*10, -y*10);

  //brushes and pens
  QBrush redBrush(Qt::red);
  QBrush blueBrush(Qt::blue);
  QBrush blackBrush(Qt::black);
  QPen blackpen(Qt::black);
  QPen redpen(Qt::red);
  blackpen.setWidth(1);

  //position history
  int x_flat = 0;
  int y_flat = 0;

  if ((x - std::floor(x)) > 0.5 && (y - std::floor(y)) > 0.5 ){
    x_flat = std::ceil(x)*10;
    y_flat = std::ceil(y)*10;
  }
  else if ((x - std::floor(x)) < 0.5 && (y - std::floor(y)) > 0.5 ){
    x_flat = std::floor(x)*10;
    y_flat = std::ceil(y)*10;
  }
  else if ((x - std::floor(x)) > 0.5 && (y - std::floor(y)) < 0.5 ){
    x_flat = std::ceil(x)*10;
    y_flat = std::floor(y)*10;
  }
  else {
    x_flat = std::floor(x)*10;
    y_flat = std::floor(y)*10;
  }

  ui->x_flat_lbl->setText(QString::number(x_flat));
  ui->y_flat_lbl->setText(QString::number(y_flat));
  scene->addRect(300+(x_flat),300-(y_flat),10,10,blackpen,blackBrush);


  //robot
  triangle->setPos(300+ x*10, 300 -y*10);
  scene->addEllipse(300+ x*10, 300 -y*10,5,5, redpen, redBrush);

  //get rotation
  float theta_quat = 0.0;
  float theta_deg = 0.0;
  auto qstring_w = QString::fromStdString(std::to_string(msg->pose.pose.orientation.w));
  ui->quat->setText(qstring_w);
  theta_quat = ui->quat->text().toFloat();

  float temp = 0.0;
  //temp = (180/3.1415)*acos(theta_quat)*2;
  temp = (180/3.1415)*atan(theta_quat);
  theta_deg = temp;


  auto qstring_deg = QString::fromStdString(std::to_string(temp));
  ui->angle->setText(qstring_deg);

  triangle->setRotation(theta_deg);
}
