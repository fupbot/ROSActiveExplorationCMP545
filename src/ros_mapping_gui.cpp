#include "ros_mapping_gui.h"
#include "ui_ros_mapping_gui.h"

//global variables and general program parameters
double x_pos;                       //robot x coordinate
double y_pos;                       //robot y coordinate
double yaw_rot;                     //robot rotation angle
int img_side = 500;                 //to alter img size, change here and on the declaration of world below
Obstacle world[500][500]{};         //world definition
double scale_factor = 20.0;         //scale factor
int grid_size = 6;                  //square pixels of minimum grid size
double sonar_readings[8][2]{};      //array to store sonar readings in the point cloud format - 8 (x,y) readings
bool btn_BAYES = 0;
bool btn_HIMM = 0;

//---------------------------------------------------------------------------------------
//Widget configurations
RosMappingGUI::RosMappingGUI(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::RosMappingGUI)
{
  ui->setupUi(this);

  //buttons config
  //QPushButton *pbtn_BAYES = new QPushButton(this);
  QObject::connect(ui->pbtn_BAYES, &QPushButton::clicked, this, &RosMappingGUI::onBayesButtonClicked);
  //QPushButton *pbtn_HIMM = new QPushButton(this);
  QObject::connect(ui->pbtn_HIMM, &QPushButton::clicked, this, &RosMappingGUI::onHIMMButtonClicked);

  //initialize world struct
  for (int i=0;i<img_side;++i) {                          //iterate x
    for (int j=0;j<img_side;++j) {                        //iterate y
      world[i][j].occupied = 0;
      world[i][j].prob_occ = 0.5;
      world[i][j].himm_occ = 0;
    }
  }

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

//ros SPIN ONCE function
void RosMappingGUI::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
   }
  else {
    QApplication::quit();
  }
}

//function to check whether BAYES button is pressed
void RosMappingGUI::onBayesButtonClicked(){
  btn_BAYES = 1;
  btn_HIMM = 0;
  ui->pbtn_HIMM->setChecked(0);
};

//function to check whether HIMM button is pressed
void RosMappingGUI::onHIMMButtonClicked(){
  btn_HIMM = 1;
  btn_BAYES = 0;
  ui->pbtn_BAYES->setChecked(0);
};

//function to print robot on scene
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

  //iterates X image to plot obstacles
  for (int i=0;i<img_side;++i) {                          //iterate x
    //iterates Y image to plot obstacles
    for (int j=0;j<img_side;++j) {                        //iterate y

      //BAYES printing of cells
      if(btn_BAYES == 1 && btn_HIMM == 0){
        //if it is occupied, then paint the square black
        if (world[i][j].occupied == 1){
          //std::cout << world[i][j].prob_occ << "\n";

          if (world[i][j].prob_occ > float(0.8)){
            value =  QColor("black").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].prob_occ > float(0.65) &&
                   world[i][j].prob_occ <= float(0.8)) {
            value =  QColor("darkGray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].prob_occ > float(0.5) &&
                   world[i][j].prob_occ <= float(0.65)) {
            value =  QColor("gray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].prob_occ <= float(0.5)){
            value =  QColor("white").rgba();
            image.setPixel(i, j, value);
          }
         }

         else{
            value = QColor("lightGray").rgba();
            image.setPixel(i, j, value);
          }
        }

      //HIMM printing
      else{
        //if it is occupied, then paint the square black
        if (world[i][j].occupied == 1){

          if(world[i][j].himm_occ > 10){
            value =  QColor("black").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].himm_occ > 6 &&
                   world[i][j].himm_occ < 11){
            value =  QColor("darkGray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].himm_occ > 2 &&
                   world[i][j].himm_occ < 7){
            value =  QColor("gray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].himm_occ > -1 &&
                   world[i][j].himm_occ < 3){
            value =  QColor("lightGray").rgba();
            image.setPixel(i, j, value);
          }
          else{
            value =  QColor("white").rgba();
            image.setPixel(i, j, value);
          }
        }
        else{
           value = QColor("lightGray").rgba();
           image.setPixel(i, j, value);
         }
      }
    }
  }


  //GRID lines
  for (int r=0; r<img_side; r+=grid_size){
    for (int t=0; t<img_side; t++){
      value =  QColor("white").rgba();
      image.setPixel(r, t, value);
    }
  }
  for (int r=0; r<img_side; r+=grid_size){
    for (int t=0; t<img_side; t++){
      value =  QColor("white").rgba();
      image.setPixel(t, r, value);
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


//odometry and rotation function
void RosMappingGUI::posePosition(const nav_msgs::Odometry::ConstPtr &msg){
  //only start execution if one of the buttons is pressed
  if (btn_BAYES == 0 && btn_HIMM == 0) return;

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

//sonar function
void RosMappingGUI::sonarObstacles(const sensor_msgs::PointCloudConstPtr& son){
  //only start execution if one of the buttons is pressed
  if (btn_BAYES == 0 && btn_HIMM == 0) return;

  //yaw rotation in degrees 0 to 360 CCW
  double ang_deg = yaw_rot * (180/3.1415);
  if (ang_deg < 0){
    ang_deg = ang_deg + 360;
  }

  //stores readings in x, y format
  for (unsigned long i=0; i<8; i++){
    sonar_readings[i][0] = double(son->points[i].x);
    sonar_readings[i][1] = double(son->points[i].y);
  }
 /*
    //double dist = sqrt(pow(son->points[i].x,2) + pow(son->points[i].y,2));     //euclid dist

    //if euclid distance less than 2m, store that value
    //if (dist < 2.0){

      //y readings are inverted in order to be transfered to image coordinates
//      double x_rotated = (son->points[i].x * cos(ang_deg*(3.1415/180))) - (son->points[i].y * sin(ang_deg*(3.1415/180)));  //rotation conversion
//      double y_rotated = (son->points[i].x * sin(ang_deg*(3.1415/180))) + (son->points[i].y * cos(ang_deg*(3.1415/180)));

      //rotation, scaling and translation
//      sonar_readings[i][0] = scale_factor*x_rotated + scale_factor*x_pos + (img_side/2);
//      sonar_readings[i][1] = -scale_factor*y_rotated - scale_factor*y_pos + (img_side/2);
//    }
//    else, store as zero
//    else {
//      sonar_readings[i][0] = 0.0;
//      sonar_readings[i][1] = 0.0;
//    }
//  }

  //robot initialization offset is at the center of picture (0,0) becomes ((img_side/2),(img_side/2))
  //renders world
  for (int i = 0; i < img_side; i++){     //x coordinate of image
    for (int j=0; j < img_side; j++){     //y coordinate of image
      for (int k=0; k < 8; k++){
        if (int(sonar_readings[k][0]) == i &&
            int(sonar_readings[k][1]) == j){

          //variables to mark the occupancy grid that contains the pixel
          int lower_x = int(floor(i/grid_size)*grid_size);        //get the grid square boundaries that the
          int upper_x = lower_x + grid_size-1;                    //occupied pixel is i
          int lower_y = int(floor(j/grid_size)*grid_size);        //get the grid square boundaries
          int upper_y = lower_y + grid_size-1;

          if (world[upper_x][upper_y].occupied == 0){           //if it has not visited the square before
            for (int k = lower_x; k < upper_x+1; k++){
              for (int l = lower_y; l < upper_y+1; l++){
                world[k][l].occupied = 1;
              }
            }
          }

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
*/

  //function to mark occupied cells in the world
  markOccupied();
};

//function to mark on world the occupied cells
void  RosMappingGUI::markOccupied(){
  //robot initialization offset is at the center of picture (0,0) becomes ((img_side/2),(img_side/2))
  //marks as occupied the cells whose probability is greater than 0.5
  for (int i = 0; i < img_side; i++){                               //x coordinate of image
    for (int j=0; j < img_side; j++){                               //y coordinate of image

      //variables to mark the occupancy grid that contains the pixel
      int lower_x = int(floor(i/grid_size)*grid_size);        //get the grid square boundaries that the
      int upper_x = lower_x + grid_size-1;                    //occupied pixel is i
      int lower_y = int(floor(j/grid_size)*grid_size);        //get the grid square boundaries
      int upper_y = lower_y + grid_size-1;

      //variable to store highest probability within the cell
      float sum_obs   = float(0.0);
      float sum_frees = float(0.0);
      int count_obs   = 0;
      int count_frees = 0;


      //BAYES marking of cells
      if(btn_BAYES == 1 && btn_HIMM == 0){
        //if it has been visited the square before
        if((double(world[i][j].prob_occ) < 0.49 ||
            double(world[i][j].prob_occ) > 0.51)){

          for (int k = lower_x; k < upper_x+1; k++){
            for (int l = lower_y; l < upper_y+1; l++){
              if(double(world[k][l].prob_occ) < 0.49){                //points lower than obstacle
                sum_frees = sum_frees + world[k][l].prob_occ;
                count_frees++;
              }
              else if (double(world[k][l].prob_occ) > 0.51){          //points higher than obstacle
                sum_obs = sum_obs + world[k][l].prob_occ;
                count_obs++;
              }
              world[k][l].occupied = 1;                                      //occupied actually means visited
            }
          }

          //mean probability of grid cell that is not 0.5
          float mean_prob = 0.0;
          if ( count_obs > count_frees ){                   //if any pixel is considered an obstacle -- maybe expand pixels?
            mean_prob = sum_obs/count_obs;
          }
          else {
            mean_prob = sum_frees/count_frees;
          }


          for (int k = lower_x; k < upper_x+1; k++){       //makes every point in cell the same value as the mean
            for (int l = lower_y; l < upper_y+1; l++){
              world[k][l].prob_occ = mean_prob;
            }
          }
        }

        //else if(world[i][j].occupied == 1){
        //else if(world[i][j].prob_occ < float(0.49))
          //world[i][j].occupied = 1;
        //}
        else{
          world[i][j].occupied = 0;                //otherwise, mark as free (0)
        }
      }

      //HIMM marking of cells
      else{

        //halt readings at -1
        if (world[i][j].himm_occ < -1){
          world[i][j].himm_occ = -1;
        }
        else if (world[i][j].himm_occ > 15){                 //latch obstacles as it is
          for (int k = lower_x; k < upper_x+1; k++){
            for (int l = lower_y; l < upper_y+1; l++){
              world[k][l].himm_occ = 15;
            }
          }
        }
        else{


        int sum_HIMM   = 0;
        int count_HIMM = 0;

        //int count_HIMM_neg = 0;
        //if it is occupied
//        if(world[i][j].himm_occ != 0 &&
//           world[i][j].occupied == 0){    //only goes in cell once

        if(world[i][j].himm_occ != 0 &&
           world[i][j].occupied == 0){    //goes in cell once for the first time

          //std::cout << world[i][j].himm_occ << "\n";

          //GROWTH RATE OPERATOR
//          if (world[i][j].himm_occ > -1){

//          world[i][j].himm_occ = 3;
//          world[i][j].himm_occ = int((world[i-1][j-1].himm_occ * 0.5) + (world[i][j-1].himm_occ * 0.5) + (world[i+1][j-1].himm_occ * 0.5) +
//                                     (world[i-1][j].himm_occ * 0.5)   + (world[i][j].himm_occ * 1.0)   + (world[i+1][j].himm_occ * 0.5) +
//                                     (world[i-1][j+1].himm_occ * 0.5) + (world[i][j+1].himm_occ * 0.5) + (world[i+1][j+1].himm_occ * 0.5));
//          }

          //loop grid cell

          //THIS PART IS CAUSING THE CRASH
          // |
          // |
          // v
          for (int k = lower_x; k < upper_x+1; k++){
            for (int l = lower_y; l < upper_y+1; l++){
              world[k][l].occupied = 1;

              //world[k][l].himm_occ = world[i][j].himm_occ;             //LEAVING ONLY THIS LINE SORT OF WORKS

              if(world[k][l].himm_occ != 0){
                count_HIMM++;                                       //count all positive readings
                sum_HIMM = sum_HIMM + world[k][l].himm_occ;
              }
              //              else{
              //                count_HIMM = 1;                                     //if not positive, mark as free
              //                sum_HIMM = 0;
              //              }

            }
          }

          //          //makes every value in cell the mean value
          //          //int temp = int((sum_HIMM - count_HIMM_neg)/(count_HIMM + count_HIMM_neg));

          int temp = int(sum_HIMM/count_HIMM);

          for (int k = lower_x; k < upper_x+1; k++){
            for (int l = lower_y; l < upper_y+1; l++){
              world[k][l].himm_occ = temp;
            }
          }
        }
        else if(world[i][j].himm_occ != 0 &&
                world[i][j].occupied == 1){

          if (world[i][j].himm_occ == world[lower_x][lower_y].himm_occ &&    //if it has already been averaged within the cell
              world[i][j].himm_occ == world[upper_x][upper_y].himm_occ){}
          else{

            int sum_HIMM   = 0;
            int count_HIMM = 0;

            for (int k = lower_x; k < upper_x+1; k++){
              for (int l = lower_y; l < upper_y+1; l++){
                world[k][l].occupied = 1;

                if(world[k][l].himm_occ != 0){
                  count_HIMM++;                                       //count all positive readings
                  sum_HIMM = sum_HIMM + world[k][l].himm_occ;
                }

              }
            }

            int temp = int(sum_HIMM/count_HIMM);

            for (int k = lower_x; k < upper_x+1; k++){
              for (int l = lower_y; l < upper_y+1; l++){
                world[k][l].himm_occ = temp;
              }
            }
          }
        }
        else{
          //world[i][j].occupied = 0;
        }
      }
     }
    }
  }

      /*
      for (int k=0; k < 8; k++){
        if (int(sonar_readings[k][0]) == i &&
            int(sonar_readings[k][1]) == j){

          //variables to mark the occupancy grid that contains the pixel
          int lower_x = int(floor(i/grid_size)*grid_size);        //get the grid square boundaries that the
          int upper_x = lower_x + grid_size-1;                    //occupied pixel is i
          int lower_y = int(floor(j/grid_size)*grid_size);        //get the grid square boundaries
          int upper_y = lower_y + grid_size-1;

          if (world[upper_x][upper_y].occupied == 0){             //if it has not visited the square before
            for (int k = lower_x; k < upper_x+1; k++){
              for (int l = lower_y; l < upper_y+1; l++){
                world[k][l].occupied = 1;
              }
            }
          }

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
  */
  //calls method to calculate probabilty for each sensor
  for (int i = 0; i < 8; i++){
    if (btn_BAYES == 1 && btn_HIMM == 0){
      bayesProb(i);
    }
    else{
      HIMMProb(i);
    }
  }

  //call function to generate map
  generateMap();
};


//BAYES PROBABILITY CALCULATOR
void RosMappingGUI::bayesProb(int which_sonar){
  //only start execution if BAYES button is pressed
  if (btn_BAYES == 0 && btn_HIMM == 1) return;

  //maximum value in order not to hit 100% probability
  double max = 0.98;
  //big R in bayes equation (maximum sonar reading distance)
  double R = 5.18;
  //beta in bayes equation (maximum angle of sonar reading)
  double beta = 0.261799;     //equivalent to 15 degrees

/*
  //sonar readings are in the following angles according to robot frame of reference:
  //sonar_readings[0][] = +90   -> +1.5708
  //sonar_readings[1][] = +50   -> +0.872665
  //sonar_readings[2][] = +30   -> +0.523599
  //sonar_readings[3][] = +10   -> +0.174533
  //sonar_readings[4][] = +350  -> -0.174533
  //sonar_readings[5][] = +330  -> -0.523599
  //sonar_readings[6][] = +310  -> -0.872665
  //sonar_readings[7][] = +270  -> -1.5708
*/

  //inverse angles of each sensor in radians relative to robot frame of reference
  double sonar_angles[8] = {-1.5708, -0.872665, -0.523599, -0.174533, 0.174533, 0.523599, 0.872665, 1.5708};

  //rotate reading to the specified sonar frame of reference
  //brings all sonar readings from robot's (x,y) to their own reference ---> acoustic axis = x axis
  double x_rot = (sonar_readings[which_sonar][0] * cos(sonar_angles[which_sonar])) -
                 (sonar_readings[which_sonar][1] * sin(sonar_angles[which_sonar]));
  double y_rot = (sonar_readings[which_sonar][0] * sin(sonar_angles[which_sonar])) +
                 (sonar_readings[which_sonar][1] * cos(sonar_angles[which_sonar]));

  //variables in bayes equation
  double r = sqrt(pow(x_rot, 2) + pow(y_rot, 2));
  double alpha = atan2(y_rot, x_rot);

  //calculate bayes probability
  if(fabs(alpha) < beta  &&                   //only gets values that are within cone of readings
     r < 4.0){                                //only gets readings that are under 4 meters

    //loop through rectangular region of interest
    for (int i = int(ceil(x_rot - fabs(r*cos(alpha)))); i < int(ceil(x_rot)+1); i++){
      for (int j = int(ceil(-fabs(r*sin(alpha)))); j < int(ceil(fabs(r*sin(alpha))+1)); j++){

        //get previous probability reading from world model
        //angle to rotate back to robot frame and then to world frame
        double yaw_2f = yaw_rot - sonar_angles[which_sonar];

        //rotate
        double x_prev = (i * cos(yaw_2f)) -
                        (j * sin(yaw_2f));
        double y_prev = (i * sin(yaw_2f)) +
                        (j * cos(yaw_2f));

        //scale and translate
        int x_previous = int(((x_prev + x_pos)*scale_factor) + (img_side/2));
        int y_previous = int((-(y_prev + y_pos)*scale_factor) + (img_side/2));

        //previous value for prob occupied
        float previous_occ = world[x_previous][y_previous].prob_occ;

        //---------------------------------------------------------
        //region I -  possible true locations of sonar readings
        //region II - locations that are know to not have obstacles

        //narrow down to work only on sonar frame of reference
        if (fabs(atan2(j, i)) < beta){

          //region I - assumed sonar reading error of 5cm
          if ((i > x_rot - 0.05) && (i < x_rot + 0.05)){
            float previous_emp = 1 - previous_occ;                                      //previous value for prob empty
            double prob_s_occ = 0.5 * (((R-r)/R) + ((beta - alpha)/beta)) * max;        //prob of reading S given it is occupied
            double prob_s_emp = 1 - prob_s_occ;                                         //prob of reading S given it is empty

            double prob_occ_s = (prob_s_occ * double(previous_occ))/                    //prob of being occupied given reading S
                                ((prob_s_occ* double(previous_occ)) +
                                (prob_s_emp * double(previous_emp)));

            //Update world.prob_occ struct
            double x_prob = (i * cos(yaw_2f)) -                                      //rotate
                            (j * sin(yaw_2f));
            double y_prob = (i * sin(yaw_2f)) +
                            (j * cos(yaw_2f));

            int x_post = int(((x_prob + x_pos)*scale_factor) +  (img_side/2));       //scale and translate
            int y_post = int((-(y_prob + y_pos)*scale_factor) + (img_side/2));

            world[x_post][y_post].prob_occ = float(prob_occ_s);
          }

          //region II
          else if (i < x_rot - 0.05){

            float previous_emp = 1 - previous_occ;                                      //previous value for prob empty
            double prob_s_emp = 0.5 * (((R-r)/R) + ((beta - alpha)/beta)) * max;        //prob of reading S given it is occupied
            double prob_s_occ = 1 - prob_s_emp;                                         //prob of reading S given it is empty

            double prob_occ_s = (prob_s_occ * double(previous_occ))/                    //prob of being occupied given reading S
                                ((prob_s_occ* double(previous_occ)) +
                                (prob_s_emp * double(previous_emp)));

            //Update world.prob_occ struct
            double x_prob = (i * cos(yaw_2f)) -                                      //rotate
                            (j * sin(yaw_2f));
            double y_prob = (i * sin(yaw_2f)) +
                            (j * cos(yaw_2f));

            int x_post = int(((x_prob + x_pos)*scale_factor) +  (img_side/2));       //scale and translate
            int y_post = int((-(y_prob + y_pos)*scale_factor) + (img_side/2));

            world[x_post][y_post].prob_occ = float(prob_occ_s);
          }

          //beyond reading
          else{} //do nothing
        }
      }
    }
  }
};

//HIMM probability CALCULATOR
void RosMappingGUI::HIMMProb(int which_sonar){
  //only start execution if HIMM button is pressed
  if (btn_BAYES == 1 && btn_HIMM == 0) return;

  //inverse angles of each sensor in radians relative to robot frame of reference
  double sonar_angles[8] = {-1.5708, -0.872665, -0.523599, -0.174533, 0.174533, 0.523599, 0.872665, 1.5708};

  //rotate reading to the specified sonar frame of reference
  //brings all sonar readings from robot's (x,y) to their own reference ---> acoustic axis = x axis
  double x_rot = (sonar_readings[which_sonar][0] * cos(sonar_angles[which_sonar])) -
                 (sonar_readings[which_sonar][1] * sin(sonar_angles[which_sonar]));
  double y_rot = (sonar_readings[which_sonar][0] * sin(sonar_angles[which_sonar])) +
                 (sonar_readings[which_sonar][1] * cos(sonar_angles[which_sonar]));

  //calculates deviation from acoustic axis
  double alpha = atan2(y_rot, x_rot);
  //calculates distance from origin
  double r = sqrt(pow(x_rot, 2) + pow(y_rot, 2));

  //if the selected cell is within 5 degress of acoustic axis
  if (fabs(alpha) < 5*(3.1415/180)){

    //update point that contains reading

    //loop through rectangular region of interest
    for (int i = int(ceil(x_rot - fabs(r*cos(alpha)))); i < int(ceil(x_rot)+1); i++){
      for (int j = int(ceil(-fabs(r*sin(alpha)))); j < int(ceil(fabs(r*sin(alpha))+1)); j++){

        //angle to rotate back to robot frame and then to world frame
        double yaw_2f = yaw_rot - sonar_angles[which_sonar];

        //rotate
        double x_prev = (i * cos(yaw_2f)) -
                        (j * sin(yaw_2f));
        double y_prev = (i * sin(yaw_2f)) +
                        (j * cos(yaw_2f));

        //scale and translate
        int x_himm = int(((x_prev + x_pos)*scale_factor) + (img_side/2));
        int y_himm = int((-(y_prev + y_pos)*scale_factor) + (img_side/2));


        //previous value for himm occupation criteria
        int previous_himm = world[x_himm][y_himm].himm_occ;

        //std::cout << "At x = " << x_himm << " and y = " << y_himm << "\n";

        //if it is at the point being read by the sonar
        //if(i == int(x_rot) && j == int(y_rot)){

        if((i > x_rot - 0.05) && (i < x_rot + 0.05) &&
           (j > y_rot - 0.02) && (j < y_rot + 0.02)){
          for (int i = x_himm - 2; i < x_himm + 3; i++){
            for (int j = y_himm - 2; j < y_himm + 3; j++){
              world[i][j].himm_occ = previous_himm + 3;           //increases and inflates pixel
            }
          }
          //world[x_himm][y_himm].himm_occ = previous_himm + 3;   //increase
          //debug
          //std::cout << "Increasing... " << previous_himm + 3 << "\n";
        }
        else if((j > y_rot - 0.02) && (j < y_rot + 0.02) &&
                (i < x_rot - 0.05)){

          for (int i = x_himm - 2; i < x_himm + 3; i++){
            for (int j = y_himm - 2; j < y_himm + 3; j++){
              world[i][j].himm_occ = previous_himm - 1;           //decreases and inflates pixel
            }
          }
          //world[x_himm][y_himm].himm_occ = previous_himm - 1;   //decrease
          //std::cout << "Decreasing... " << previous_himm - 1 << "\n";
        }
        else{}
      }
    }

  }
  //if it not within the cone of readings
  else{}

}


