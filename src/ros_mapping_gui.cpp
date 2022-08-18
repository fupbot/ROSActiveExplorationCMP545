//v1.4 - Working HIMM and Bayes
//@fupbot - Aug/22

#include "ros_mapping_gui.h"
#include "ui_ros_mapping_gui.h"

//global variables and general program parameters
double x_pos;                       //robot x coordinate
double y_pos;                       //robot y coordinate
double yaw_rot;                     //robot rotation angle
int img_side = 500;                 //to alter img size, change here and on the declaration of world below
Obstacle world[500][500]{};         //world definition
double scale_factor = 20.0;         //scale factor
int grid_size = 5;                  //square pixels of minimum grid size
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

          if (world[i][j].prob_occ > float(0.8)){
            value =  QColor("black").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].prob_occ > float(0.65) &&
                   world[i][j].prob_occ <= float(0.8)) {
            value =  QColor("darkGray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].prob_occ > float(0.55) &&
                   world[i][j].prob_occ <= float(0.65)) {
            value =  QColor("gray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].prob_occ <= float(0.45)){
            value =  QColor("white").rgba();
            image.setPixel(i, j, value);
          }
          else{
            value =  QColor("lightGray").rgba();
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
            value =  QColor("black").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].himm_occ > 2 &&
                   world[i][j].himm_occ < 7){
            value =  QColor("darkGray").rgba();
            image.setPixel(i, j, value);
          }
          else if (world[i][j].himm_occ > -1 &&
                   world[i][j].himm_occ < 3){
            value =  QColor("gray").rgba();
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

  //function to mark occupied cells in the world
  markOccupied();
};

//function to mark on world the occupied cells
void  RosMappingGUI::markOccupied(){
  //robot initialization offset is at the center of picture (0,0) becomes ((img_side/2),(img_side/2))
  //marks as occupied the cells whose probability is greater than 0.5
  for (int i = 0; i < img_side; i++){                                //x coordinate of image
    for (int j = 0; j < img_side; j++){                               //y coordinate of image

      //variables to mark the occupancy grid that contains the pixel
      int lower_x = int(floor(i/grid_size)*grid_size);        //get the grid square boundaries that the
      int upper_x = lower_x + grid_size;                      //occupied pixel is i
      int lower_y = int(floor(j/grid_size)*grid_size);        //get the grid square boundaries
      int upper_y = lower_y + grid_size;

      if(world[i][j].occupied == true){
        //bayes
        if((world[i][j].prob_occ > world[lower_x][lower_y].prob_occ) ||
           (world[i][j].prob_occ < world[lower_x][lower_y].prob_occ) ||
           (world[i][j].prob_occ > world[upper_x][upper_y].prob_occ) ||
           (world[i][j].prob_occ < world[upper_x][upper_y].prob_occ)){             //if square is already even, dont do anything

            if(world[lower_x + grid_size/2][lower_y + grid_size/2].prob_occ < float(0.49) ||
               world[lower_x + grid_size/2][lower_y + grid_size/2].prob_occ > float(0.51)){
              for (int k = lower_x; k < upper_x; k++){
                for (int l = lower_y; l < upper_y; l++){
                  world[k][l].prob_occ = world[lower_x + grid_size/2][lower_y + grid_size/2].prob_occ;
                  world[k][l].occupied = true;
                }
              }
            }
      }
        //himm
        if((world[i][j].himm_occ != world[lower_x][lower_y].himm_occ) ||
           (world[i][j].himm_occ != world[upper_x-1][upper_y-1].himm_occ)){
            if(world[i][j].himm_occ != 0){
              for (int k = lower_x; k < upper_x; k++){
                for (int l = lower_y; l < upper_y; l++){
                  world[k][l].himm_occ = world[i][j].himm_occ;
                  world[k][l].occupied = true;
                }
              }
            }
        }
      }
    }
  }


    //fill in the gaps

    for (int i = int(grid_size*1.5); i < int(img_side - grid_size*1.5); i+=grid_size){                //x coordinate of image
      for (int j = int(grid_size*1.5); j < int(img_side - grid_size*1.5); j+=grid_size){              //y coordinate of image

      //variables to mark the occupancy grid that contains the pixel
      int prev_x = int(i - grid_size);              //get the grid square boundaries that the
      int post_x = int(i + grid_size);              //occupied pixel is i
      int prev_y = int(j - grid_size);              //get the grid square boundaries
      int post_y = int(j + grid_size);

      float mean_free = 0.0;
      //float mean_free_himm = 0.0;

      if(world[i][j].occupied == 0){

        int count_frees = 0;

        for (int m = prev_x; m <= post_x; m+=grid_size){
          for (int n = prev_y; n <= post_y; n+=grid_size){
            if(world[m][n].prob_occ < float(0.5) || world[m][n].himm_occ < 0){
              count_frees++;
              mean_free += world[m][n].prob_occ;
            }
          }
        }
        mean_free = mean_free/count_frees;

        //free space -- if most of neighbouring cells are free, then set as free
        if (count_frees > 4){
            for (int k = i - grid_size/2; k < i + grid_size/2; k++){
              for (int l = j - grid_size/2; l < j + grid_size/2; l++){
                world[k][l].himm_occ = -1;
                world[k][l].prob_occ = mean_free;
                world[k][l].occupied = true;
              }
            }
        }
      }

      bool  up_down = false;
      float mean_up_down = 0.0;
      if (world[prev_x][prev_y].prob_occ > float(0.5) && world[post_x][post_y].prob_occ > float(0.5)){
        up_down = true;
        mean_up_down = (world[prev_x][prev_y].prob_occ + world[post_x][post_y].prob_occ)/2;
      }

      bool down_up = false;
      float mean_down_up = 0.0;
      if (world[prev_x][post_y].prob_occ > float(0.5) && world[post_x][prev_y].prob_occ > float(0.5)){
        down_up = true;
        mean_down_up = (world[prev_x][post_y].prob_occ + world[post_x][prev_y].prob_occ)/2;
      }
      //occupied space
      else if (up_down || down_up){
        for (int k = i - grid_size/2; k < i + grid_size/2; k++){
          for (int l = j - grid_size/2; l < j + grid_size/2; l++){
            if (up_down)
              world[k][l].prob_occ = mean_up_down;
            else
              world[k][l].prob_occ = mean_down_up;

            world[k][l].occupied = true;
          }
        }
      }
    }
  }

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

//function to convert coordinates between frames of reference
std::pair<int, int> RosMappingGUI::convertCoord(double scale, double angle, double trans_x, double trans_y, double x, double y){
  //angle in radians
  //trans_x and trans_y are translation distance in x and y
  //x_pos and y_pos are robots coordinates

  //rotate
  double x_rotated = (x * cos(angle)) -
                     (y * sin(angle));
  double y_rotated = (x * sin(angle)) +
                     (y * cos(angle));

  //scale and translate
  int x_new = int(((x_rotated  + x_pos)*scale) + trans_x);
  int y_new = int((-(y_rotated + y_pos)*scale) + trans_y);

  return std::make_pair(x_new, y_new);
}

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
  //if the reading is valid
  if(fabs(alpha) < beta){                                           //only gets values that are within cone of readings                                                          //only gets readings that are under 4 meters

    //convert pair of coordinates to WORLD coordinates
    double yaw_2f = yaw_rot - sonar_angles[which_sonar];   //angle sonar to world

    //convert r and R to WORLD dimensions
    double r_world = r*scale_factor;
    double R_world = R*scale_factor;

    //find how squares fit along r
    int num_cells = int(r_world/grid_size);

    //coordinates of previous visited cell to check whether it has already been visited or not
    int x_prev = 0;
    int y_prev = 0;

    //loop every cell for every degree
    for(int j = 0; j < num_cells+1; j++){
      int radius = (j*grid_size) + (grid_size/2);

        //loop every degree in the cone of readings
        for(int i = -15; i < 16; i++){

          //iterate from middle to right (CW), then from middle to left (CCW)
          int res = 0;
          if((i+15) < 16)      //0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 -1 -2 -3 -4 -5 -6 -7 -8 -9 -10 -11 -12 -13 -14 -15
            res = i+15;
          else
            res = -i;

          //angle of partial reading
          double alpha_temp = (res*(3.1415/180));

          //ang of reference for each iteration
          double ang_ref = yaw_2f + alpha_temp;

          double x_ref = (radius*cos(ang_ref)) +  (x_pos*scale_factor + img_side/2);
          double y_ref = -(radius*sin(ang_ref)) + (-y_pos*scale_factor + img_side/2);

          int mid_x = int(floor(x_ref/grid_size)*grid_size  + grid_size/2);        //get the grid square boundaries that the
          int mid_y = int(floor(y_ref/grid_size)*grid_size + grid_size/2);        //get the grid square boundaries


           //if readings are out of map's bounds
          if(mid_x > 500 || mid_y > 500 || mid_x < 0 || mid_y < 0)
            break;

          //previous value for prob occupied
          float previous_occ = world[mid_x][mid_y].prob_occ;

          //if the cell has not been visited yet
          if (x_prev != mid_x && y_prev != mid_y){

            //BAYES region II - not occupied
            if (radius < (r_world - grid_size)){

              float previous_emp = 1 - previous_occ;                                                              //previous value for prob empty
              double prob_s_emp = 0.5 * (((R_world-radius)/R_world) + ((beta - fabs(alpha_temp))/beta)) * max;    //prob of reading S given it is occupied
              double prob_s_occ = 1 - prob_s_emp;                                                                //prob of reading S given it is empty

              double prob_occ_s = (prob_s_occ * double(previous_occ))/                                           //prob of being occupied given reading S
                                  ((prob_s_occ* double(previous_occ)) +
                                  (prob_s_emp * double(previous_emp)));

              //only takes in probabilities that are lower that the previous one for region II
              if(prob_occ_s < double(previous_occ)){
                world[mid_x][mid_y].occupied = true;
                world[mid_x][mid_y].prob_occ = float(prob_occ_s);
              }

            }

            //BAYES region I - occupied
            else if(radius > (r_world - grid_size) && radius < (r_world + grid_size)){

              float previous_emp = 1 - previous_occ;                                                                  //previous value for prob empty
              double prob_s_occ = 0.5 * (((R_world-radius)/R_world) + ((beta - fabs(alpha_temp))/beta)) * max;        //prob of reading S given it is occupied
              double prob_s_emp = 1 - prob_s_occ;                                                                     //prob of reading S given it is empty

              double prob_occ_s = (prob_s_occ * double(previous_occ))/                                           //prob of being occupied given reading S
                                  ((prob_s_occ* double(previous_occ)) +
                                  (prob_s_emp * double(previous_emp)));


              world[mid_x][mid_y].occupied = true;
              world[mid_x][mid_y].prob_occ = float(prob_occ_s);

            }
            else{ }

            //mark as visited
            x_prev = mid_x;
            y_prev = mid_y;
          }

      }
    }
  }
};

//HIMM probability CALCULATOR
void RosMappingGUI::HIMMProb(int which_sonar){
  //only start execution if HIMM button is pressed
  if (btn_BAYES == 1 && btn_HIMM == 0) return;

  //beta (maximum angle of sonar reading)
  double beta = 0.174533;     //equivalent to 10 degrees

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

  //calculate HIMM values
  //if the reading is valid
  if(fabs(alpha) < beta){                                           //only gets values that are within cone of readings                                                          //only gets readings that are under 4 meters

    //convert pair of coordinates to WORLD coordinates
    double yaw_2f = yaw_rot - sonar_angles[which_sonar];   //angle sonar to world

    //convert r to WORLD dimensions
    double r_world = r*scale_factor;

    //find how squares fit along r
    int num_cells = int(r_world/grid_size);

    //coordinates of previous visited cell to check whether it has already been visited or not
    int x_prev = 0;
    int y_prev = 0;

    //loop every cell for every degree
    for(int j = 0; j < num_cells+1; j++){
      int radius = (j*grid_size) + (grid_size/2);

        //loop every degree in the cone of readings
        for(int i = -10; i < 11; i++){

          //iterate from middle to right (CW), then from middle to left (CCW)
          int res = 0;
          if((i+10) < 11)      //0 1 2 3 4 5 6 7 8 9 10  -1 -2 -3 -4 -5 -6 -7 -8 -9 -10
            res = i+10;
          else
            res = -i;

          //angle of partial reading
          double alpha_temp = (res*(3.1415/180));

          //ang of reference for each iteration
          double ang_ref = yaw_2f + alpha_temp;

          //std::cout << res << "ang ref " << res*3.1415/180 << "\n";

          double x_ref = (radius*cos(ang_ref)) +  (x_pos*scale_factor + img_side/2);
          double y_ref = -(radius*sin(ang_ref)) + (-y_pos*scale_factor + img_side/2);

          int mid_x = int(floor(x_ref/grid_size)*grid_size  + grid_size/2);        //get the grid square boundaries that the
          int mid_y = int(floor(y_ref/grid_size)*grid_size + grid_size/2);         //get the grid square boundaries


           //if readings are out of map's bounds
          if(mid_x > 500 || mid_y > 500 || mid_x < 0 || mid_y < 0)
            break;

          //previous value for prob occupied
          int previous_occ = world[mid_x][mid_y].himm_occ;

          //boundary values
          //max = 15
          //min = -1

          //if the cell has not been visited yet
          if (x_prev != mid_x && y_prev != mid_y){

            //FREE
            if (radius < (r_world - grid_size) || r > 4.5){
              //mark occupied
              world[mid_x][mid_y].occupied = true;
              //LOWER boundary value
              if(previous_occ - 1 < -1)
                world[mid_x][mid_y].himm_occ = -1;
              else
                world[mid_x][mid_y].himm_occ = previous_occ - 1;
            }

            //OCCUPIED
            else if(radius > (r_world - grid_size) && radius < (r_world + grid_size)){
              //mark occupied
              world[mid_x][mid_y].occupied = true;
              //UPPER boundary value
              if(previous_occ + 3 > 15)
                world[mid_x][mid_y].himm_occ = 15;
              else
                world[mid_x][mid_y].himm_occ = previous_occ + 3;
            }
            else{}

            //mark as visited
            x_prev = mid_x;
            y_prev = mid_y;
          }

      }
    }
  }
}
