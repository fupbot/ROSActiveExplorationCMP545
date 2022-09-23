# ROS Active Exploration App
Application developed during the 'Mobile Robotics' master's level course at INF-UFRGS. <br>
As the contents of the course are taught, the topics were put into practice in this application. <br>
It begins as a simple trajectory plotter and ends as an active exploration module. <br>
The methodology used derives from my advisor's phD thesis entitled "Navegação exploratória baseada em problemas de valores de contorno" which uses harmonic potential fields for navigation and exploration, and it is available at https://bit.ly/3BE1Car <br>
Videos and images of application can be seen at 'media' folder in this repository <br><br>
--------------- <br>
GUI:   QT 4.9.2 with ROS plugin. <br>
OS:    ROS1 Noetic running ROSAria package and Ubuntu 20.04 <br>
Robot: Pioneer 3DX and simulator Pioneer <br>
--------------- <br><br>
Version history: <br>

v1.0 - Connects to /fupbot/nav_pack/ ROS navigator in a GUI showing trajectory and occupancy grid. <br>
v1.1 - Creates occupancy grid for obstacles based on sonar data and shows robot at correct location and orientation. <br>
v1.2 - Improved rendering of occupancy grid. <br>
v1.3 - Partially working BAYES and HIMM methods. <br>
v1.4 - Working BAYES and HIMM obstacle detection methods. <br><br>

<img src="https://github.com/fupbot/Simple_ROS_mapper/blob/main/media/bayes_vs_himm.png" width="600">

<br><br>
v1.5 - (Almost) working version of Harmonic Field navigation. <br>
v1.6 - Working harmonic fields. <br>
v1.7 - Working exploration mode. <br><br>

<img src="https://github.com/fupbot/Simple_ROS_mapper/blob/main/media/exp_v1_7.png" width="600">
<br>
<br>
v1.8 - Improved exploration mode with local activation window. <br>
v1.9 - Improved navigation. Pending vector speed control <br>
v2.0 - Working active exploration. Final project for conclusion of 'Mobile Robotics Course'. <br><br>

<img src="https://github.com/fupbot/Simple_ROS_mapper/blob/main/media/mapa_denso.gif" width="600">
<br><br>


