/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include <fstream>

int main(int argc, char **argv) {
  
  // output the NIS info for both laser and radar measurements
  /*
  laserfile << "car_0 , car_1 , car_2" << std::endl;
  ofstream radarfile("../output/radar_NIS.csv");
  radarfile << "car_0 , car_1 , car_2" << std::endl;
  */

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  float x_pos = 0;
  viewer->setCameraPosition(x_pos - 26, 0, 15.0, x_pos + 25, 0, 0, 0, 0, 1);

  Highway highway(viewer);

  // initHighway(viewer);

  int frame_per_sec = 30;
  int sec_interval = 10;
  int frame_count = 0;
  int time_us = 0;

  double egoVelocity = 25;

  while (frame_count < (frame_per_sec * sec_interval)) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
    highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer);

    /*
    laserfile << highway.traffic[0].ukf.NIS_laser_ << " ,"
              << highway.traffic[1].ukf.NIS_laser_ << " ,"
              << highway.traffic[2].ukf.NIS_laser_ << std::endl;
    radarfile << highway.traffic[0].ukf.NIS_radar_ << " ,"
              << highway.traffic[1].ukf.NIS_radar_ << " ,"
              << highway.traffic[2].ukf.NIS_radar_ << std::endl;
    */

    viewer->spinOnce(1000 / frame_per_sec);
    frame_count++;
    time_us = 1000000 * frame_count / frame_per_sec;
  }

  /*
  laserfile.close();
  radarfile.close();
  */
 
}