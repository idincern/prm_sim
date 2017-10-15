#include <cstdio>
#include <iostream>
#include <cstdlib>

#include "globalmap.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat hallway(void){
  //Contains two 'unknown' (grey) areas on the map
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(125, 125, 125));
  cv::rectangle(image, cv::Point(20,20),cv::Point(60, 200),cv::Scalar(255,255,255),-1);
  cv::rectangle(image, cv::Point(60,20),cv::Point(200,100),cv::Scalar(255,255,255),-1);

  return image;
}

int main() {
  cv::Mat map = hallway();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  GlobalMap g(20.0, 0.1, 0.2);
  TGlobalOrd robot{10, 10}, start{4, 2};
  g.setReference(robot);

  std::vector<TGlobalOrd> path = g.build(map, start, {4, 5});


  cv::namedWindow("Building network", CV_WINDOW_NORMAL);
  g.showOverlay(colourMap, path);
  cv::imshow("Building network", colourMap);
  cv::waitKey(5000);

  path = g.build(map, start, {4, 10});

  g.showOverlay(colourMap, path);
  cv::imshow("Building network", colourMap);
  cv::waitKey(1000);

  path = g.build(map, start, {4, 15});

  g.showOverlay(colourMap, path);
  cv::imshow("Building network", colourMap);
  cv::waitKey(1000);

  path = g.build(map, start, {10, 15});

  g.showOverlay(colourMap, path);
  cv::imshow("Building network", colourMap);
  cv::waitKey(1000);

  path = g.build(map, start, {19, 15});

  g.showOverlay(colourMap, path);
  cv::imshow("Building network", colourMap);
  cv::waitKey(1000);

}



