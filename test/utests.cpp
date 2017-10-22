#include "gtest/gtest.h"
#include "../src/types.h"
#include "../src/localmap.h"
#include "../src/graph.h"
#include "../src/prmplanner.h"

#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <utility>
#include <vector>

static bool ShowPrm = false;        //Default is false
static unsigned int MaxTries = 10;  //Default max amount of tries is 10 for prm gen

/* Various opencv images for testing */

cv::Mat blankMap(void){
  //This map is completely white
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));

  return image;
}

cv::Mat partionedMap(void){
  //This map has a horizontal black line through the middle
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::line(image,cv::Point(0,100),cv::Point(200,100),cv::Scalar(0,0,0),1);

  return image;
}

cv::Mat partionedMap2(void){
  //This map has two large rectangles
  double mapSize = 20.0, res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::rectangle(image, cv::Point(0,0),cv::Point(70, 180), cv::Scalar(125,125,125),-1);
  cv::rectangle(image, cv::Point(130,160),cv::Point(200, 200), cv::Scalar(125,125,125),-1);

  return image;
}

cv::Mat partionedMap3(void){
  //This map has a horizontal black line diagonally
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::line(image,cv::Point(0,0),cv::Point(200,200),cv::Scalar(0,0,0),1);

  return image;
}

cv::Mat unknownMap(void){
  //Contains two 'unknown' (grey) areas on the map
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::rectangle(image, cv::Point(10,10),cv::Point(70,75),cv::Scalar(125,125,125),-1);
  cv::rectangle(image, cv::Point(80,80),cv::Point(200,200),cv::Scalar(125,125,125),-1);

  return image;
}

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

cv::Mat pole(void){
  //Contains two 'unknown' (grey) areas on the map
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));
  cv::circle(image, cv::Point(100,100), 50, cv::Scalar(0,0,0), -1);

  return image;
}

cv::Mat passage(void){
  //Contains multiple boxes
  double mapSize = 20.0;
  double res = 0.1;
  int pixels = (int) mapSize / res;

  cv::Mat image(pixels, pixels, CV_8UC1, cv::Scalar(255, 255, 255));

  bool twoBoxes = true;

  for(int i = 0; i < image.rows; i+=10){
    if(twoBoxes){
      cv::rectangle(image, cv::Point(25, i),cv::Point(75, i+10),cv::Scalar(0,0,0),-1);
      cv::rectangle(image, cv::Point(125,i),cv::Point(175, i+10),cv::Scalar(0,0,0),-1);
    } else {
      cv::rectangle(image, cv::Point(0, i),cv::Point(20, i+10),cv::Scalar(0,0,0),-1);
      cv::rectangle(image, cv::Point(90,i),cv::Point(110, i+10),cv::Scalar(0,0,0),-1);
      cv::rectangle(image, cv::Point(190, i),cv::Point(200, i+10),cv::Scalar(0,0,0),-1);
    }

    twoBoxes != twoBoxes;
  }

  cv::rectangle(image, cv::Point(0, 20),cv::Point(100, 25),cv::Scalar(255, 255, 255),-1);
  cv::rectangle(image, cv::Point(100, 120),cv::Point(200, 125),cv::Scalar(255, 255, 255),-1);

  return image;
}

/* Tests for testing a configuration space for connections */

TEST(ConfigSpace, Expand){
  LocalMap l(20.0, 0.1);

  cv::Mat img = hallway();

  EXPECT_EQ(255, img.at<uchar>(21, 21));

  l.expandConfigSpace(img, 1);

  EXPECT_EQ(125, img.at<uchar>(21, 21));
}

TEST(ConfigSpace, ConnectInEmptyMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = blankMap();
  //Vertical
  ASSERT_TRUE(l.canConnect(img, cv::Point(100, 0), cv::Point(100, 200)));

  //Horizontal
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 100), cv::Point(200, 100)));

  //Diagonal
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 0), cv::Point(200, 200)));
}

TEST(ConfigSpace, ConnectInPartionedMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = partionedMap();
  //Vertical
  ASSERT_FALSE(l.canConnect(img, cv::Point(100, 0), cv::Point(100, 200)));

  //Horizontal
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 150), cv::Point(10, 150)));

  //Diagonal
  ASSERT_FALSE(l.canConnect(img, cv::Point(0, 0), cv::Point(200, 200)));

  //A Diagonal not intersecting the line
  ASSERT_TRUE(l.canConnect(img, cv::Point(110, 90), cv::Point(150, 50)));
}

TEST(ConfigSpace, ConnectInUnknownMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = partionedMap();
  //Vertical
  ASSERT_FALSE(l.canConnect(img, cv::Point(100, 0), cv::Point(100, 200)));

  //Horizontal -Two small points in known blank area of the map
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 50), cv::Point(200, 50)));

  //Diagonal
  ASSERT_FALSE(l.canConnect(img, cv::Point(0, 0), cv::Point(200, 200)));
}

TEST(ConfigSpace, ConnectOutsideMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = blankMap();
  ASSERT_FALSE(l.canConnect(img, cv::Point(-100, 200), cv::Point(100, 100)));
  ASSERT_FALSE(l.canConnect(img, cv::Point(100, 200), cv::Point(-100, -100)));
}

/* Tests for converting from TGlobalOrds to local OgMap points */

TEST(LocalMap, ConvertPositivePoints){
  LocalMap l(20.0, 0.1);

  TGlobalOrd ref = {10, 10};
  TGlobalOrd p1 = {5, 15}, p2 = {15, 15}, p3 = {5, 5}, p4 = {15, 5}, p5 = {10, -10};

  EXPECT_EQ(cv::Point(50, 50), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(150, 50), l.convertToPoint(ref, p2));
  EXPECT_EQ(cv::Point(50, 150), l.convertToPoint(ref, p3));
  EXPECT_EQ(cv::Point(150, 150), l.convertToPoint(ref, p4));
  EXPECT_EQ(cv::Point(100, 300), l.convertToPoint(ref, p5)); //Outside the map space
}

TEST(LocalMap, ConvertDecimalPoints){
  LocalMap l (20.0, 0.1);
  TGlobalOrd ref = {10, 10};
  TGlobalOrd p1 = {5.1, 15.2};
  TGlobalOrd p2 = {5.15, 15.23}; //This will round to 5.2, 15.2
  EXPECT_EQ(cv::Point(51, 48), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(52, 48), l.convertToPoint(ref, p2));

  LocalMap l2(20.0, 0.1);
  ref = {-10, -10};
  p1 = {-15.2, -5.1};
  EXPECT_EQ(cv::Point(48, 51), l.convertToPoint(ref, p1));
}

TEST(LocalMap, ConvertNegativePoints){
  LocalMap l(20.0, 0.1);

  TGlobalOrd ref = {-10, -10};
  TGlobalOrd p1 = {-15, -5}, p2 = {-5, -5}, p3 = {-15, -15}, p4 = {-5, -15};

  EXPECT_EQ(cv::Point(50, 50), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(150, 50), l.convertToPoint(ref, p2));
  EXPECT_EQ(cv::Point(50, 150), l.convertToPoint(ref, p3));
  EXPECT_EQ(cv::Point(150, 150), l.convertToPoint(ref, p4));
}

TEST(LocalMap, ConvertPointsOnLine){
  LocalMap l(20.0, 0.1);

  TGlobalOrd ref = {0, 0};
  TGlobalOrd p1 = {0, 5}, p2 = {5, 0}, p3 = {-5, 0}, p4 = {0, -5};

  EXPECT_EQ(cv::Point(100, 50), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(150, 100), l.convertToPoint(ref, p2));
  EXPECT_EQ(cv::Point(50, 100), l.convertToPoint(ref, p3));
  EXPECT_EQ(cv::Point(100, 150), l.convertToPoint(ref, p4));
}

/* Tests for rendering on an OgMap (opencv image) */

TEST(LocalMap, RenderPRM){
  LocalMap l(20.0, 0.1);

  cv::Point p1(50, 50), p2(150, 50), p3(50, 150), p4(100, 300);
  cv::Mat m;
  cv::cvtColor(unknownMap(),m,CV_GRAY2RGB); //This must be a rgb image

  //Create map
  std::vector<std::pair<cv::Point, cv::Point>> prm;
  prm.push_back(std::make_pair(p1, p2));
  prm.push_back(std::make_pair(p1, p3));
  prm.push_back(std::make_pair(p1, p4));

  prm.push_back(std::make_pair(p2, p1));
  prm.push_back(std::make_pair(p2, p3));
  prm.push_back(std::make_pair(p2, p4));

  prm.push_back(std::make_pair(p3, p1));
  prm.push_back(std::make_pair(p3, p2));

  prm.push_back(std::make_pair(p4, p1));
  prm.push_back(std::make_pair(p4, p2));

  l.overlayPRM(m, prm);

  //Check that all points and lines have been rendered correctly...
  for(auto const &node: prm){
    if(l.inMap(node.first)){
      cv::Vec3b intensity = m.at<cv::Vec3b>(node.first);
      EXPECT_EQ(255, intensity.val[0]); //Blue nodes
      EXPECT_EQ(0, intensity.val[1]);
      EXPECT_EQ(0, intensity.val[2]);
    }
  }
}

TEST(LocalMap, RenderPath){
  LocalMap l(20.0, 0.1);

  cv::Point p1(50, 50), p2(150, 50), p3(50, 150), p4(100, 300);
  cv::Mat m;
  cv::cvtColor(unknownMap(),m,CV_GRAY2RGB); //This must be a rgb image

  //Create path
  std::vector<cv::Point> path = {p3, p1, p2};
  l.overlayPath(m, path);

  //Check that all points and lines have been rendered correctly...
  for(auto const &node: path){
    if(l.inMap(node)){
      cv::Vec3b intensity = m.at<cv::Vec3b>(node);
      EXPECT_EQ(0, intensity.val[0]); //Red nodes
      EXPECT_EQ(0, intensity.val[1]);
      EXPECT_EQ(255, intensity.val[2]);
    }
  }
}

/* Sanity tests for image generation using opencv */

TEST(ImageGen, CorrectDimensions){
  double mapSize=20.0;
  double resolution=0.1;

  int pixels = (int) mapSize / resolution;

  // Create a simple blank map
  cv::Mat image = blankMap();

  // Let's check map size compared to allocation, just in case
  EXPECT_EQ(pixels, image.rows);
  EXPECT_EQ(pixels, image.cols);

  // Let's check the map is blank
  EXPECT_EQ(255,image.at<uchar>(0,0));

  // Draw a link from top left to botom right corner
  cv::line(image,cv::Point(0,0),cv::Point(pixels,pixels),cv::Scalar(0,0,0),1);

  // Let's check the centre is now black (0)
  EXPECT_EQ(0, image.at<uchar>(pixels/2,pixels/2));
}

/* Probablistic road map generation tests */

TEST(PrmGen, SimplePath){
  cv::Mat map = partionedMap2();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  TGlobalOrd robot{10, 10}, goal{15, 15}, start{5,5};
  PrmPlanner g(20.0, 0.1);

  g.setReference(robot);
  g.expandConfigSpace(map, 0.2);

  std::vector<TGlobalOrd> path;

  int cnt(0);
  while(path.size() <= 0 && cnt < MaxTries){
    path = g.build(map, robot, goal);

    if(ShowPrm){
      g.showOverlay(colourMap, path);
      cv::imshow("test", colourMap);
      cv::waitKey(1000);
    }

    cnt++;
  }

  ASSERT_TRUE(path.size() > 0);
}

TEST(PrmGen, ComplicatedPath){
  //The start and goal locations are much more seperated out
  cv::Mat map = partionedMap2();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  TGlobalOrd robot{10, 10}, start{1, 1}, goal{10, 19};
  PrmPlanner g(20.0, 0.1);

  g.setReference(robot);
  g.expandConfigSpace(map, 0.2);

  std::vector<TGlobalOrd> path;

  int cnt(0);
  while(path.size() <= 0 && cnt < MaxTries){
    path = g.build(map, start, goal);

    if(ShowPrm){
      g.showOverlay(colourMap, path);
      cv::imshow("test", colourMap);
      cv::waitKey(1000);
    }

    cnt++;
  }

  ASSERT_TRUE(path.size() > 0);
}

TEST(PrmGen, Hallway){
  cv::Mat map = hallway();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  TGlobalOrd robot{10, 10}, start{4, 2}, goal{19, 14};
  PrmPlanner g(20.0, 0.1);

  g.setReference(robot);
  g.expandConfigSpace(map, 0.2);

  std::vector<TGlobalOrd> path;

  int cnt(0);
  while(path.size() <= 0 && cnt < MaxTries){
    path = g.build(map, start, goal);

    if(ShowPrm){
      g.showOverlay(colourMap, path);
      cv::imshow("test", colourMap);
      cv::waitKey(1000);
    }

    cnt++;
  }

  ASSERT_TRUE(path.size() > 0);
}

TEST(PrmGen, Pole){
  cv::Mat map = pole();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  TGlobalOrd robot{10, 10}, start{1, 1}, goal{19, 19};
  PrmPlanner g(20.0, 0.1);

  g.setReference(robot);
  g.expandConfigSpace(map, 0.2);

  std::vector<TGlobalOrd> path;

  int cnt(0);
  while(path.size() <= 0 && cnt < MaxTries){
    path = g.build(map, start, goal);

    if(ShowPrm){
      g.showOverlay(colourMap, path);
      cv::imshow("test", colourMap);
      cv::waitKey(1000);
    }

    cnt++;
  }

  ASSERT_TRUE(path.size() > 0);
}

TEST(PrmGen, Passage){
  cv::Mat map = passage();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  TGlobalOrd robot{10, 10}, start{1, 1}, goal{19, 19};
  PrmPlanner g(20.0, 0.1);

  g.setReference(robot);
  g.expandConfigSpace(map, 0.2);

  std::vector<TGlobalOrd> path;

  int cnt(0);
  while(path.size() <= 0 && cnt < MaxTries){
    path = g.build(map, start, goal);

    if(ShowPrm){
      g.showOverlay(colourMap, path);
      cv::imshow("test", colourMap);
      cv::waitKey(1000);
    }

    cnt++;
  }

  ASSERT_TRUE(path.size() > 0);
}

TEST(PrmGen, NoPath){
  //The start is in an unreachable section of the map
  cv::Mat map = partionedMap2();
  cv::Mat colourMap;
  cv::cvtColor(map, colourMap, CV_GRAY2BGR);

  PrmPlanner g(20.0, 0.1);
  TGlobalOrd robot{10, 10}, start{1, 5}, goal{10, 19};
  g.setReference(robot);
  g.expandConfigSpace(map, 0.2);

  std::vector<TGlobalOrd> path = g.build(map, start, goal);

  EXPECT_EQ(0, path.size());
}

/* Graph tests */
//The below tests are based on the graph examples found
//on the website: https://brilliant.org/wiki/dijkstras-short-path-finder/
TEST(Graph, FoundPath1){
  Graph g(7);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addVertex(3);
  g.addVertex(4);
  g.addVertex(5);
  g.addVertex(6);
  g.addVertex(7);
  g.addVertex(8);

  g.addEdge(0, 1, 3.0);
  g.addEdge(0, 2, 7.0);
  g.addEdge(0, 3, 5.0);
  g.addEdge(1, 4, 7.0);
  g.addEdge(1, 2, 1.0);
  g.addEdge(2, 3, 3.0);
  g.addEdge(2, 4, 2.0);
  g.addEdge(2, 5, 1.0);
  g.addEdge(2, 6, 3.0);
  g.addEdge(3, 6, 2.0);
  g.addEdge(4, 5, 2.0);
  g.addEdge(4, 7, 1.0);
  g.addEdge(5, 7, 3.0);
  g.addEdge(5, 6, 3.0);
  g.addEdge(5, 8, 2.0);
  g.addEdge(6, 8, 4.0);
  g.addEdge(7, 8, 5.0);

  std::vector<vertex> path = g.shortestPath(0, 8);

  EXPECT_EQ(0, path[0]);
  EXPECT_EQ(1, path[1]);
  EXPECT_EQ(2, path[2]);
  EXPECT_EQ(5, path[3]);
  EXPECT_EQ(8, path[4]);
}

TEST(Graph, FoundPath2){
  Graph g(7);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addVertex(3);
  g.addVertex(4);
  g.addVertex(5);
  g.addVertex(6);
  g.addVertex(7);
  g.addVertex(8);

  g.addEdge(0, 1, 3.0);
  g.addEdge(0, 2, 7.0);
  g.addEdge(0, 3, 5.0);
  g.addEdge(1, 4, 7.0);
  g.addEdge(1, 2, 1.0);
  g.addEdge(2, 3, 3.0);
  g.addEdge(2, 4, 2.0);
  g.addEdge(2, 5, 1.0);
  g.addEdge(2, 6, 3.0);
  g.addEdge(4, 5, 2.0);
  g.addEdge(4, 7, 1.0);
  g.addEdge(5, 7, 3.0);
  g.addEdge(5, 6, 3.0);
  g.addEdge(6, 8, 4.0);
  g.addEdge(7, 8, 5.0);

  std::vector<vertex> path = g.shortestPath(0, 8);

  EXPECT_EQ(0, path[0]);
  EXPECT_EQ(1, path[1]);
  EXPECT_EQ(2, path[2]);
  EXPECT_EQ(6, path[3]);
  EXPECT_EQ(8, path[4]);
}

TEST(Graph, UnorderedVerticies){
  Graph g(7);

  g.addVertex(10);
  g.addVertex(11);
  g.addVertex(12);
  g.addVertex(13);
  g.addVertex(25);
  g.addVertex(15);
  g.addVertex(16);
  g.addVertex(17);
  g.addVertex(50);

  g.addEdge(10, 11, 3.0);
  g.addEdge(10, 12, 7.0);
  g.addEdge(10, 13, 5.0);
  g.addEdge(11, 25, 7.0);
  g.addEdge(11, 12, 1.0);
  g.addEdge(12, 13, 3.0);
  g.addEdge(12, 25, 2.0);
  g.addEdge(12, 15, 1.0);
  g.addEdge(12, 16, 3.0);
  g.addEdge(25, 15, 2.0);
  g.addEdge(25, 17, 1.0);
  g.addEdge(15, 17, 3.0);
  g.addEdge(15, 16, 3.0);
  g.addEdge(16, 50, 4.0);
  g.addEdge(17, 50, 5.0);

  std::vector<vertex> path = g.shortestPath(10, 50);

  EXPECT_EQ(10, path[0]);
  EXPECT_EQ(11, path[1]);
  EXPECT_EQ(12, path[2]);
  EXPECT_EQ(16, path[3]);
  EXPECT_EQ(50, path[4]);
}

TEST(Graph, NoPath){
  Graph g(7);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addVertex(3);
  g.addVertex(4);
  g.addVertex(5);
  g.addVertex(6);
  g.addVertex(7);
  g.addVertex(8);

  g.addEdge(0, 1, 3.0);
  g.addEdge(0, 2, 7.0);
  g.addEdge(0, 3, 5.0);
  g.addEdge(1, 4, 7.0);
  g.addEdge(1, 2, 1.0);
  g.addEdge(2, 3, 3.0);
  g.addEdge(2, 4, 2.0);
  g.addEdge(2, 5, 1.0);
  g.addEdge(2, 6, 3.0);
  g.addEdge(4, 5, 2.0);
  g.addEdge(4, 7, 1.0);
  g.addEdge(5, 7, 3.0);
  g.addEdge(5, 6, 3.0);

  //No edges to vertex 8
  EXPECT_EQ(0, g.shortestPath(0, 8).size());
}

TEST(Graph, MaxNeighbours){
  Graph g(3);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addVertex(3);
  g.addVertex(4);
  g.addVertex(5);

  ASSERT_TRUE(g.addEdge(0, 1, 1.0));
  ASSERT_TRUE(g.addEdge(0, 2, 1.0));
  ASSERT_TRUE(g.addEdge(0, 3, 1.0));
  ASSERT_FALSE(g.addEdge(0, 4, 1.0));
}

TEST(Graph, AlreadyNeighbours){
  Graph g(5);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);

  ASSERT_TRUE(g.addEdge(0, 1, 1.0));
  ASSERT_FALSE(g.addEdge(0, 1, 2.0));
  ASSERT_FALSE(g.addEdge(1, 0, 3.0));

  ASSERT_TRUE(g.addEdge(0, 2, 1.0));
  ASSERT_TRUE(g.addEdge(1, 2, 1.0));
}

TEST(Graph, RemoveVertex){
  Graph g(3);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);

  ASSERT_TRUE(g.removeVertex(0));
  ASSERT_TRUE(g.removeVertex(1));

  std::map<vertex, edges> c = g.container();
  ASSERT_TRUE(c.find(0) == c.end());
  ASSERT_TRUE(c.find(1) == c.end());
  ASSERT_FALSE(c.find(2) == c.end());
}

TEST(Graph, RemoveEdge){
  Graph g(5);

  g.addVertex(0);
  g.addVertex(1);
  g.addVertex(2);
  g.addVertex(3);
  g.addVertex(4);
  g.addVertex(5);

  g.addEdge(0, 1, 1.0);
  g.addEdge(0, 2, 1.0);
  g.addEdge(2, 3, 1.0);
  g.addEdge(2, 4, 1.0);

  g.removeEdgesWithVertex(0);

  std::map<vertex, edges> c = g.container();

  EXPECT_EQ(0, c[1].size());
  EXPECT_EQ(2, c[2].size());
  EXPECT_EQ(0, c[0].size());
}

int main (int argc, char **argv){
  //Run with './devel/lib/prm_sim/prm_sim-test' in catkin_ws
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--show") {
       ShowPrm = true;
       std::cout << "We did it" << std::endl;
    } else if (std::string(argv[i]) == "-t") {
       MaxTries = std::stoi(argv[i + 1]);
       i++;
    } else {
       std::cout << "Invlaid arguments: " << argv[i] << std::endl;
       std::cout << "Format is `prm_sim-test -t <max_rounds> --show`" << std::endl;
       return 0;
    }
  }

  return RUN_ALL_TESTS();
}
