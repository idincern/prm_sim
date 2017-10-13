#include "gtest/gtest.h"
#include "../src/localmap.h"
#include "../src/graph.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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

TEST(LocalMap, connectInEmptyMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = blankMap();
  //Vertical
  ASSERT_TRUE(l.canConnect(img, cv::Point(100, 0), cv::Point(100, 200)));

  //Horizontal
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 100), cv::Point(200, 100)));

  //Diagonal
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 0), cv::Point(200, 200)));
}

TEST(LocalMap, connectInPartionedMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = partionedMap();
  //Vertical
  ASSERT_FALSE(l.canConnect(img, cv::Point(100, 0), cv::Point(100, 200)));

  //Horizontal
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 150), cv::Point(10, 150)));

  //Diagonal
  ASSERT_FALSE(l.canConnect(img, cv::Point(0, 0), cv::Point(200, 200)));
}

TEST(LocalMap, connectInUnknownMap){
  LocalMap l(20.0, 0.1);

  cv::Mat img = partionedMap();
  //Vertical
  ASSERT_FALSE(l.canConnect(img, cv::Point(100, 0), cv::Point(100, 200)));

  //Horizontal -Two small points in known blank area of the map
  ASSERT_TRUE(l.canConnect(img, cv::Point(0, 50), cv::Point(200, 50)));

  //Diagonal
  ASSERT_FALSE(l.canConnect(img, cv::Point(0, 0), cv::Point(200, 200)));
}

TEST(LocalMap, convertPositivePoints){
  LocalMap l(20.0, 0.1);

  TGlobalOrd ref = {10, 10};
  TGlobalOrd p1 = {5, 15}, p2 = {15, 15}, p3 = {5, 5}, p4 = {15, 5}, p5 = {10, -10};

  EXPECT_EQ(cv::Point(50, 50), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(150, 50), l.convertToPoint(ref, p2));
  EXPECT_EQ(cv::Point(50, 150), l.convertToPoint(ref, p3));
  EXPECT_EQ(cv::Point(150, 150), l.convertToPoint(ref, p4));
  EXPECT_EQ(cv::Point(100, 300), l.convertToPoint(ref, p5)); //Outside the map space
}

TEST(LocalMap, convertNegativePoints){
  LocalMap l(20.0, 0.1);

  TGlobalOrd ref = {-10, -10};
  TGlobalOrd p1 = {-15, -5}, p2 = {-5, -5}, p3 = {-15, -15}, p4 = {-5, -15};

  EXPECT_EQ(cv::Point(50, 50), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(150, 50), l.convertToPoint(ref, p2));
  EXPECT_EQ(cv::Point(50, 150), l.convertToPoint(ref, p3));
  EXPECT_EQ(cv::Point(150, 150), l.convertToPoint(ref, p4));
}

TEST(LocalMap, convertPointsOnLine){
  LocalMap l(20.0, 0.1);

  TGlobalOrd ref = {0, 0};
  TGlobalOrd p1 = {0, 5}, p2 = {5, 0}, p3 = {-5, 0}, p4 = {0, -5};

  EXPECT_EQ(cv::Point(100, 50), l.convertToPoint(ref, p1));
  EXPECT_EQ(cv::Point(150, 100), l.convertToPoint(ref, p2));
  EXPECT_EQ(cv::Point(50, 100), l.convertToPoint(ref, p3));
  EXPECT_EQ(cv::Point(100, 150), l.convertToPoint(ref, p4));
}

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
  EXPECT_EQ(0,image.at<uchar>(pixels/2,pixels/2));
}

//These tests are based on the graph examples found
//on the website: https://brilliant.org/wiki/dijkstras-short-path-finder/
TEST(Graphs, FoundPath1){
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

TEST(Graphs, FoundPath2){
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

TEST(Graphs, UnorderedVerticies){
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

TEST(Graphs, NoPath){
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

TEST(Graphs, MaxNeighbours){
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

int main (int argc, char **argv){
  //Run with './devel/lib/prm_sim/prm_sim-test' in catkin_ws
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


