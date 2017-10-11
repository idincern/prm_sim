#include "gtest/gtest.h"
#include "../src/graph.h"

//These tests are based on the graph examples found
//on the website: https://brilliant.org/wiki/dijkstras-short-path-finder/
TEST(GraphTest, FoundPath1){
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

TEST(GraphTest, FoundPath2){
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

TEST(GraphTest, NoPath){
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

TEST(GraphTest, MaxNeighbours){
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


