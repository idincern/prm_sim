/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef GLOBALMAP_H
#define GLOBALMAP_H

#include <map>
#include <utility>

#include "localmap.h"
#include "graph.h"
#include "types.h"


class GlobalMap
{
public:
  /*! @brief Constructor for a GlobalMap.
   *
   *  @param mapSize The size of the OgMap in meters (square maps only).
   *  @param res The resolution of the OgMaps provided to this object.
   *  @param robotDiameter Used in computing the effective configuration space.
   *
   *  @note This will set the reference position to 0,0 by default. To change this,
   *        call setReference().
   */
  GlobalMap(double mapSize, double mapRes, double robotDiameter);

  /*! @brief Builds a prm network between a start and end ordinate.
   *
   *  @param space The OgMap to build the prm network within.
   *  @param start The starting ordinate. This is usually the robot's position.
   *  @param goal  The goal ordiante to reach from start.
   *  @param vector<TGlobalOrd> - An ordered vector of globalOrd's between start
   *                              and goal. This will be empty if no path was
   *                              discovered.
   */
  std::vector<TGlobalOrd> build(cv::Mat &space, TGlobalOrd start, TGlobalOrd goal);

  /*! @brief Overlays the current state of the PRM unto a colour OgMap.
   *
   *  Not only will this overlay the prm (in blue), but if supplied with
   *  a valid non-empty path, it will also overaly this trajectory (red).
   *
   *  @param space The OgMap to overlay the PRM and path on top of.
   *  @param path The path to also overlay. Empty if no path to display.
   */
  void showOverlay(cv::Mat &space, std::vector<TGlobalOrd> path);

  /*! @brief Sets the reference position of the provided OgMaps.
   *
   *  @param reference The reference to set, this is usually the robot's
   *                   global position.
   */
  void setReference(const TGlobalOrd reference);

  /*! @brief Updates the size of the OgMaps provided.
   *
   *  @param mapSize The size of the OgMap in meters (square maps only).
   */
  void setMapSize(double mapSize);

private:
  Graph graph_;                             /*!< A graph representation of the roadmap network */
  LocalMap lmap_;                           /*!< An object for interacting with the ogMap provided to this object */
  std::map<vertex, TGlobalOrd> network_;    /*!< A look up table to convert a vertex to coordinate within map */
  vertex nextVertexId_;                     /*!< Used for generating unique vertex ids for coordiantes... TODO: make atomic?? */
  TGlobalOrd reference_;                    /*!< Reference ordinate for the local map, this is usually the robot position */
  double mapSize_;                          /*!< The mapSize in m */
  double robotDiameter_;                    /*!< The diameter of the robot in m */

  /*! @brief Optimises a path between two points in a config space.
   *
   *  In some cases, the shortest path in a PRM network may not be the
   *  most direct route between a start and goal. This function aims
   *  to remove the points in the path that can be directly accessed
   *  by earlier points. For example, if there is a direct path between
   *  start and goal, this function will find it.
   *
   *  @param cspace The configuration space to test for direct access within.
   *  @param path An ordered representation of the path, where the first element
   *              is the start, and the end element is the goal.
   */
  std::vector<TGlobalOrd> optimisePath(cv::Mat &cspace, std::vector<TGlobalOrd> path);

  /*! @brief Returns a representation of the internal PRM.
   *
   *  @return vector<<Point, Point>> - A vector of pairs of points. This represents
   *                                   their connection to eachother.
   */
  std::vector<std::pair<cv::Point, cv::Point>> getPRM();

  /*! @brief Converts a path of globalOrds to OgMap points.
   *
   *  @param path The path of ordiantes to convert.
   *  @return vector<points> - The converted path of OgMap points.
   */
  std::vector<cv::Point> toPointPath(std::vector<TGlobalOrd> path);

  /*! @brief Converts a path of verticies to Global ords.
   *
   *  @param path The path of verticies to convert.
   *  @return vector<TGlobalOrd> - The converted path of Verticies.
   */
  std::vector<TGlobalOrd> toOrdPath(std::vector<vertex> path);

  /*! @brief Connects a single node to the PRM network.
   *
   *  Given a node, determine what other nodes it can connect to
   *  within the configuration space.
   *
   *  @param cspace The configuration space to build the PRM network within.
   *  @param node The node to connect to the PRM network.
   *  @param imposeMaxDist In some cases, we might want to ignore the distance
   *                       constraints between other nodes within the cspace.
   */
  void connectNode(cv::Mat &cspace, vertex node, bool imposeMaxDist);

  /*! @brief Connects all known nodes to the PRM network.
   *
   *  @param cspace The configuration space to build the PRM network within.
   *  @param imposeMaxDist In some cases, we might want to ignore the distance
   *                       constraints between other nodes within the cspace.
   */
  void connectNodes(cv::Mat &cspace, bool imposeMaxDist);

  /*! @brief Get the vertex corresponding to a global ordiante.
   *
   *  If no vertex exists for this ordinate, then add it to the graph
   *  and return the newly generated vertex.
   *
   *  @param ordinate The ordiante to find or add.
   *  @return vertex - The vertex representation of the ordinate.
   */
  vertex findOrAdd(TGlobalOrd ordinate);

  /*! @brief Determines if an ordiante exists within the graph as a vertex.
   *
   *  @param ordinate The ordiante to find or add.
   *  @return TRUE - If the ordiante exists.
   */
  bool existsAsVertex(TGlobalOrd ord);

  /*! @brief Finds the vertex corresponding to an ordiante.
   *
   *  @param ordinate The ordiante to find.
   *  @param v A reference to put the found vertex into.
   *  @return TRUE - If the ordiante was found within the network_.
   */
  bool lookup(TGlobalOrd ord, vertex &v);

  /*! @brief Returns the next unique vertex id within the graph.
   *
   *  @return vertex The next unique vertex id.
   */
  vertex nextVertexId();
};

#endif // GLOBALMAP_H
