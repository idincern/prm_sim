/*! @file
 *
 *  @brief A roadmap... TODO!
 *
 *  @author arosspope
 *  @date 12-10-2017
*/
#ifndef LOCALMAP_H
#define LOCALMAP_H

#include "graph.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

typedef struct
{
  unsigned int x;   /*!< x coordinate within local map (pixel) */
  unsigned int y;   /*!< y coordinate within global map (pixel) */
} TLocalOrd;

class LocalMap
{
public:
  //TODO: Inputs robot size for effective cfg space?
  LocalMap();

  //Updates the internal map
  void setMap(cv::Mat m);

  //Converts global ordinates (in m) to local ords
  //TODO: Consider putting this in a global types.h file
  TLocalOrd convert(double x, double y);

  //Given a local map, determine if we can connect two ordinates
  bool canConnect(cv::Mat map, TLocalOrd start, TLocalOrd end);

  //Draw prm overlay on local map
  //void drawPRM(std::map<vertex, edges> container, cv::Mat m);
private:
  cv::Mat map_;

};

#endif // LOCALMAP_H
