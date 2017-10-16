/*! @file
 *
 *  @brief A library of simple types shared amongst classes.
 *
 *  @author arosspope
 *  @date 13-10-2017
*/
#ifndef TYPES
#define TYPES

struct TGlobalOrd
{
  double x;   /*!< x coordinate within global map (m) */
  double y;   /*!< y coordinate within global map (m) */

  bool operator== (const TGlobalOrd &o1){
    return (this->x == o1.x && this->y == o1.y);
  }
};

#endif // TYPES

