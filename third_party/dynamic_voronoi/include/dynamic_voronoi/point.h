#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

#include <unordered_set>

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}

  void set(int _x, int _y){
    x = _x;
    y = _y;
  }

  // Equality
  bool operator == (const IntPoint& pos) const
  {
    return (this->x == pos.x && this->y == pos.y);
  }

  int x, y;
};

/*! A light-weight double point with fields x,y */
class DblPoint {
public:
  DblPoint() : x(0), y(0) {}
  DblPoint(double _x, double _y) : x(_x), y(_y) {}

  void set(double _x, double _y){
    x = _x;
    y = _y;
  }

  double x, y;
};


template <> 
struct std::hash<IntPoint> {
  /* implement hash function so we can put IntPoint into an unordered_set */
  std::size_t operator()(const IntPoint& pos) const noexcept {
    // NOTE: better to use something like boost hash_combine
    return 0.5 * (pos.x + pos.y)*(pos.x + pos.y + 1) + pos.y;
  }
};

#endif
