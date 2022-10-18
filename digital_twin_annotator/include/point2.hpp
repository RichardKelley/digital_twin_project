#ifndef POINT2_HPP
#define POINT2_HPP

#include <iostream>

namespace digital_twin {

  struct Point2 {
    float x = 0.0f;
    float y = 0.0f;

    friend std::ostream& operator<<(std::ostream& os, const Point2& pt) {
      os << "Point(" << pt.x << ", " << pt.y << ")";
      return os;
    }
  };

} // namespace digital_twin

#endif // POINT2_HPP
