#ifndef EIGENUTIL_H
#define EIGENUTIL_H

#include <vector>
#include <boost/math/tools/polynomial.hpp>

class EigenUtil {
public:
  static boost::math::tools::polynomial<double> PolyFit(
      const std::vector<double> &xv,
      const std::vector<double> &yv,
      size_t                    order);
private:
  EigenUtil() {}
};

#endif /* end of include guard */

