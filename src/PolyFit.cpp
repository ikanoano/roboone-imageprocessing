/*
 *  Example code for fitting a polynomial to sample data (using Eigen 3)
 *
 *  Copyright (C) 2014  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2014  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <iostream>
#include <eigen3/Eigen/QR>
#include "../include/EigenUtil.hpp"

boost::math::tools::polynomial<double> EigenUtil::PolyFit(
    const std::vector<double> &xv,
    const std::vector<double> &yv,
    size_t                    order) {
  Eigen::MatrixXd A(xv.size(), order+1);
  Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
  Eigen::VectorXd result;

  assert(xv.size() == yv.size());
  assert(xv.size() >= order+1);

  // create matrix
  for (size_t i = 0; i < xv.size(); i++)
  for (size_t j = 0; j < order+1; j++)
    A(i, j) = pow(xv.at(i), j);

  // solve for linear least squares fit
  result = A.householderQr().solve(yv_mapped);

  return boost::math::tools::polynomial<double>((double*)result.data(), (double*)(result.data()+result.rows()*result.cols()));
}

/*
int main() {
  std::vector<double> x_values, y_values;
  double x, y;

  while (scanf("%lf %lf\n", &x, &y) == 2) {
    x_values.push_back(x);
    y_values.push_back(y);
  }

  const auto curve = EigenUtil::PolyFit(x_values, y_values, 3);
  std::cout << curve << std::endl;
  //printf("%f + %f*x + %f*x^2 + %f*x^3\n", curve[0], curve[1], curve[2], curve[3]);
  std::cout <<
    "f(0) = " <<
    boost::math::tools::evaluate_polynomial(curve.data().data(), 0.0, curve.size()) <<
    ", f(1) = " <<
    boost::math::tools::evaluate_polynomial(curve.data().data(), 1.0, curve.size()) <<
    std::endl;

  return 0;
}
*/
