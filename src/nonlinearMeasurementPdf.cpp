#include "chairbot_selfdrive/nonlinearMeasurementPdf.h"
#include <wrappers/rng/rng.h>  // Wrapper around several rng libraries

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE 3

namespace BFL {
using namespace MatrixWrapper;

NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise)
    : ConditionalPdf<ColumnVector, ColumnVector>(MEASMODEL_DIMENSION_MOBILE,
                                                 MEASMODEL_NUMCONDARGUMENTS_MOBILE) {
  _measNoise = measNoise;
}

NonlinearMeasurementPdf::~NonlinearMeasurementPdf() {}

//! 要は確率をProbability型で返せればおｋ
//  Probability p(0.3)とかでも作れる
Probability NonlinearMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const {
  ColumnVector state = ConditionalArgumentGet(0);
  ColumnVector expected_measurement(2);
  expected_measurement(1) = state(1);
  expected_measurement(2) = state(2);
  Probability prb = _measNoise.ProbabilityGet(measurement - expected_measurement);
  return prb;
}

}  // namespace BFL
