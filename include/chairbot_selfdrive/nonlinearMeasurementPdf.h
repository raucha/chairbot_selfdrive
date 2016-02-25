#ifndef __NON_LINEAR_MEAS_MOBILE__
#define __NON_LINEAR_MEAS_MOBILE__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
// #include "map/map.h"
#include <math.h>

namespace BFL {
/// Non Linear Conditional Gaussian
class NonlinearMeasurementPdf
    : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> {
 public:
  /// Constructor
  /**
       @param additiveNoise Pdf representing the additive Gaussian uncertainty
  */
  NonlinearMeasurementPdf(const Gaussian& measNoise);

  /// Destructor
  virtual ~NonlinearMeasurementPdf();

  // implement this virtual function for measurement model of a particle filter
  virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

 private:
  Gaussian _measNoise;
};

}  // End namespace BFL

#endif  //
