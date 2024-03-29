#ifndef __NON_LINEAR_SYSTEM_MOBILE__
#define __NON_LINEAR_SYSTEM_MOBILE__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>

namespace BFL {
/// NonLinear Conditional Gaussian
class NonlinearSystemPdf
    : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector> {
 public:
  /// Constructor
  /** @param additiveNoise Pdf representing the additive Gaussian uncertainty
  */
  NonlinearSystemPdf(const Gaussian& additiveNoise);

  /// Destructor
  virtual ~NonlinearSystemPdf();

  // implement this virtual function for system model of a particle filter
  virtual bool SampleFrom(Sample<MatrixWrapper::ColumnVector>& one_sample, int method = DEFAULT,
                          void* args = NULL) const;

 private:
  Gaussian _additiveNoise;
};

}  // End namespace BFL

#endif  //
