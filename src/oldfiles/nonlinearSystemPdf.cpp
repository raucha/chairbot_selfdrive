#include "chairbot_selfdrive/nonlinearSystemPdf.h"
#include <wrappers/rng/rng.h>  // Wrapper around several rng libraries
#include <iostream>

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE 3

namespace BFL {
using namespace MatrixWrapper;

NonlinearSystemPdf::NonlinearSystemPdf(const Gaussian& additiveNoise)
    : ConditionalPdf<ColumnVector, ColumnVector>(SYSMODEL_DIMENSION_MOBILE,
                                                 SYSMODEL_NUMCONDARGUMENTS_MOBILE) {
  _additiveNoise = additiveNoise;
}

NonlinearSystemPdf::~NonlinearSystemPdf() {}

//! 要はノイズの乗った次状態予測値をone_sampleで返せればおｋ
bool NonlinearSystemPdf::SampleFrom(Sample<ColumnVector>& one_sample, int method,
                                    void* args) const {
  //! [x, y, rad]
  ColumnVector state = ConditionalArgumentGet(0);
  //! [v, sigma]
  ColumnVector vel = ConditionalArgumentGet(1);

  // sample from additive noise
  Sample<ColumnVector> noise;  //! [v, sigma]
  _additiveNoise.SampleFrom(noise, method, args);

  // system update
  state(1) += cos(state(3)) * (vel(1) + noise.ValueGet()(1));
  state(2) += sin(state(3)) * (vel(1) + noise.ValueGet()(1));
  state(3) += (vel(2) + noise.ValueGet()(2));
  // std::cout << "cov: " << _additiveNoise.CovarianceGet() << std::endl;
  // std::cout << "predict: " << vel(1) << "  " << vel(2) << std::endl;

  // store results in one_sample
  one_sample.ValueSet(state);
  // one_sample.ValueSet(state + noise.ValueGet());

  return true;
  // ColumnVector state = ConditionalArgumentGet(0);
  // ColumnVector vel = ConditionalArgumentGet(1);
  //
  // // system update
  // state(1) += cos(state(3)) * vel(1);  //* 100;
  // state(2) += sin(state(3)) * vel(1);  // * 100;
  // state(3) += vel(2);
  // // std::cout << "cov: " << _additiveNoise.CovarianceGet() << std::endl;
  // // std::cout << "predict: " << vel(1) << "  " << vel(2) << std::endl;
  //
  // // sample from additive noise
  // Sample<ColumnVector> noise;
  // _additiveNoise.SampleFrom(noise, method, args);
  //
  // // store results in one_sample
  // one_sample.ValueSet(state + noise.ValueGet());
  //
  // return true;
}

}  // namespace BFL
