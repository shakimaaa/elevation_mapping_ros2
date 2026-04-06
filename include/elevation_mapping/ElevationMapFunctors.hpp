/**
 * @file ElevationMapFunctors.hpp
 * @author Beauhowe Zhang <zbohao7@gmail.com>
 * @brief 栅格逐元素算子：将方差裁剪到 [min,max]，过大则置为 inf 以触发融合中的剔除逻辑。
 */

#pragma once

namespace elevation_mapping {

template <typename Scalar>
struct VarianceClampOperator {
  VarianceClampOperator(const Scalar& minVariance, const Scalar& maxVariance) : minVariance_(minVariance), maxVariance_(maxVariance) {}
  const Scalar operator()(const Scalar& x) const {
    return x < minVariance_ ? minVariance_ : (x > maxVariance_ ? std::numeric_limits<float>::infinity() : x);
  }
  Scalar minVariance_, maxVariance_;
};

}  // namespace elevation_mapping
