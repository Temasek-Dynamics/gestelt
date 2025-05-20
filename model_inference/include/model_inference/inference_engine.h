#pragma once
#include <vector>

class InferenceEngine // the base class for all inference engines
{
public:
  virtual std::vector<float> predict(const std::vector<float>& in) = 0;
  virtual ~InferenceEngine() = default;
};

