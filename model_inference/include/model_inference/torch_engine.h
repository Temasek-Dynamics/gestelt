#pragma once
#include "model_inference/inference_engine.h"
#include <torch/script.h>


class TorchEngine final : public InferenceEngine
{
public:
  explicit TorchEngine(const std::string& path,
                       const std::string& device = "cpu");

  std::vector<float> predict(const std::vector<float>& in) override;

private:
  torch::jit::script::Module module_;
  torch::Device device_;
};

