#include "model_inference/torch_engine.h"


TorchEngine::TorchEngine(const std::string& path, const std::string& device)
    : device_(device)
{
  module_ = torch::jit::load(path, device_);
  module_.eval();
}

std::vector<float> TorchEngine::predict(const std::vector<float>& in)
{
  torch::NoGradGuard guard;
  auto input = torch::from_blob(
      const_cast<float*>(in.data()), {1,static_cast<long>(in.size())},
      torch::kFloat).to(device_);

  auto out = module_.forward({input}).toTensor().to(torch::kCPU);
  return {out.data_ptr<float>(), out.data_ptr<float>() + out.numel()};
}
