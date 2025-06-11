from config import mission_cfg, train_cfg
import torch
import os
from gestelt_bringup.src.quad_nn import network_with_GRU

def conversion(model_path):
    # create dummy input
    input_size = train_cfg["model"]["input_size"]
    dummy_input = torch.randn(1, input_size, dtype=torch.float32)
    model = torch.load(model_path).to("cpu")
    traced_model = torch.jit.trace(model, dummy_input)
    traced_model.save("/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/deployed_model/trained_model/NN_close_600.pt")


if __name__ == "__main__":
    training_results_dir = "/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile"
    model_path = os.path.join(training_results_dir, mission_cfg["NN_deploy_model_name"])

    conversion(model_path)
