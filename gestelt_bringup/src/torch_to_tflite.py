import os
import sys
import numpy as np
import torch
from torch.nn.utils import remove_spectral_norm
import torch.onnx
import tensorflow as tf
import onnx
# from onnx_tf.backend import prepare
import onnx2tf

# 添加包含模型定义的目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.append(parent_dir)

# 从当前项目导入模型定义
from Learning_Agile.config import train_cfg
from gestelt_bringup.src.quad_nn import network
# Hyper-parameters 
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']

def create_representative_dataset(input_size, num_samples=100):
    """create representative dataset for INT8 quantization"""
    def representative_dataset():
        for _ in range(num_samples):
            data = np.random.uniform(-1, 1, (1, input_size)).astype(np.float32)
            yield [data]
    return representative_dataset

def convert_pytorch_to_tflite(model_path, output_dir, input_size=37, output_size=20):
    """
    Pipeline: PyTorch -> ONNX -> TensorFlow -> TFLite
    
    Args:
        model_path (str): Path to the PyTorch model file.
        output_dir (str): Directory to save the converted models.
        input_size (int): Size of the input layer.
        output_size (int): Size of the output layer.
    
    """
    
    os.makedirs(output_dir, exist_ok=True)
    onnx_path = os.path.join(output_dir, "model.onnx")
    tf_saved_model_path = os.path.join(output_dir, "onnx2tf_converted_model")
    tflite_path = os.path.join(output_dir, "model_int8.tflite")
    
    ###############################
    # step 1: PyTorch -> ONNX
    ###############################
    print("Step 1: PyTorch -> ONNX")
    
    # load PyTorch model
    # model = network(input_size, hidden_size, hidden_size,
    #             weights_vector_length=train_cfg['model']['weights_vector_length'],
    #             activation=train_cfg['model']['activation'])
    
    # model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
    # model.eval()
    device = torch.device("cpu")
    model = torch.load(model_path, map_location=device)
    # create dummy input
    dummy_input = torch.randn(1, input_size, dtype=torch.float32)
    
    # 导出到ONNX
    torch.onnx.export(
        model,                       
        (dummy_input,),                
        onnx_path,                   
        opset_version=19,           
    )

   
    # onnx_model = onnx.load(onnx_path)
    # onnx.checker.check_model(onnx_model)

    # ###############################
    # # step 2: ONNX -> TensorFlow SavedModel
    # ###############################
    # onnx2tf.convert(
    #     input_onnx_file_path = onnx_path,
    #     output_folder_path   = tf_saved_model_path, 
    #     output_signaturedefs = "output",
    # )
    
    # ###############################
    # # step 3: TensorFlow SavedModel -> TFLite
    # ###############################
    # converter = tf.lite.TFLiteConverter.from_saved_model(tf_saved_model_path)
    
    
    # # set static quantization
    # converter.optimizations = [tf.lite.Optimize.DEFAULT]
    # converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    # converter.inference_input_type = tf.int8
    # converter.inference_output_type = tf.int8
    
    # # create representative dataset
    # converter.representative_dataset = create_representative_dataset(input_size)
    
    # # convert to TFLite model
    # tflite_model = converter.convert()
    
    # # save TFLite model
    # with open(tflite_path, 'wb') as f:
    #     f.write(tflite_model)
    # print(f"quantized (INT8) TFLite model saved to {tflite_path}")
    
    # # validate models
    verify_models(model, onnx_path, tf_saved_model_path, tflite_path, dummy_input.numpy(), input_size)

def verify_models(pytorch_model, onnx_path, tf_saved_model_path, tflite_path, input_data, input_size):
    """Validate all models by comparing their outputs and performance.
    Args:
        pytorch_model (torch.nn.Module): PyTorch model.
        onnx_path (str): Path to the ONNX model.
        tf_saved_model_path (str): Path to the TensorFlow SavedModel.
        tflite_path (str): Path to the TFLite model.
        input_data (np.ndarray): Input data for validation.
        input_size (int): Size of the input layer.
    """
    
    # 1. PyTorch
    pytorch_output = pytorch_model(torch.tensor(input_data)).detach().numpy()
    print(f"PyTorch model output shape: {pytorch_output.shape}")
    print('torch model output', pytorch_output)
    
    # 2. ONNX
    import onnxruntime
    ort_session = onnxruntime.InferenceSession(onnx_path)
    ort_inputs = {ort_session.get_inputs()[0].name: input_data}
    ort_output = ort_session.run(None, ort_inputs)[0][0]
    print(f"ONNX model output shape: {ort_output.shape}")
    print('onnx model output',ort_output)
    
    # PyTorch vs ONNX 
    onnx_error = np.abs(pytorch_output - ort_output).mean()
    print(f"PyTorch vs ONNX mean abs error: {onnx_error}")
    
    
    # 3. TensorFlow SavedModel
    # loaded_model = tf.saved_model.load(tf_saved_model_path)
    # infer = loaded_model.signatures["serving_default"]
    # print("output_name:", infer.structured_outputs)
    # tf_output = infer(tf.convert_to_tensor(input_data))['109'].numpy()
    # print(f"TensorFlow model output shape: {tf_output.shape}")
    
    # # PyTorch vs TensorFlow
    # tf_error = np.abs(ort_output - tf_output).mean()
    # print(f"ONNX vs TensorFlow mean output error: {tf_error}")
    
    # # 4. TFLite
    # interpreter = tf.lite.Interpreter(model_path=tflite_path)
    # interpreter.allocate_tensors()
    
    # # obtain the input and output format for the tflite model
    # input_details = interpreter.get_input_details()
    # output_details = interpreter.get_output_details()
    
    # # extract the quatization parameters
    # input_scale, input_zero_point = input_details[0]["quantization"]
    # output_scale, output_zero_point = output_details[0]["quantization"]
    
    # # pre-process the input data based on the quantization parameters
    # input_data_quantized = input_data / input_scale + input_zero_point
    # input_data_quantized = input_data_quantized.astype(np.int8)
    
    # # send the input data to the interpreter
    # interpreter.set_tensor(input_details[0]['index'], input_data_quantized)
    
    # # tflite model inference
    # interpreter.invoke()
    
    # # obtain the output data
    # tflite_output_quantized = interpreter.get_tensor(output_details[0]['index'])
    
    # # de-quantize the output data
    # tflite_output = (tflite_output_quantized.astype(np.float32) - output_zero_point) * output_scale
    
    # print(f"TFLite model shape is: {tflite_output.shape}")
    
    # # Pytorch vs TFLite
    # tflite_error = np.abs(pytorch_output - tflite_output).mean()
    # print(f"Pytorch vs TFLite (INT8) mean error: {tflite_error}")
    
    # performance comparison
    import time
    
    # PyTorch
    start = time.time()
    for _ in range(100):
        pytorch_model(torch.tensor(input_data))
    pytorch_time = (time.time() - start) / 100
    
    # ONNX
    start = time.time()
    for _ in range(100):
        ort_session.run(None, ort_inputs)
    onnx_time = (time.time() - start) / 100
    
    # TensorFlow
    # start = time.time()
    # for _ in range(100):
    #     infer(tf.convert_to_tensor(input_data))
    # tf_time = (time.time() - start) / 100
    
    # # TFLite
    # start = time.time()
    # for _ in range(100):
    #     interpreter.set_tensor(input_details[0]['index'], input_data_quantized)
    #     interpreter.invoke()
    #     interpreter.get_tensor(output_details[0]['index'])
    # tflite_time = (time.time() - start) / 100
    
    print("\nperformance comparison (average time for 100 runs):")
    print(f"PyTorch: {pytorch_time*1000:.2f} ms")
    print(f"ONNX: {onnx_time*1000:.2f} ms (speed up: {pytorch_time/onnx_time:.2f}x)")
    # print(f"TensorFlow: {tf_time*1000:.2f} ms (speed up: {pytorch_time/tf_time:.2f}x)")
    # print(f"TFLite (INT8): {tflite_time*1000:.2f} ms (speed up: {pytorch_time/tflite_time:.2f}x)")

if __name__ == "__main__":
    # training_results_dir
    training_results_dir = "/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/training_results/new_format/"
    model_path = training_results_dir+"2025-05-08/19-28-20/trained_model/NN_close_600.pth"
    output_dir = os.path.join(training_results_dir,"2025-05-08/19-28-20/compressed_model/NN_close_600")

    convert_pytorch_to_tflite(model_path, output_dir, input_size, output_size)