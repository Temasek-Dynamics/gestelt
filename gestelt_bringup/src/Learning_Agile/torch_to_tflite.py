import os
import sys
import numpy as np
import torch
import torch.onnx
import tensorflow as tf
import onnx
from onnx_tf.backend import prepare

# 添加包含模型定义的目录到路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.append(parent_dir)

# 从当前项目导入模型定义
from config import mission_cfg, train_cfg
from quad_nn import network_with_GRU
# Hyper-parameters 
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']

def create_representative_dataset(input_size, num_samples=100):
    """创建代表性数据集用于量化校准"""
    def representative_dataset():
        for _ in range(num_samples):
            # 生成随机输入数据，范围在[-1, 1]之间
            data = np.random.uniform(-1, 1, (1, input_size)).astype(np.float32)
            yield [data]
    return representative_dataset

def convert_pytorch_to_tflite(model_path, output_dir, input_size=37, output_size=20):
    """完整的转换流程：PyTorch -> ONNX -> TensorFlow -> TFLite"""
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 定义输出路径
    onnx_path = os.path.join(output_dir, "model.onnx")
    tf_saved_model_path = os.path.join(output_dir, "tf_saved_model")
    tflite_path = os.path.join(output_dir, "model_int8.tflite")
    
    # 步骤1: PyTorch -> ONNX
    print("步骤1: 转换 PyTorch 模型到 ONNX...")
    
    # 加载PyTorch模型
    model = network_with_GRU(input_size, hidden_size, hidden_size, output_size)
    model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
    # model.eval()
    
    # 创建示例输入
    dummy_input = torch.randn(1, input_size, dtype=torch.float32)
    
    # 导出到ONNX
    onnx_program=torch.onnx.export(
        model,                       # 模型
        (dummy_input,),                 # 模型输入
        onnx_path,                   # 保存路径
        # export_params=True,          # 存储训练好的参数
        opset_version=17,            # ONNX版本


    )
    print(f"ONNX模型已保存到: {onnx_path}")
    # 验证ONNX模型
    onnx_model = onnx.load(onnx_path)
    onnx.checker.check_model(onnx_model)
    print("ONNX模型验证成功!")
    
    # 步骤2: ONNX -> TensorFlow SavedModel
    print("\n步骤2: 转换 ONNX 到 TensorFlow SavedModel...")
    
    # 加载ONNX模型并转换为TensorFlow
    tf_rep = prepare(onnx_model)
    
    # 保存为SavedModel
    tf_rep.export_graph(tf_saved_model_path)
    print(f"TensorFlow SavedModel已保存到: {tf_saved_model_path}")
    
    # 步骤3: TensorFlow SavedModel -> TFLite (INT8)
    print("\n步骤3: 转换 TensorFlow SavedModel 到 TFLite (INT8)...")
    
    # 加载SavedModel
    converter = tf.lite.TFLiteConverter.from_saved_model(tf_saved_model_path)
    
    
    # 设置INT8量化
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    
    # 创建校准数据集
    converter.representative_dataset = create_representative_dataset(input_size)
    
    # 执行转换
    tflite_model = converter.convert()
    
    # 保存TFLite模型
    with open(tflite_path, 'wb') as f:
        f.write(tflite_model)
    print(f"量化TFLite模型(INT8)已保存到: {tflite_path}")
    
    # 验证和测试各模型
    verify_models(model, onnx_path, tf_saved_model_path, tflite_path, dummy_input.numpy(), input_size)

def verify_models(pytorch_model, onnx_path, tf_saved_model_path, tflite_path, input_data, input_size):
    """验证所有模型的输出并比较性能"""
    print("\n模型验证与性能测试:")
    
    # 1. PyTorch模型输出
    pytorch_output = pytorch_model(torch.tensor(input_data)).detach().numpy()
    print(f"PyTorch模型输出形状: {pytorch_output.shape}")
    
    # 2. ONNX模型输出
    import onnxruntime
    ort_session = onnxruntime.InferenceSession(onnx_path)
    ort_inputs = {ort_session.get_inputs()[0].name: input_data}
    ort_output = ort_session.run(None, ort_inputs)[0]
    print(f"ONNX模型输出形状: {ort_output.shape}")
    
    # 计算误差
    onnx_error = np.abs(pytorch_output - ort_output).mean()
    print(f"PyTorch vs ONNX平均误差: {onnx_error}")
    
    # 3. TensorFlow SavedModel输出
    loaded_model = tf.saved_model.load(tf_saved_model_path)
    infer = loaded_model.signatures["serving_default"]
    tf_output = infer(tf.convert_to_tensor(input_data))['output'].numpy()
    print(f"TensorFlow模型输出形状: {tf_output.shape}")
    
    # 计算误差
    tf_error = np.abs(ort_output - tf_output).mean()
    print(f"ONNX vs TensorFlow平均误差: {tf_error}")
    
    # 4. TFLite模型输出
    interpreter = tf.lite.Interpreter(model_path=tflite_path)
    interpreter.allocate_tensors()
    
    # 获取输入输出细节
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    # 量化参数
    input_scale, input_zero_point = input_details[0]["quantization"]
    output_scale, output_zero_point = output_details[0]["quantization"]
    
    # 预处理输入数据
    input_data_quantized = input_data / input_scale + input_zero_point
    input_data_quantized = input_data_quantized.astype(np.int8)
    
    # 设置输入
    interpreter.set_tensor(input_details[0]['index'], input_data_quantized)
    
    # 运行推理
    interpreter.invoke()
    
    # 获取输出
    tflite_output_quantized = interpreter.get_tensor(output_details[0]['index'])
    
    # 反量化
    tflite_output = (tflite_output_quantized.astype(np.float32) - output_zero_point) * output_scale
    
    print(f"TFLite模型输出形状: {tflite_output.shape}")
    
    # 计算误差 (与TensorFlow模型比较)
    tflite_error = np.abs(tf_output - tflite_output).mean()
    print(f"TensorFlow vs TFLite (INT8)平均误差: {tflite_error}")
    
    # 性能测试
    import time
    
    # PyTorch性能
    start = time.time()
    for _ in range(100):
        pytorch_model(torch.tensor(input_data))
    pytorch_time = (time.time() - start) / 100
    
    # ONNX性能
    start = time.time()
    for _ in range(100):
        ort_session.run(None, ort_inputs)
    onnx_time = (time.time() - start) / 100
    
    # TensorFlow性能
    start = time.time()
    for _ in range(100):
        infer(tf.convert_to_tensor(input_data))
    tf_time = (time.time() - start) / 100
    
    # TFLite性能
    start = time.time()
    for _ in range(100):
        interpreter.set_tensor(input_details[0]['index'], input_data_quantized)
        interpreter.invoke()
        interpreter.get_tensor(output_details[0]['index'])
    tflite_time = (time.time() - start) / 100
    
    print("\n性能对比 (平均推理时间):")
    print(f"PyTorch: {pytorch_time*1000:.2f} ms")
    print(f"ONNX: {onnx_time*1000:.2f} ms (加速比: {pytorch_time/onnx_time:.2f}x)")
    print(f"TensorFlow: {tf_time*1000:.2f} ms (加速比: {pytorch_time/tf_time:.2f}x)")
    print(f"TFLite (INT8): {tflite_time*1000:.2f} ms (加速比: {pytorch_time/tflite_time:.2f}x)")

if __name__ == "__main__":
    # 模型路径
    model_path = "/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/deployed_model/2025-05-08/19-28-20/trained_model/converted_models/model_state_dict.pth"
    
    # 输出目录
    output_dir = os.path.join("/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/deployed_model/2025-05-08/19-28-20/trained_model/converted_models")
    
    # 执行转换
    convert_pytorch_to_tflite(model_path, output_dir)