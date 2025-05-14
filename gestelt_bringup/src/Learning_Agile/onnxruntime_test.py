import os
import numpy as np
import onnxruntime as ort
import time
import argparse
from pprint import pprint

def test_onnx_model(model_path, input_size=37, num_tests=100, verbose=True):
    """
    使用 ONNX Runtime 测试 ONNX 模型的推理性能和正确性
    
    参数:
        model_path: ONNX 模型的路径
        input_size: 模型输入大小
        num_tests: 测试次数
        verbose: 是否打印详细信息
    """
    # 检查模型文件是否存在
    if not os.path.exists(model_path):
        print(f"错误: 模型文件 {model_path} 不存在!")
        return
    
    print(f"正在加载 ONNX 模型: {model_path}")
    
    # 创建 ONNX Runtime 会话
    try:
        # 获取可用的执行提供程序
        providers = ort.get_available_providers()
        print(f"可用的执行提供程序: {providers}")
        
        # 创建推理会话
        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        session = ort.InferenceSession(
            model_path, 
            sess_options=session_options,
            providers=providers
        )
        
        # 获取模型输入和输出信息
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        output_name = session.get_outputs()[0].name
        output_shape = session.get_outputs()[0].shape
        
        print("\n模型信息:")
        print(f"输入名称: {input_name}, 形状: {input_shape}")
        print(f"输出名称: {output_name}, 形状: {output_shape}")
        
        # 验证输入尺寸
        actual_input_size = input_shape[1] if len(input_shape) > 1 else input_shape[0]
        if actual_input_size != input_size and input_shape[1] != -1:
            print(f"警告: 指定的输入尺寸 ({input_size}) 与模型的输入尺寸 ({actual_input_size}) 不匹配!")
            # 调整为模型的实际输入尺寸
            input_size = actual_input_size
            
        # 生成随机测试数据
        test_data = np.random.randn(1, input_size).astype(np.float32)
        
        # 执行单次推理以确保模型可用
        print("\n执行测试推理...")
        try:
            outputs = session.run(None, {input_name: test_data})
            output = outputs[0]
            print(f"测试推理成功! 输出形状: {output.shape}")
            
            if verbose:
                print("\n输出预览:")
                print(f"输出最小值: {np.min(output)}")
                print(f"输出最大值: {np.max(output)}")
                print(f"输出均值: {np.mean(output)}")
                print(f"输出前五个值: {output.flatten()[:5]}")
        
        except Exception as e:
            print(f"测试推理失败: {e}")
            return
            
        # 性能测试
        print(f"\n开始性能测试 ({num_tests} 次)...")
        inference_times = []
        
        for i in range(num_tests):
            # 生成新的随机数据
            test_data = np.random.randn(1, input_size).astype(np.float32)
            
            # 计时
            start_time = time.time()
            _ = session.run(None, {input_name: test_data})
            end_time = time.time()
            
            inference_times.append((end_time - start_time) * 1000)  # 转换为毫秒
            
            if verbose and (i+1) % 10 == 0:
                print(f"已完成 {i+1}/{num_tests} 次测试")
                
        # 计算统计数据
        avg_time = np.mean(inference_times)
        std_time = np.std(inference_times)
        min_time = np.min(inference_times)
        max_time = np.max(inference_times)
        
        print("\n性能测试结果:")
        print(f"平均推理时间: {avg_time:.2f} ms")
        print(f"标准差: {std_time:.2f} ms")
        print(f"最小推理时间: {min_time:.2f} ms")
        print(f"最大推理时间: {max_time:.2f} ms")
        print(f"推理频率: {1000/avg_time:.2f} Hz")
        
        # 测试不同批量大小
        print("\n测试不同批量大小的性能:")
        batch_sizes = [1, 2, 4, 8, 16]
        
        for batch_size in batch_sizes:
            # 跳过批量处理如果模型不支持
            if input_shape[0] not in [-1, 1, batch_size]:
                print(f"模型不支持批量大小 {batch_size}，跳过测试")
                continue
                
            test_data = np.random.randn(batch_size, input_size).astype(np.float32)
            
            try:
                # 预热
                for _ in range(5):
                    _ = session.run(None, {input_name: test_data})
                
                # 计时
                start_time = time.time()
                for _ in range(10):
                    _ = session.run(None, {input_name: test_data})
                end_time = time.time()
                
                avg_time = (end_time - start_time) * 100  # 10次推理，转为毫秒
                print(f"批量大小 {batch_size}: {avg_time:.2f} ms, {1000/avg_time:.2f} Hz")
            except Exception as e:
                print(f"批量大小 {batch_size} 测试失败: {e}")
        
        return session
        
    except Exception as e:
        print(f"错误: 无法加载模型 - {e}")
        return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试ONNX模型的性能和正确性")
    parser.add_argument(
        "--model", 
        type=str, 
        default="/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/src/Learning_Agile/deployed_model/2025-05-08/19-28-20/trained_model/converted_models/model.onnx", 
        help="ONNX模型路径"
    )
    parser.add_argument("--input_size", type=int, default=37, help="模型输入大小")
    parser.add_argument("--num_tests", type=int, default=100, help="性能测试次数")
    parser.add_argument("--verbose", action="store_true", help="显示详细信息")
    
    args = parser.parse_args()
    
    # 测试模型
    session = test_onnx_model(
        args.model, 
        input_size=args.input_size, 
        num_tests=args.num_tests, 
        verbose=args.verbose
    )
    
    if session:
        print("\n模型测试成功完成!")

        # 打印更多模型元数据信息
        model_metadata = session.get_modelmeta()
        if model_metadata.custom_metadata_map:
            print("\n模型元数据:")
            pprint(model_metadata.custom_metadata_map)