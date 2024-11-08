import multiprocessing
import time

from config import mission_cfg, train_cfg,current_dir
from learningAgileBase import LearningAgileBase
from learning_agile_sim import LearningAgileSim, Gate
options = {}
options['MPC_BACKWARD']=True
options['USE_PREV_SOLVER']=True
options['PDP_GRADIENT']= True
options['SQP_RTI_OPTION']=True
options['JAX_SVD']=False
options['STATIC_GATE_TEST']=False
options['ORIGIN_REWARD']=False
options['CLOSE_LOOP_TRAINING']=True
options['TRAINING']=False
options['DEBUG']=False
options['BACKWARD']=True

class MyClass:
    def __init__(self, name, 
                 shared_data,
                 mission_cfg:dict,
                 train_cfg:dict,
                 options:dict):
        self.name = name
        self.shared_data = shared_data
        self.shared_data[name] = 0  # 初始值
        self.learning_agile_sim = LearningAgileSim(python_sim_time=5,
                                                    mission_cfg=mission_cfg,
                                                    dyn_step=1/1,
                                                    options=options)
    def run_task(self):
        for i in range(5):
            time.sleep(1)  # 模拟耗时操作
            self.shared_data[self.name] += 1  # 更新共享数据
            print(f"{self.name} running: {self.shared_data[self.name]}")

def process_instance(instance):
    instance.run_task()

if __name__ == "__main__":
    multiprocessing.set_start_method('fork')
    with multiprocessing.Manager() as manager:
        shared_data = manager.dict()  # 创建共享字典

        # 创建多个 MyClass 实例，每个实例共享 `shared_data`
        instances = [MyClass(f"Instance_{i}", 
                             shared_data,
                             mission_cfg=mission_cfg,
                             train_cfg=train_cfg,
                             options=options) for i in range(2)]
        # instances = [LearningAgileBase(mission_cfg=mission_cfg,
        #                                     train_cfg=train_cfg,
        #                                     options=options) for i in range(2)]

        # 创建并启动多个进程
        processes = [multiprocessing.Process(target=process_instance, args=(inst,)) for inst in instances]
        for p in processes:
            p.start()
        for p in processes:
            p.join()

        # 打印每个实例的最终数据
        print("Final shared data:", dict(shared_data))
