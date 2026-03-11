import torch
import numpy as np

class CudaTimer:
    def __init__(self, name):
        self.name = name

        self.start = torch.cuda.Event(enable_timing=True)
        self.end = torch.cuda.Event(enable_timing=True)

        self.latency_list = []
    
    def start_timer(self):
        self.start.record()
    
    def end_timer(self):
        self.end.record()
        torch.cuda.synchronize()

        self.current_runtime = self.start.elapsed_time(self.end)
        self.latency_list.append(self.current_runtime)

    def get_average_runtime(self):
        return float(np.average(self.latency_list))

    def get_current_runtime(self):
        return self.current_runtime
        
        