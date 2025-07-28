import matplotlib.pyplot as plt
import re
from collections import defaultdict
import os


file_path = os.path.join(os.path.dirname(__file__), "dasm_log.txt")

task_latencies = defaultdict(list)

with open(file_path, "r") as file:
    for line in file:
        line = line.strip()
        match = re.match(r"\[DASM\] (\w+) ID = \d+, End-to-End latency = ([\d.]+) µs", line)
        if match:
            task, latency = match.groups()
            task_latencies[task].append(float(latency))

# 각 task별 1ms 단위 히스토그램 출력
for task in ['SFM', 'Lane', 'Detection']:
    if task in task_latencies:
        plt.figure(figsize=(16, 8))
        plt.hist(task_latencies[task], bins=range(int(min(task_latencies[task]))-50, int(max(task_latencies[task])) + 50, 50), edgecolor='black')
        plt.title(f"{task} End-to-End Latency Histogram")
        plt.xlabel("Latency (µs)")
        plt.ylabel("Frequency")
        plt.grid(True)
        plt.show()

