import sys
import time
import psutil

def get_all_processes():
    nodes = ['standalone_publisher', 'standalone_subscriber', 'composition_1pub_1sub',
             'composition_1pub_1sub', 'composition_1pub_2sub', 'composition_2pub_1sub']
    processes = []
    processes_name = []
    for p in psutil.process_iter(['name']):
        if p.info['name'] in nodes:
            processes.append(p)
            processes_name.append(p.name())
    return processes, processes_name

def test_node():
    node_processes,processes_name = get_all_processes()
    print(processes_name)
    while(1):
        cpu_percent = 0.0
        cpu_percents = []
        for p in node_processes:
            cur_cpu = p.cpu_percent(interval=2)
            cpu_percent = cpu_percent + cur_cpu
            cpu_percents.append(cur_cpu)
        memory_percent =0.0
        memory_percents = []
        for p in node_processes:
            cur_mem = p.memory_percent()
            memory_percent = memory_percent + cur_mem
            memory_percents.append(1.0*int(cur_mem*100)/100)
        # print(processes_name)
        # print(cpu_percents)
        # print(memory_percents)
        print("cpu:",cpu_percent,"memory:", memory_percent)
        print("------------------------------------------------")
        time.sleep(2)

test_node()
