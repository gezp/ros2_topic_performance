#!/usr/bin/python3
import subprocess
import time
import psutil

def get_process(name):
    processes = []
    for p in psutil.process_iter(['name']):
        if p.info['name'] == name:
            processes.append(p)
    return processes

def run_launch_cmd(launch_file,params={}):
    cmd = "ros2 launch ros2_topic_performance " + launch_file
    for k,v in params.items():
        cmd += " "+k+":="+str(v)
    #print(cmd)
    subprocess.Popen([cmd],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,shell=True)

def killall():
    node_processes = get_process("standalone_publisher") + \
                     get_process("standalone_subscriber") + \
                     get_process("composed_npub_nsub")
    for p in node_processes:
        p.kill()

def launch_standalone_nodes(pub_num,sub_num,params):
    for i in range(pub_num):
        params["name"] = "publisher" + str(i)
        run_launch_cmd(launch_file="standalone_publisher.launch.py", params=params)
    for i in range(sub_num):
        params["name"] = "subscriber" + str(i)
        run_launch_cmd(launch_file="standalone_subscriber.launch.py", params=params)
    time.sleep(1)
    node_processes = get_process("standalone_publisher") + get_process("standalone_subscriber")
    return node_processes

def launch_composed_nodes(params):
    run_launch_cmd(launch_file="composed_npub_nsub.launch.py", params=params)
    time.sleep(1)
    node_processes = get_process("composed_npub_nsub")
    return node_processes

def performace_test(node_processes, time_sec = 5, enable_ouput = False):
    # do something
    cpu_sum = 0
    mem_sum = 0
    cnt = 0
    while(cnt<time_sec):
        cpu_percent = 0.0
        for p in node_processes:
            cpu_percent = cpu_percent + p.cpu_percent()
        memory_percent =0.0
        for p in node_processes:
            memory_percent = memory_percent + p.memory_percent()
        if enable_ouput:
            print("cpu:",cpu_percent,"memory:", memory_percent)
            print("------------------------------------------------")
        time.sleep(1.0)
        mem_sum += memory_percent
        cpu_sum += cpu_percent
        cnt=cnt+1
    return cpu_sum/cnt, mem_sum/cnt