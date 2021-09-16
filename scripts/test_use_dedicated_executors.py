#!/usr/bin/python3
import time
from my_utils import * 

# test composition nodes
def test_composed_nodes(params = {},debug = False):
    # launch
    node_processes = launch_composed_nodes(params=params)
    cpu_percent,memory_percent = performace_test(node_processes,time=10,enable_ouput=debug)
    print("cpu:",cpu_percent,"memory:", memory_percent)
    for p in node_processes:
        p.kill()

# parameters
test_params={}
test_params["enable_output_delay"]=False
test_params["enable_output_address"]=False
test_params["publisher_num"] = 1
test_params["subscriber_num"] = 1
test_params["rate"]= 100
test_params["point_num"]= 10000

killall()
# case1
print("case 1: composed publisher and composed subscriber with a multi-threaded-executor")
test_params["use_dedicated_executors"]=False
test_composed_nodes(test_params)
# case2
print("case 2: composed publisher and composed subscriber with multiple single-threaded-executor")
test_params["use_dedicated_executors"]=True
test_composed_nodes(test_params)