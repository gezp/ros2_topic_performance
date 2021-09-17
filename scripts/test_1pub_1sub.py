#!/usr/bin/python3
from my_utils import * 

# test standalone nodes
def test_standalone_nodes(params = {},debug = False):
    # launch
    node_processes = launch_standalone_nodes(pub_num=1,sub_num=1,params=params)
    cpu_percent,memory_percent = performace_test(node_processes,time=10,enable_ouput=debug)
    print("cpu:",cpu_percent,"memory:", memory_percent)
    for p in node_processes:
        p.kill()

# test composition nodes
def test_composed_nodes(params = {},debug = False):
    # launch
    node_processes = launch_composed_nodes(params=params)
    cpu_percent,memory_percent = performace_test(node_processes,time=10,enable_ouput=debug)
    print("cpu:",cpu_percent,"memory:", memory_percent)
    for p in node_processes:
        p.kill()

# param
test_params={}
test_params["enable_output_delay"]=False
test_params["enable_output_address"]=False
test_params["rate"]=100
test_params["point_num"]=10000

killall()
# case1
print("case 1: standalone publisher and standalone subscriber")
test_standalone_nodes(test_params)
# case2
print("case 2: composed publisher and composed subscriber (with multiple single-threaded-executor)")
test_composed_nodes(test_params)