# ros2_topic_performance

In ROS 2, you can compose multiple nodes in a single process with the lower overhead and optionally more efficient communication (Intra Process Communication). you can find more details :

* [About Composition](https://docs.ros.org/en/galactic/Concepts/About-Composition.html)
* [Tutorials/Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)
* [Intra-Process-Communication](https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html)

this package aims to test performance (cpu and memory usage) between standalone nodes and composed nodes.

* this package only considers manual composition (compile-time composition)

## Test

i test on my local computer

* CPU:  Intel(R)  i7-8700  (Cores: 6 Threads: 12)
* RAM:  32GB

launch these nodes

```bash
# for Normal multi-process
ros2 launch ros2_topic_performance standalone_1pub_1sub.launch.py 
# for Manual composition
# ros2 launch ros2_topic_performance composition_1pub_1sub.launch.py 
```

test cpu and memory usage 

```bash
python scripts/test.py
```

* use python library `psutil` to collect data.

in addition, you can set message length and publishing rate in launch file, and the message type is `string` .

* message length: 100000 (default)
* rate : 50Hz (default)

## Test Result

### 1 pub and 1 sub

|                                                    | cpu(%) | memory(%) |
| -------------------------------------------------- | ------ | --------- |
| Normal multi-process                               | 1.8    | 0.098     |
| Manual composition                                 | 1.0    | 0.046     |
| Manual composition with intraprocess communication | 1.2    | 0.045     |
| Manual composition with zero copy (use unique_ptr) | 1.0    | 0.045     |

* `intraprocess communication` :  enable intraprocess communication in `Manual composition` by using `options.use_intra_process_comms(true)`.
* `zero copy` : publish message `unique_ptr` in `Manual composition with intraprocess communication`
  * in this case, `zero copy` performs similar to manual composition, but with smaller latency. (latency is not tested in this package)

### 1 pub and 2 sub

|                                                    | cpu(%) | memory(%) |
| -------------------------------------------------- | ------ | --------- |
| Normal multi-process                               | 8.0    | 0.145     |
| Manual composition                                 | 1.7    | 0.047     |
| Mixed composition                                  | 2.0    | 0.088     |
| Manual composition with intraprocess communication | 2.0    | 0.047     |
| Mixed composition with intraprocess communication  | 2.1    | 0.089     |
| Manual composition with zero copy (use unique_ptr) | 1.7    | 0.047     |
| Mixed composition with zero copy (use unique_ptr)  | 1.9    | 0.089     |

* `Mixed composition` :  Manual composition for 1pub and 1 sub, then create a standalone sub node.
