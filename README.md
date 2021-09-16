# ros2_topic_performance

In ROS 2, you can compose multiple nodes in a single process with the lower overhead and optionally more efficient communication (Intra Process Communication). you can find more details :

* [About Composition](https://docs.ros.org/en/galactic/Concepts/About-Composition.html)
* [Tutorials/Composition](https://docs.ros.org/en/galactic/Tutorials/Composition.html)
* [Intra-Process-Communication](https://docs.ros.org/en/galactic/Tutorials/Intra-Process-Communication.html)

this package aims to test performance (cpu and memory usage) between standalone nodes and composed nodes.

* this package only considers manual composition (compile-time composition), it's performance should as same as run-time composition

## Quickly Test

i test on my local computer

* CPU:  Intel(R)  i7-8700  (Cores: 6 Threads: 12)
* RAM:  32GB

test `standalone publisher and subscriber` vs `composed publisher and subscriber`

```bash
python scripts/test_1pub_1sub.py
```

* a sample result is avialable : [result](scripts/test_1pub_1sub_result.txt)

test `a multi-threaded-executor` vs `multiple single-threaded-executor` for composition

```bash
test_use_dedicated_executors.py
```

* a sample result is avialable : [result](scripts/test_use_dedicated_executors_result.txt)

## Test Method

python library `psutil` is used to collect data (cpu and memory usage), and `scripts/my_utils.py` is a useful tool to launch nodes and collect performance's data in python.

the message type i used in is `geometry_msgs::msg::PolygonStamped` (contains an array of points).

* points number: 1000 (default, 12 Byte for each point)
* rate : 1Hz (default)


## Test Result (out of date)

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
