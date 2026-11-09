# solution

Sketch for running your solution to tasks1 and task2.

```
ros2 launch solution solution.launch.py task:=task1
ros2 launch solution solution.launch.py task:=task2
```

* [solution.launch.py](launch/solution.launch.py) includes `assessment.launch.py` and starts this task's node. New launch arguments go in that launch file.
* [task1.py](solution/task1.py) and [task2.py](solution/task2.py) are sketches: both wait for the simulation to be ready.

The launch file sets `use_sim_time` for all of this package's nodes. Do not remove this directive or things may break.

Parameters expecting files should be passed using the package name, that is prefixed by, `package://solution/`, so that a file `example` in a folder `config` under package `solution`
is defined as `package://solution/config/example`. Do not use absolute paths.
