# solution_interfaces

The solution's own messages and services, for when the standard ones and those of
`assessment_interfaces` do not fit.

1. Add a `.msg` file to `msg/`, or a `.srv` file to `srv/`, e.g. `msg/Target.msg`.
2. List it in `rosidl_generate_interfaces()` in [CMakeLists.txt](CMakeLists.txt).
3. If it uses types of another package, e.g. `geometry_msgs/Point`, add that package to
   `find_package()` and `DEPENDENCIES` there, and as a `<depend>` in [package.xml](package.xml).
4. Add `<exec_depend>solution_interfaces</exec_depend>` to the package.xml of the package that
   uses it, e.g. `solution`, and run `colcon build`.

Then, in Python: `from solution_interfaces.msg import Target`.

`msg/Example.msg` and `srv/Example.srv` are examples: replace or remove them, but keep at least one
interface, or the package does not build.
