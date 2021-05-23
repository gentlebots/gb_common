# gb_behavior_tree

This package lets you to execute a behavior tree, in the same ways as it would be executed inside a PlanSys2 action.

This executable requires:
* A parameter file that contains a `plugin` param with an array of string with the BT node types to use. Example:
```
bt_executor_node:
  ros__parameters:
    plugins:
      - gb_attention_track_bt_node
      - gb_attention_scan_bt_node
```
* A parameter `bt_xml_file` with the full path pf the behavior tree to execute.
* **Optionaly** you can set different arguments, using argv's, that will be inserted in the blackboard of the behavior tree with the ids `arg1, arg2,...argN`.

Example of exeution form the command line:

```
fmrico@argo:~/ros/ros2/gentlebots_ws$ ros2 run gb_behavior_tree bt_executor robot run --ros-args --params-file install/gb_behavior_tree/share/gb_behavior_tree/config/default.yaml -p "bt_xml_file:=/home/fmrico/ros/ros2/gentlebots_ws/bt_example_1.xml"
[INFO] [1621755781.384320262] [bt_executor_node]: plugin: [gb_attention_track_bt_node]
[INFO] [1621755781.385680634] [bt_executor_node]: plugin: [gb_attention_scan_bt_node]
[INFO] [1621755781.387013942] [bt_executor_node]: set arg0 = [robot]
[INFO] [1621755781.387031478] [bt_executor_node]: set arg1 = [run]
[INFO] [1621755781.387041866] [bt_executor_node]: bt_xml_file: [/home/fmrico/ros/ros2/gentlebots_ws/bt_example_1.xml]
```
