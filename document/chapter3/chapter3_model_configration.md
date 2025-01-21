# 第3章 仿真配置
AimRT_Mujoco_Sim 的设计初衷就是让用户仅通过配置文件来描述整个机器人结构（xml 配置文件）以及通信行为（yaml 配置文件）， 即可实现机器人仿真。本章将详细介绍 AimRT_Mujoco_Sim 的模型配置和通信配置。

## 3.1 模型配置（.xml）
模型配置（.xml）：描述机器人模型的结构、关节、传感器、驱动器等信息。在项目根目录下的`./src/examples/XXX/install/linux/bin/cfg/model`文件夹下。
主要用于：
  - 定义机器人的物理结构（连杆、关节）
  - 配置驱动器参数（电机、驱动器）
  - 设置传感器属性（IMU、接触传感器）
  - 指定物理参数（质量、惯性、摩擦系数）
  
具体使用请参考 [mujoco 官方文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

## 3.2 通信配置（.yaml）
通信配置（.yaml）：配置机器人仿真模块和其他模块的通信。在项目的`./src/examples/inverted_pendulum/install/linux/bin/cfg`文件夹下。
本项目使用 AimRT 作为中间件进行通信。其主要用于：
  - 用来订阅其他控制节点的控制指令，并将控制指令转化为 Mujoco 仿真场景中的动作指令
  - 用来发布 Mujoco 仿真场景中的状态信息，包括机器人当前的位置、姿态、速度等
  
关于其配置主要分为`aimrt`节点配置和`MujocoSimModule`节点配置。关于`aimrt`节点配详细使用方法可以参考[AimRT官方文档](https://docs.aimrt.org/index.html)，本小节主要介绍`MujocoSimModule`节点配置。

### 3.2.1 MujocoSimModule 节点配置

| 节点                           | 类型   | 是否可选 | 默认值 | 作用                                                            |
| ------------------------------ | ------ | -------- | ------ | --------------------------------------------------------------- |
| simulation_model_path          | string | 必选     | ""     | xml模型配置文件路径                                             |
| sim_executor                   | string | 必选     | ""     | Mujoco仿真引擎的执行器                                          |
| gui_executor                   | string | 必选     | ""     | Mujocox可视化渲染的执行器                                       |
| subscriber_options             | array  | 可选     | []     | 订阅者选项，用于订阅控制指令                                    |
| subscriber_options[i].topic    | string | 必选     | ""     | 订阅的控制指令的话题名称                                        |
| subscriber_options[i].type     | string | 必选     | ""     | 订阅的控制指令的类型                                            |
| subscriber_options[i].options  | map    | 可选     | -      | 订阅的控制指令的配置项，具体请参考[驱动器配置](#322-驱动器配置) |
| publisher_options              | array  | 可选     | []     | 发布机器人状态信息的选项                                        |
| publisher_options[i].topic     | string | 必选     | ""     | 发布者选项，用于发布状态信息                                    |
| publisher_options[i].frequency | number | 必选     | 1000   | 发布状态信息的频率(Hz) ，最大值为1000                           |
| publisher_options[i].executor  | string | 必选     | ""     | 发布者所绑定的执行器类型                                        |
| publisher_options[i].type      | string | 必选     | ""     | 发布的状态信息类型                                              |
| publisher_options[i].options   | map    | 可选     | -      | 发布状态信息的配置项  ，具体请参考[传感器配置](#323-传感器配置) |

使用注意点如下：
- subscriber_options 为数组类型， 每一个元素代表会开启一个订阅者用于订阅控制指令。
- publisher_options 为数组类型， 每一个元素代表会开启一个发布者用于发布状态信息。
- subscriber_options[i].type 为订阅的控制指令的类型，目前可选的有：
  - [`joint_actuator`](#joints-类驱动器选项joint_actuator)

- publisher_options[i].frequency 为发布状态信息的频率，单位为 Hz，最大值为1000。用户在设置时建议设置为 1000 的因数。
- publisher_options[i].type 为发布的状态信息类型，目前可选的有：
  - [`joint_sensor`](#joints-类传感器选项joint_sensor)
  - [`imu_sensor`](#imu-类传感器选项imu_sensor)

### 3.2.2 驱动器配置
#### joint 类驱动器选项（joint_actuator）

| 节点               | 类型   | 是否可选 | 默认值 | 作用                                    |
| ------------------ | ------ | -------- | ------ | --------------------------------------- |
| name               | string | 必选     | ""     | 该关节的名称                            |
| bind_joint         | string | 必选     | ""     | 该关节在 xml 中绑定的关节名称           |
| bind_actuator_type | string | 必选     | ""     | 该关节驱动器 在xml 中绑定的驱动器的类型 |
| bind_actuator_name | string | 必选     | ""     | 该关节驱动器在  xml中绑定的驱动器的名称 |

使用注意点如下：
- 同一选项下的每一个元素会通过一个订阅者进行订阅
- bind_joint 必须与 xml 文件中定义的名称一致， 即和下面代码块中 joint 字段一致
- bind_actuator_type 必须与 xml 文件中定义的名称一致，即和下面代码块中  的 motor 所处字段一致
- bind_actuator_name 必须与 xml 文件中定义的名称一致，即和下面代码块中  的 name 字段一致
```xml
  <actuator> 
    <motor name="testr_motor" joint="hinge1" gear="1" ctrlrange="-10000 10000"/> 
  </actuator>
```
### 3.2.3 传感器配置
#### joint 类传感器选项（joint_sensor）

| 节点                 | 类型   | 是否可选 | 默认值 | 作用                                |
| -------------------- | ------ | -------- | ------ | ----------------------------------- |
| name                 | string | 必选     | ""     | 该关节驱动器的名称                  |
| bind_joint           | string | 必选     | ""     | 该关节驱动器在 xml 中绑定的关节名称 |
| bind_jointpos_sensor | string | 必选     | ""     | 该关节在 xml 中绑定的位置传感器名称 |
| bind_jointvel_sensor | string | 必选     | ""     | 该关节在 xml 中绑定的速度传感器名称 |

使用注意点如下：
- 同一选项下的每一个元素会通过一个发布者进行发布
- bind_joint 必须与 xml 文件中定义的名称一致, 即和下面代码块中 joint 字段一致
- bind_jointpos_sensor 必须与 xml 文件中定义的名称一致， 即和下面代码块中 jointpos 的 name 字段一致
- bind_jointvel_sensor 必须与 xml 文件中定义的名称一致， 即和下面代码块中 jointvel 的 name 字段一致
```xml
  <sensor> 
    <jointpos name="test_jointpos_hinge1" joint="hinge1" noise="0.01"/>  
    <jointvel name="test_jointpos_hinge1" joint="hinge1" noise="0.01"/> 
  </sensor> 
```

#### imu 类传感器选项（imu_sensor）

| 节点               | 类型   | 是否可选 | 默认值 | 作用                                    |
| ------------------ | ------ | -------- | ------ | --------------------------------------- |
| name               | string | 必选     | ""     | IMU 传感器的名称标识                    |
| bind_site          | string | 必选     | ""     | IMU 传感器在 xml 中绑定的 site 名称     |
| bind_framequat     | string | 必选     | ""     | 绑定在 xml 中测量姿态四元数的传感器名称 |
| bind_gyro          | string | 必选     | ""     | 绑定在 xml 中测量角速度的传感器名称     |
| bind_framepos      | string | 必选     | ""     | 绑定在 xml 中测量位置的传感器名称       |
| bind_velocimeter   | string | 必选     | ""     | 绑定在 xml 中测量线速度的传感器名称     |
| bind_accelerometer | string | 必选     | ""     | 绑定在 xml 中测量线加速度的传感器名称   |

使用注意点如下：
- 同一选项下的每一个元素会通过一个发布者进行发布
- bind_site 必须与 xml 文件中定义的 site 名称一致
- bind_framequat 必须与 xml 文件中 framequat 的 name 字段一致
- bind_gyro 必须与 xml 文件中 gyro 的 name 字段一致
- bind_framepos 必须与 xml 文件中 framepos 的 name 字段一致
- bind_velocimeter 必须与 xml 文件中 velocimeter 的 name 字段一致
- bind_accelerometer 必须与 xml 文件中 accelerometer 的 name 字段一致
```xml
  <sensor>
    <framequat name="test-orientation" objtype="site" objname="imu" noise="0"/>
    <gyro name="test-angular-velocity" site="imu" noise="0.001"/>
    <framepos name="test-linear-pos" objtype="site" objname="imu"/>
    <velocimeter name="test-linear-vel" site="imu"/>
    <accelerometer name="test-linear-acceleration" site="imu" noise="0.001"/>
  </sensor>
```
### 3.2.4 示例
以下是一个简单的例子:

```yaml
aimrt:
  ...

MujocoSimModule:
  simulation_model_path: ./cfg/model/inverted_pendulum_flat.xml
  sim_executor: work_thread_pool
  gui_executor: work_thread_pool
  subscriber_options:
    - topic: /inverted_pendulum/joint_command
      type: joint_actuator
      options:
        joints:
          - name: center_joint
            bind_joint: hinge1
            bind_actuator_type: motor
            bind_actuator_name: center_motor

  publisher_options:
    - topic: /inverted_pendulum/joint_state
      frequency: 1000
      executor: work_thread_pool
      type: joint_sensor
      options:
        joints:
          - name: center_joint
            bind_joint: hinge1
            bind_jointpos_sensor: jointpos_hinge1
            bind_jointvel_sensor: jointvel_hinge1
          - name: edge_joint
            bind_joint: hinge2
            bind_jointpos_sensor: jointpos_hinge2
            bind_jointvel_sensor: jointvel_hinge2

```
在上述例子中， 我们创建了一个发布者，其将两个关节（center_joint 和 edge_joint）的位置和速度信息作为一组数据发布到 `/inverted_pendulum/joint_state`话题。同时，我们创建了一个订阅者，其订阅 `/inverted_pendulum/joint_command`话题，并将控制指令转化为 Mujoco 仿真场景中的动作指令。