# ROS2 to AWS IOT Core proxy

In short, this project is a basic MQTT proxy to redirect messages between ROS2 Humble and AWS MQTT.

Currently it packages up messages as json strings, it does not infer types on the ROS2 or AWS IOT end.

## Installation 

Close this repo into your `ros_workspace/src` folder.

You will need the awsiotSDK for this plugin.

```bash
python3 -m pip install awsiotsdk
```

Some environments have a newer version of `setuptools` than works with this project setup. I had to install version 58.2.0:

```bash
pip install setuptools==58.2.0
```

The build also uses colcon [installation here](https://colcon.readthedocs.io/en/released/user/installation.html)

## Build and configuration

### The AWS config

On the AWS side, an IOT Thing record with an attached key and policy is required for this to talk to AWS IOT.

Take particular notice of the policies as these can prevent or (more preferably) allow access to specific IOT topics.

Full details on how to do this [are here](https://github.com/aws-samples/aws-iot-robot-connectivity-samples-ros2)

### Configuration file

Update the endpoint, file paths and ClientID to the values from your IOT configuration.

**Note:**

- The `incomingTopics` is the map for AWS IOT core topics to subscribe and push to ROS2 topics.
- The `outgoingTopics` is the map for ROS2 topics to subscribe and push to AWS IOT core topics.
- You can create a race condition if you map the same topics in both directions, this wont hurt the IOT brokers on the AWS end but it will probably cause your robot issues!
- the `debug` field, if it's set to true, the console will be quite chatty - if it is not there or if it is not set to `"true"` it will not show proxy relay messages

```json
{
  "endpoint" : "ENDPOINT-ats.iot.REGION.amazonaws.com",
  "rootCAPath" : "/path_to/rootCA.crt",
  "certificatePath" : "/path_to/device.cert.pem",
  "privateKeyPath" : "/path_to/device.private.key",
  "port" : 8883,
  "clientID" : "CLIENTID",
  "debug" : "false",
  "incomingTopics": [
    [ "iotTopic1_src", "rostopic1_dst" ],
    [ "iotTopic2_src", "rostopic2_dst" ]
  ],
  "outgoingTopics": [
    [ "rostopic1_src", "iotTopic1_dst" ],
    [ "rostopic2_src", "iotTopic2_dst" ]
  ]
}
``` 

### Build

Use colcon to build the package:

```bash

cd ros2_ws dir
colcon build

```

## Launching the project

The project can be run using a launch file. By default, it will try to use an environment variable for the config file, if this environment variable doesn't exist, it defaults to `~/iot_params.json`.

```bash
# get humble setup
source /opt/ros/humble/setup.bash
# now your project 
source /path/to/ros2_ws/install/setup.bash

# location of the parameter file
AWS_IOT_PARAMETER_FILE="/path/to/my/configfile.json"

# launch
ros2 launch aws_iot_ros_proxy mqtt_proxy.launch.py
```

## using this in your own project

Clone the whole or move the `aws_iot_ros_proxy` project folder into your projects `src` folder.

Either run multiple launch files or integrate them in your projects.

## TODO

The configuration is currently a little bit tricky and does not allow for GreenGrass nucleus automatic authentication in docker.

- make the authentication simpler
- make the GG nucleus config work automatically
- rebuild it in rust (for speed and bragging rights)

## License

I like beer, so buy me a beer if you want and I'm not responsible for how you use this code.

```text
/* 
 * — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — 
 * “THE BEER-WARE LICENSE” (Revision 42):
 * <Spidey> wrote this file. As long as you retain this  
 * notice you can do whatever you want with this stuff. If we meet
 * some day, and you think this stuff is worth it, you can buy me
 * a beer in return.
 * — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — — 
 * Amendment 1: The author(s) of this code accept absolutely no 
 * liability for any damage or general bad things that may come as 
 * part of its use. Any use of this software is deemed an agreement 
 * to absolve the author(s) of any liability, culpability, 
 * durability and any other "(*)ability" (good or bad).
 */
```
