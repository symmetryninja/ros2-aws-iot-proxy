#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from awscrt import mqtt
from awsiot import mqtt_connection_builder
from std_msgs.msg import String
from functools import partial

class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')

        # Iot endpoint URL override
        self.declare_parameter('iot_endpoint', '')
        self.iot_endpoint = self.get_parameter('iot_endpoint').get_parameter_value().string_value
        self.get_logger().info('AWS IOT endpoint: %s!' % self.iot_endpoint)

        # Params for iot endpoint config
        self.declare_parameter('path_for_config', '')
        self.path_for_config = self.get_parameter('path_for_config').get_parameter_value().string_value
        self.get_logger().info('AWS IOT config file: %s!' % self.path_for_config)

        with open(self.path_for_config) as f:
          self.bridge_config = json.load(f)

        # detect whether the endpoint URL has been overridden
        if len(self.iot_endpoint) != 0:
            self.bridge_config['endpoint'] = self.iot_endpoint

        self.get_logger().info('Config we are loading is :\n{}'.format(self.bridge_config))
        # Build mqtt connection
        self.mqtt_conn = mqtt_connection_builder.mtls_from_path(
            endpoint=self.bridge_config['endpoint'],
            port= self.bridge_config['port'],
            cert_filepath= self.bridge_config['certificatePath'],
            pri_key_filepath= self.bridge_config['privateKeyPath'],
            ca_filepath= self.bridge_config['rootCAPath'],
            client_id= self.bridge_config['clientID'],
            http_proxy_options=None,
            clean_session=False,
            keep_alive_secs=30,
            on_connection_interrupted=self.on_connection_interrupted,
            on_connection_resumed=self.on_connection_resumed,
            on_connection_success=self.on_connection_success,
            on_connection_failure=self.on_connection_failure,
            on_connection_closed=self.on_connection_closed
        )
        connected_future = self.mqtt_conn.connect()
        connected_future.result()

        self.ros_publishers = {} # Publish IOT string messages to ROS topics
        self.mqtt_ros_map = {} # a map of iot topics to ros topics

        # iot-ros
        self.get_logger().info('iot-ros - setting up subs and pubs')
        self.init_mqtt_subs()

        self.ros_subscribers = {} # Subscribe to ROS topics and publish to IOT topics
        self.ros_mqtt_map = {}

        # ros-iot
        self.get_logger().info('ros-iot - setting up subs')
        self.init_ros_subs()

    # iot-ros subs
    def init_mqtt_subs(self):
        for iot_topic in self.bridge_config['iot_topics']:
            # pack out the mqtt_ros_map map
            self.mqtt_ros_map[iot_topic[0]] = iot_topic[1]
            # create a ros publisher keyed with the iot topic
            self.ros_publishers[iot_topic[0]] = self.create_publisher(String, iot_topic[1], 10)
            # subscribe iot topic
            self.get_logger().info('Subscribing to IOT topic "{}"...'.format(iot_topic[0] ))
            subscribe_future, packet_id = self.mqtt_conn.subscribe(
                topic=iot_topic[0],
                qos=mqtt.QoS.AT_LEAST_ONCE,
                callback=self.iot_on_message_received)
            subscribe_result = subscribe_future.result()
            self.get_logger().info('Subscribed with {}'.format(str(subscribe_result['qos'])))

    def iot_on_message_received(self, topic, payload, **kwargs):
        self.get_logger().info('Received message from IOT topic "{}": {}'.format(topic, payload))
        msg = String()
        msg.data = payload.strip().decode()
        self.ros_publishers[topic].publish(msg)
        self.get_logger().info('Publishing: "{}" to "{}" '.format(msg.data, self.mqtt_ros_map[topic]))

    # ros-iot methods
    def init_ros_subs(self):
        for ros_topic in self.bridge_config['ros_topics']: 
            self.ros_mqtt_map[ros_topic[0]] = ros_topic[1]
            self.get_logger().info('Subscribing to ROS2 topic "{}"...'.format(ros_topic[0]))
            self.ros_subscribers[ros_topic[0]] = self.create_subscription(
                String,
                ros_topic[0],
                partial(self.ros_listener_callback,ros_topic[1]),
                10)

    def ros_listener_callback(self, target_topic, msg):
        self.get_logger().info('Received message from ROS2 topic "{}": {}'.format(target_topic, msg))
        payload = msg.data
        self.mqtt_conn.publish(
                topic=target_topic,
                payload=payload,
                qos=mqtt.QoS.AT_LEAST_ONCE)

    ## helper: out object
    def dump(self, obj):
        for attr in dir(obj):
            print('obj.%s = %r' % (attr, getattr(obj, attr)))

# Stock callbacks
    # Callback when connection is accidentally lost.
    def on_connection_interrupted(self, connection, error, **kwargs):
        self.get_logger().info('Connection interrupted. error: {}'.format(error))

    # Callback when an interrupted connection is re-established.
    def on_connection_resumed(self, connection, return_code, session_present, **kwargs):
        self.get_logger().info('Connection resumed. return_code: {} session_present: {}'.format(return_code, session_present))

        if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
            self.get_logger().info('Session did not persist. Resubscribing to existing topics...')
            resubscribe_future, _ = connection.resubscribe_existing_topics()

            # Cannot synchronously wait for resubscribe result because we're on the connection's event-loop thread,
            # evaluate result with a callback instead.
            resubscribe_future.add_done_callback(self.on_resubscribe_complete)


    def on_resubscribe_complete(self, resubscribe_future):
        resubscribe_results = resubscribe_future.result()
        self.get_logger().info('Resubscribe results: {}'.format(resubscribe_results))

        for topic, qos in resubscribe_results['topics']:
            if qos is None:
                self.get_logger().info('Server rejected resubscribe to topic: {}'.format(topic))
                # exit


    # Callback when the connection successfully connects
    def on_connection_success(self, connection, callback_data):
        assert isinstance(callback_data, mqtt.OnConnectionSuccessData)
        self.get_logger().info('Connection Successful with return code: {} session present: {}'.format(callback_data.return_code, callback_data.session_present))

    # Callback when a connection attempt fails
    def on_connection_failure(self, connection, callback_data):
        assert isinstance(callback_data, mqtt.OnConnectionFailureData)
        self.get_logger().info('Connection failed with error code: {}'.format(callback_data.error))

    # Callback when a connection has been disconnected or shutdown successfully
    def on_connection_closed(self, connection, callback_data):
        self.get_logger().info('Connection closed')


def main(args=None):
    rclpy.init(args=args)

    mqtt_bridge = MqttBridge()

    rclpy.spin(mqtt_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mqtt_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
