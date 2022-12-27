import json
import rospy
import std_msgs.msg as ros_std_msgs

import lib.log as log_util
import database.sensors_database_service as sensors_database_service

_MODULE_ID = 'ros.sensors_subscriber'


def _sensors_read_handler(msg: ros_std_msgs.String):
    readings_obj = json.loads(msg.data)
    sensors_database_service.set_sensors_readings(readings_obj)


def subscriber_setup():
    func_id = f"{_MODULE_ID}.subscriber_setup"
    topic_id = '/example_sensors_node/example_sensors_topic'

    try:
        rospy.Subscriber(topic_id, ros_std_msgs.String, _sensors_read_handler)

    except Exception as err:
        err_msg = f"error subscribing to ROS topic: {topic_id}: {err}"
        log_util.print_log(func_id, 'ERROR', err_msg)
        return False

    return True
