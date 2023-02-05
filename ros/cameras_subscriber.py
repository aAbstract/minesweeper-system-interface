import base64
import cv2
import pickle
import rospy
import std_msgs.msg as ros_std_msgs

import lib.log as log_util
import database.cameras_database_serivce as cameras_database_serivce

_MODULE_ID = 'ros.cameras_subscriber'


def _cameras_ros_reader(msg: ros_std_msgs.String):
    input_bin_stream = msg.data.encode()

    # base64 decode
    decoded_bin_frame = base64.b64decode(input_bin_stream)

    # recover frame from binary stream
    frame = pickle.loads(decoded_bin_frame)

    cameras_database_serivce.set_cameras_frame(frame)


def subscriber_setup():
    func_id = f"{_MODULE_ID}.subscriber_setup"
    topic_id = '/camera_adapter_node/camera_feed'

    try:
        rospy.Subscriber(topic_id, ros_std_msgs.String, _cameras_ros_reader)

    except Exception as err:
        err_msg = f"error subscribing to ROS topic: {topic_id}: {err}"
        log_util.print_log(func_id, 'ERROR', err_msg)
        return False

    return True
