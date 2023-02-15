import rospy
import std_msgs.msg as ros_std_msgs

import lib.log as log_man
import database.joystick_database_service as joystick_database_service

_MODULE_ID = 'ros.joystick_subscriber'

_la_sub: rospy.Subscriber = None
_ra_sub: rospy.Subscriber = None
_cmd_sub: rospy.Subscriber = None


def _left_analog_read_handler(msg: ros_std_msgs.String):
    joystick_database_service.set_joystick_left_analog(msg.data)


def _right_analog_read_handler(msg: ros_std_msgs.String):
    joystick_database_service.set_joystick_right_analog(msg.data)


def _cmd_read_handler(msg: ros_std_msgs.String):
    joystick_database_service.set_joystick_cmd(msg.data)


def subscriber_setup():
    global _la_sub
    global _ra_sub
    global _cmd_sub

    func_id = f"{_MODULE_ID}.subscriber_setup"

    la_topic_id = '/joystick_node/left_analog'
    ra_topic_id = '/joystick_node/right_analog'
    cmd_topic_id = '/joystick_node/cmd'

    try:
        _la_sub = rospy.Subscriber(
            la_topic_id, ros_std_msgs.String, _left_analog_read_handler)
        _ra_sub = rospy.Subscriber(
            ra_topic_id, ros_std_msgs.String, _right_analog_read_handler)
        _cmd_sub = rospy.Subscriber(
            cmd_topic_id, ros_std_msgs.String, _cmd_read_handler)

    except Exception as err:
        err_msg = f"error subscribing to joystick ROS topics: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)
        return False

    return True


def subscriber_shutdown():
    func_id = f"{_MODULE_ID}.subscriber_shutdown"

    try:
        _la_sub.unregister()
        _ra_sub.unregister()
        _cmd_sub.unregister()

    except Exception as err:
        err_msg = f"error shutting down module: {_MODULE_ID}: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)
        return False

    return True
