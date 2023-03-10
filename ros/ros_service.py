import os
import rospy
import time

import lib.log as log_man
import lib.settings as settings_man

import ros.sensors_subscriber as sensors_subscriber
import ros.cameras_subscriber as cameras_subscriber
import ros.joystick_subscriber as joystick_subscriber

_MODULE_ID = 'ros.ros_service'
_NODE_NAME = 'system_interface_ros_service_node'
_ROS_SUBSCRIBERS = {
    'sensors': sensors_subscriber,
    'cameras': cameras_subscriber,
    'joystick': joystick_subscriber,
}

service_active = True


def service_setup():
    func_id = f"{_MODULE_ID}.service_setup"

    _settings_obj = settings_man.get_settings()

    if not _settings_obj['system']['ROS_enable']:
        log_man.print_log(
            func_id, 'INFO', 'service setup apported: ROS integraion is disabled')
        return

    log_man.print_log(
        func_id, 'INFO', 'initializing system interface ROS service')

    # setup ROS master URI
    os.environ['ROS_MASTER_URI'] = f"http://{_settings_obj['networking']['main_cu']}:11311/"
    os.environ['ROS_IP'] = _settings_obj['networking']['main_cu']

    # init ros node
    try:
        rospy.init_node(_NODE_NAME, anonymous=True)

        for ros_sub in _ROS_SUBSCRIBERS.keys():
            log_man.print_log(
                func_id, 'INFO', f"registering ROS subscriber: {ros_sub}")
            _ROS_SUBSCRIBERS[ros_sub].subscriber_setup()

        log_man.print_log(
            func_id, 'INFO', f"done initializing ROS node: {_NODE_NAME}")

        return True

    except Exception as err:
        err_msg = f"error initializing ROS node: {_NODE_NAME}: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)
        return False


def serivce_shutdown():
    func_id = f"{_MODULE_ID}.serivce_shutdown"

    try:
        for ros_sub in _ROS_SUBSCRIBERS.keys():
            log_man.print_log(
                func_id, 'INFO', f"unregistering ROS subscriber: {ros_sub}")
            _ROS_SUBSCRIBERS[ros_sub].subscriber_shutdown()

            return True

    except Exception as err:
        err_msg = f"error shutting down ROS service: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)
        return False


# def service_loop():
#     func_id = f"{_MODULE_ID}.service_loop"

#     _settings_obj = settings_man.get_settings()

#     if not _settings_obj['system']['ROS_enable']:
#         log_man.print_log(
#             func_id, 'INFO', 'service loop apported: ROS integraion is disabled')
#         return

#     while service_active:
#         pass
