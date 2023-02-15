import lib.log as log_man
import database.database_driver as database_driver
from models.internal.services import service_response

_MODULE_ID = 'database.joystick_service'


def get_joystick_controls():
    func_id = f"{_MODULE_ID}.get_joystick_controls"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        readings = data_obj['joystick_controls']

        return service_response(data={
            'joystick_controls': readings,
        })

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)


def set_joystick_left_analog(new_control: str):
    func_id = f"{_MODULE_ID}.set_joystick_left_analog"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        data_obj['joystick_controls']['left_analog'] = new_control
        database_driver.set_shmem_struct_json_obj(data_obj)

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)


def set_joystick_right_analog(new_control: str):
    func_id = f"{_MODULE_ID}.set_joystick_right_analog"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        data_obj['joystick_controls']['right_analog'] = new_control
        database_driver.set_shmem_struct_json_obj(data_obj)

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)


def set_joystick_cmd(new_control: str):
    func_id = f"{_MODULE_ID}.set_joystick_cmd"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        data_obj['joystick_controls']['cmd'] = new_control
        database_driver.set_shmem_struct_json_obj(data_obj)

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)
