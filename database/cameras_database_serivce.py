import lib.log as log_man
import database.database_driver as database_driver
from models.internal.services import service_response

_MODULE_ID = 'database.cameras_database_serivce'


def get_cameras_frame():
    func_id = f"{_MODULE_ID}.get_cameras_frame"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        frame = data_obj['cameras_cluster_feed']['CAM_1']

        return service_response(data={
            'frame': frame,
        })

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)


def set_cameras_frame(new_frame):
    func_id = f"{_MODULE_ID}.set_cameras_frame"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        data_obj['cameras_cluster_feed']['CAM_1'] = new_frame
        database_driver.set_shmem_struct_json_obj(data_obj)

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)
