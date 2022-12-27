import lib.log as log_man
import database.database_driver as database_driver
from models.internal.services import service_response

_MODULE_ID = 'database.sensors_database_service'


def get_sensors_readings():
    func_id = f"{_MODULE_ID}.get_sensors_readings"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        readings = data_obj['sensors_cluster_readings']

        return service_response(data={
            'sensors_readings': readings,
        })

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)


def set_sensors_readings(new_readings: dict):
    func_id = f"{_MODULE_ID}.set_sensors_readings"

    data_obj = database_driver.get_shmem_struct_json_obj()

    try:
        data_obj['sensors_cluster_readings'] = new_readings
        database_driver.set_shmem_struct_json_obj(data_obj)

    except Exception as err:
        err_msg = f"database device indexing error: {err}"
        log_man.print_log(func_id, 'ERROR', err_msg)

        return service_response(success=False, msg=err_msg)
