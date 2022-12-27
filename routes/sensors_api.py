from fastapi import APIRouter

import lib.log as log_man
from models.internal.services import service_response
import routes.route_config as route_config
import lib.http_response as http_resp_man
import database.sensors_database_service as sensors_database_service

# module config
_MODULE_ID = 'routes.sensors_api'
_ROOT_ROUTE = f"{route_config.API_ROOT}/sensors"

# module state
router = APIRouter()


@router.post(f"{_ROOT_ROUTE}/get-reading")
async def get_reading():
    func_id = f"{_MODULE_ID}.get_reading"

    log_man.print_log(func_id, 'DEBUG',
                      'received get sensors readings request')

    db_service_resp = sensors_database_service.get_sensors_readings()

    if not db_service_resp.success:
        return http_resp_man.create_json_response(success=False, msg=db_service_resp.msg)

    return http_resp_man.create_json_response(data=db_service_resp.data)
