from fastapi import APIRouter

import lib.log as log_man
import routes.route_config as route_config
import lib.http_response as http_resp_man
import database.joystick_database_service as joystick_database_service

# module config
_MODULE_ID = 'routes.joystick_api'
_ROOT_ROUTE = f"{route_config.API_ROOT}/joystick"

# module state
router = APIRouter()


@router.post(f"{_ROOT_ROUTE}/get-controls")
async def get_controls():
    func_id = f"{_MODULE_ID}.get_controls"

    log_man.print_log(func_id, 'DEBUG',
                      'received get controls readings request')

    db_service_resp = joystick_database_service.get_joystick_controls()

    if not db_service_resp.success:
        return http_resp_man.create_json_response(success=False, msg=db_service_resp.msg)

    return http_resp_man.create_json_response(data=db_service_resp.data)
