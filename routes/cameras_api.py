from fastapi import APIRouter
from fastapi.responses import StreamingResponse

import lib.log as log_man
import routes.route_config as route_config
import database.cameras_database_serivce as cameras_database_serivce

# module config
_MODULE_ID = 'routes.cameras_api'
_ROOT_ROUTE = f"{route_config.API_ROOT}/cameras"

# module state
router = APIRouter()


def _video_streamer():
    while True:
        db_service_resp = cameras_database_serivce.get_cameras_frame()

        if not db_service_resp.success or isinstance(db_service_resp.data['frame'], str):
            continue

        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + bytearray(db_service_resp.data['frame']) + b'\r\n'


@router.get(f"{_ROOT_ROUTE}/cam_1")
async def get_cam_1_feed():
    func_id = f"{_MODULE_ID}.get_cam_1_feed"

    log_man.print_log(func_id, 'DEBUG',
                      'received CAM_1 streaming request')

    return StreamingResponse(_video_streamer(),  media_type='multipart/x-mixed-replace;boundary=frame')
