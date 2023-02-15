import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

import lib.log as log_man
import lib.http_response as http_resp_man
import ros.ros_service as ros_service

from routes import sensors_api
from routes import cameras_api
from routes import joystick_api

server = FastAPI()

# middle wares
server.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=['*'],
    allow_headers=['*'],
)

# register routes
server.include_router(sensors_api.router)
server.include_router(cameras_api.router)
server.include_router(joystick_api.router)


@server.get('/')
async def root_get():
    log_man.print_log('main.root_get', 'DEBUG', 'hit main route')

    return http_resp_man.create_json_response(msg='server online')


if __name__ == '__main__':
    # setup ros service
    ros_service.service_setup()

    # start ros service loop
    # ros_service_thread = threading.Thread(target=ros_service.service_loop)
    # ros_service_thread.start()

    # start system interface web server
    uvicorn.run(server, host='127.0.0.1', port=8080)

    # shutdown ros service
    ros_service.serivce_shutdown()

    # stop ros service loop
    # ros_service.service_active = False
    # ros_service_thread.join()
