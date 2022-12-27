from typing import Any


def create_json_response(msg: str = '', data: Any = {}, success: bool = True) -> dict:
    ''' this function creates http response json in dict format '''

    return {
        'success': success,
        'msg': msg,
        'data': data,
    }
