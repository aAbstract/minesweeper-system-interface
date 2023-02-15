import axios from 'axios';
import { AxiosPromise } from 'axios';

import { incoming_http_request } from '../Models/http';

export async function make_axios_request(axios_request: AxiosPromise<any>): Promise<incoming_http_request> {
    let response = {} as incoming_http_request;

    try {
        const axios_response = await axios_request;

        response = axios_response.data as incoming_http_request;
    } catch (e) {
        const err_obj = e as any;

        response = {
            success: false,
            data: null,
            msg: `[ERROR] [${err_obj.code}]: ${err_obj.message}`,
        };
    }

    return response;
}