import axios from 'axios';

import { server_url } from '../Lib/networking';
import { make_axios_request } from './api_utils';

import { incoming_http_request } from '../Models/http';

export async function get_controls_readings(): Promise<incoming_http_request> {
    const api_url = `${server_url}/api/joystick/get-controls`;

    const axios_request = axios({
        method: 'post',
        url: api_url,
    });

    return await make_axios_request(axios_request);
}