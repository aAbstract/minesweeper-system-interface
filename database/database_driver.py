import json

import lib.log as log_man


# module state
shmem_struct_json_obj = None


def _init_module():
    global shmem_struct_json_obj

    log_man.print_log('database.database_driver.init', 'DEBUG',
                    'initlizing database driver')

    try:
        with open('./database/shmem_struct.json', 'r') as f:
            shmem_struct_json_txt = f.read()
            shmem_struct_json_obj = json.loads(shmem_struct_json_txt)

    except Exception as err:
        err_msg = f"error initlizing database driver: {err}"
        log_man.print_log('database.database_driver', 'ERROR', err_msg)

        raise err


def save_shmem_struct_file():
    shmem_struct_json_txt = json.dumps(shmem_struct_json_obj, indent=2)

    log_man.print_log('database.database_driver.save_shmem_structfile', 'DEBUG',
                    'saving arch file')

    try:
        with open('./database/shmem_struct.json', 'w') as f:
            f.write(shmem_struct_json_txt)

    except Exception as err:
        err_msg = f"error writing archtecture file: {err}"
        log_man.print_log(
            'database.database_driver.save_shmem_struct_file', 'ERROR', err_msg)
        print(shmem_struct_json_txt)


def get_shmem_struct_json_obj():
    return shmem_struct_json_obj


def set_shmem_struct_json_obj(new_obj):
    global shmem_struct_json_obj

    shmem_struct_json_obj = new_obj


_init_module()
