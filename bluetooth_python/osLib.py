import os


class OsLib:
    def __init__(self):
        pass

    def get_connected_devices(self):

        connected_devices = []

        bt_info_str = os.popen("system_profiler SPBluetoothDataType 2> /dev/null").read()
        bt_info_lst = bt_info_str.split("\n")

        for i, line in enumerate(bt_info_lst):
            if "Connected: Yes" in line:
                connected_device = bt_info_lst[i - 7].replace(":", "").replace("          ", "")
                connected_devices.append(connected_device)

        return connected_devices

    def is_bt_On(self):
        bt_info_str = os.popen("system_profiler SPBluetoothDataType 2> /dev/null").read()
        bt_info_lst = bt_info_str.split("\n")

        for line in bt_info_lst:
            if "Bluetooth Power: " in line:
                if "On" in line:
                    return True
                if "Off" in line:
                    return False
                else:
                    raise ValueError(
                        'Could not find setting of bluetooth power. \nVerify with system_profiler SPBluetoothDataType that bluetooth works correctly.')

        raise ValueError(
            'Did not find bluetooth power.\n Verify with system_profiler SPBluetoothDataType that bluetooth works correctly.')
