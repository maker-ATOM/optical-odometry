import usb.core
import json


def find_usb_devices(HID_class, Protocol_ID):
    devices = usb.core.find(find_all=True)
    mouse_device = []

    for device in devices:
        data_dict = {}

        class_id = device[0].interfaces()[0].bInterfaceClass
        protocol_id = device[0].interfaces()[0].bInterfaceProtocol

        if class_id == HID_class and protocol_id == Protocol_ID:
            data_dict["Vendor ID"] = device.idVendor
            data_dict["Product ID"] = device.idProduct
            data_dict["Bus"] = device.bus
            data_dict["Address"] = device.address
            mouse_device.append(data_dict)

    return mouse_device

def save_to_json(data, json_file_path):
    with open(json_file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)

if __name__ == "__main__":
    HID_class = 3
    Protocol_ID = 2
    json_file_path = "ids.json"

    mouse_device = find_usb_devices(HID_class, Protocol_ID)

    print(f"Found {len(mouse_device)} device(s) connected")

    if len(mouse_device) > 0:

        save_to_json(mouse_device, json_file_path)

        for device in mouse_device:
           print("---")
           for key, value in device.items():
                print(f"{{'{key}': {value}}},")