import usb.core
import json

json_file_path = "ids.json"

with open(json_file_path, "r") as json_file:
    ids = json.load(json_file)

vendor_id = ids[0]["Vendor ID"]
product_id = ids[0]["Product ID"]
Bus = ids[0]["Bus"]
Address = ids[0]["Address"]
dpi = ids[0]["DPI"]

print(f"{len(ids)} device(s) detected by Descriptor")


device = usb.core.find(idVendor=vendor_id, idProduct=product_id, bus = Bus, address = Address)

if device is None:
    print("Device Address mismatch ..!")
    exit()

ep = device[0].interfaces()[0].endpoints()[0]
i = device[0].interfaces()[0].bInterfaceNumber

device.reset()

if device.is_kernel_driver_active(i):
    device.detach_kernel_driver(i)

device.set_configuration()

eaddr = ep.bEndpointAddress
buffer_size = ep.wMaxPacketSize

dpi_x = 0
dpi_y = 0 

mm_x = 0
mm_y = 0

while True:
    try:
        data = device.read(eaddr, buffer_size, timeout = 10)

        dx = data[1]
        dy = data[2]

        dpi_x = dpi_x - dx if dx < 127 else dpi_x + (256 - dx)
        mm_x = dpi_x * 25.4 / dpi

        dpi_y = dpi_y - dy if dy < 127 else dpi_y + (256 - dy)
        mm_y = dpi_y * 25.4 / dpi

        print(f"x: {mm_x:.2f}, y: {mm_y:.2f}")
    except usb.core.USBTimeoutError as e:
        print(f"x: {mm_x:.2f}, y: {mm_y:.2f}")
    except usb.core.USBError as e:
        print("Device disconnected ...")
        exit()