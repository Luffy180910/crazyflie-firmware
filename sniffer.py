import usb.util

if __name__ == '__main__':
    vendor_id = 0x0483
    product_id = 0x5740

    dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)

    if dev is None:
        raise ValueError("cannot find usb device")

    dev.set_configuration()

    endpoint = dev[0][(0, 0)][0]

    try:
        while True:
            try:
                data = dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)
                if len(data) > 0:
                    print(data)
            except:
                pass
    except KeyboardInterrupt:
        pass

    usb.util.release_interface(dev, 0)
    usb.util.dispose_resources(dev)