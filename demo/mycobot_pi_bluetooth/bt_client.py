import sys
import bluetooth


if __name__ == "__main__":
    addr = None

    if len(sys.argv) < 2:
        print("No device specified. Searching all nearby bluetooth devices for the marscat_bt service...")
    else:
        addr = sys.argv[1]
        print("Searching for marscat_bt on {}...".format(addr))

    uuid = '26754beb-1bd0-4017-b341-154bed30b71a'
    service_matches = bluetooth.find_service(uuid=uuid, address=addr)

    if len(service_matches) == 0:
        print("Couldn't find the marscat_bt service.")
        sys.exit(0)

    first_match = service_matches[0]
    port = first_match["port"]
    name = first_match["name"]
    host = first_match["host"]

    print("Connecting to \"{}\" on {}".format(name, host))

    # Create the client socket
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((host, port))

    print("Connected. Type something...")
    while True:
        data = input()
        if not data:
            break
        sock.send(data)

    sock.close()
