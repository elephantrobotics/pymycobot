# Used for mycobot pi and mycobot_app

## Usage method

### 1.Make sure raspberry pi Bluetooth is available

The Ubuntu 20.04 system needs to restart the Bluetooth service first.

```shell
sudo systemctl restart bluetooth
```

### 2.Run Bluetooth server

```shell
python uart_peripheral_serial.py
```