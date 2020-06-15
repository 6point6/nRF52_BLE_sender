# Overview
Mbed OS5 firmware for the [Nordic Semiconductors NRF52840](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52840) Development Kit. It receives Bluetooth Low Energy advertising packets, encodes the packet using a [Protocol Buffer definition](./protobuf/adv_packet.proto), and sends the encoded data over the serial connection. 

# Flashing to a Board
First flash the [bootloader](./bootloader/0253_sam3u2c_mkit_dk_dongle_nrf5x_0x5000.bin) (turn on the board whilst holding the reset button), then the [regular](./bin/mbed-os-ble-sender.hex) or [debug](https://github.com/6point6/nRF52_BLE_sender/blob/master/bin/mbed-os-ble-sender_debug_on.hex) pre-built hex files.

# Building it yourself
Written using [Mbed Studio](https://os.mbed.com/studio/), which is the easiest way to build and flash the firmware.

# Protobuf
Followed the instructions in [this blog](https://techtutorialsx.com/2018/10/19/esp32-esp8266-arduino-protocol-buffers/) for building protobuf using a lightweight library called [nanopb](https://jpa.kapsi.fi/nanopb/), which is designed for embedded systems.

The protobuf classes `adv_packet.pb.c` and `adv_packet.pb.h` are included in the [protobuf](./protobuf) folder. To build them from scratch from the [the protobuf definition](./protobuf/adv_packet.proto) see the example (Windows) command [here](./protobuf/run_protoc.cmd).

If you want to use the same definition to build for another language, just strip the custom nanopb annotations (e.g. `[(nanopb).fixed_length = true, (nanopb).max_size = 30, (nanopb).max_count = 1]`) from the fields that have them. 
