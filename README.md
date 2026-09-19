# Scissor lamp

This is the software of the scissor lamp project, the hardware is described [here] (https://blog.spope.fr/posts/scissor-lamp/).

The software is in charge of 2 things, adjust the lamp intensity based on the height of its deployment, or override this from a remote potentiometer.

## How it works

The system has two ESP32-C6 boards that talk to each other over **ESP-NOW**, Espressif's peer-to-peer protocol that runs on the WiFi radio. It needs no access point or network. Each board runs its own Arduino sketch:

| Sketch | Board | Role |
|---|---|---|
| [`ESP32/lamp/lamp.ino`](ESP32/lamp/lamp.ino) | Waveshare ESP32-C6-Pico | Receiver: drives the LED power supply |
| [`ESP32/remote/remote.ino`](ESP32/remote/remote.ino) | Seeed XIAO ESP32-C6 | Remote: reads a knob and sends the intensity wirelessly |

### Lamp (receiver)

- A **DFRobot GP8403 DAC** (DFR0971, I2C address `0x58`, SDA = GPIO 6, SCL = GPIO 7) outputs a **0–10 V dimming signal** to the LED power supply.
- The LED power supply stays off below about 8% of its input range, so a user intensity of 1–100% is mapped onto **0.8 V–10 V**. Only 0% outputs 0 V.
- A **potentiometer on the lamp** (A0) sets the intensity locally. The sketch takes 14 ADC samples, drops the 2 highest and 2 lowest, and averages the rest to filter noise.
- The lamp has two modes:
  - **`ONBOARD`** (default): the lamp's own knob controls the light.
  - **`REMOTE`**: the value received from the remote controls the light.
- **Taking control back:** in `REMOTE` mode the lamp checks its own knob every 500 ms. If the knob has moved by more than 5%, the lamp switches back to `ONBOARD`:
  - Knob turned **up**: the light follows the knob position straight away.
  - Knob turned **down**: the light starts from the value the remote had set and dims proportionally to 0 over the rest of the knob's travel, so the brightness doesn't jump.
- The ESP-NOW receive callback only stores the incoming value. The main `loop()` is the only code that writes to the DAC, so there is no I2C access from the WiFi task.

### Remote

- On boot, the remote pairs with the lamp using the lamp's hard-coded **MAC address** (`lampMacAdress` in `remote.ino`) and sends a `MODE = REMOTE` command.
- It then reads its own potentiometer every 200 ms (same filter as the lamp) and sends a new intensity only when the percentage changes.
- The on-board LED blinks each time a message is sent.

### Protocol

Each ESP-NOW message is a packed 3-byte struct:

```c
typedef struct __attribute__((packed)) {
  uint8_t verb;   // CMD_MODE = 1, CMD_INTENSITY = 2, CMD_POWER = 3
  int16_t value;  // mode (1 = ONBOARD, 2 = REMOTE) or intensity (0..100)
} espnow_msg_t;
```

The lamp ignores packets of the wrong size and values out of range. `CMD_POWER` is reserved and not implemented yet. The message definitions are duplicated in both sketches, so a change to one sketch must be copied to the other.

## Building and flashing

**Requirements**

- Arduino IDE 2.x or `arduino-cli`
- The **esp32 by Espressif** board package, core **3.x or later**. The ESP-NOW callback signatures used here (`esp_now_recv_info_t`, `esp_now_send_info_t`) don't compile on 2.x.
- The **DFRobot_GP8403** library, vendored in [`libraries/`](libraries/DFRobot_GP8403). Only the lamp needs it. `esp_now.h`, `WiFi.h` and `Wire.h` come with the ESP32 core.

To have the Arduino IDE pick up the vendored library, set this folder as the sketchbook location (*Preferences → Sketchbook location*). With `arduino-cli`, pass `--libraries libraries`.

**Board settings**

| Sketch | Arduino IDE board | `arduino-cli` FQBN |
|---|---|---|
| lamp | *ESP32C6 Dev Module*, **USB CDC On Boot = Enabled** | `esp32:esp32:esp32c6:CDCOnBoot=cdc` |
| remote | *XIAO_ESP32C6* (defaults are fine) | `esp32:esp32:XIAO_ESP32C6` |

> ⚠️ On the lamp, **USB CDC On Boot must be enabled**. With the default setting, `Serial` goes to the UART0 pins instead of USB: the lamp works but prints nothing to the USB serial monitor.

Example with `arduino-cli`:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc --libraries libraries ESP32/lamp
arduino-cli upload  --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc -p <port> ESP32/lamp

arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6 ESP32/remote
arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32C6 -p <port> ESP32/remote
```

**Pairing a new lamp board:** the remote sends to one fixed MAC address. If you replace the lamp's ESP32, read the new board's MAC address (for example with `WiFi.macAddress()`) and update `lampMacAdress` in `remote.ino`.

**Calibration:** `POT_MAX_COUNTS` is the ADC reading with the knob turned all the way up. Each sketch has its own value (3500 on the lamp, 3310 on the remote) because the two potentiometers are different. Measure it again if you change a potentiometer. Serial output runs at 115200 baud.

