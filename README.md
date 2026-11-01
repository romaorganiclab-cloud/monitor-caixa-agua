# Monitor de caixa d'água

Arduino sketch that measures the water level in a rooftop tank, publishes it to Arduino IoT Cloud, and opens the inlet valve when the tank runs low.

An ultrasonic sensor reads the distance to the water every 2 seconds. The sketch turns that distance into a fill percentage, drives three status LEDs, and controls the valve relay. The cloud dashboard shows the live level and a 10-minute history. Around 7:00, if the overnight municipal supply did not fill the tank, the sketch writes a message the cloud can send as a push notification.

## Hardware

| Function | Pin | Notes |
| --- | --- | --- |
| Ultrasonic trigger | D7 | HC-SR04 style sensor, via NewPing |
| Ultrasonic echo | D6 | Max range configured at 120 cm |
| Green LED | D1 | Level at or above 70% |
| Yellow LED | D2 | Level from 30% up to 70% |
| Red LED | D3 | Level below 30% |
| Valve relay | D5 | Active low. Off state is high impedance, not driven high |

The relay pin is left as an input while the valve is closed so the output is floating. To open the valve the pin becomes an output driven low.

## Level calculation

The sensor is mounted above the water.

| Constant | Value | Meaning |
| --- | --- | --- |
| `ALTURA_SENSOR_BASE` | 105.5 cm | Sensor to the bottom of the tank |
| `ALTURA_UTIL_AGUA` | 80.5 cm | Usable water column, treated as 100% |
| `DISTANCIA_CHEIO` | 26.5 cm | Expected distance when the tank is full |

A reading equal to the sensor height is 0%. A reading of about 26.5 cm is 100%.

Each cycle takes the median of 7 pings. Timeouts and distances outside 20–106.5 cm are dropped. A jump larger than 8 percentage points is ignored unless it repeats for 3 cycles. Accepted readings pass through an exponential filter (20% of the new sample, at most 1.5 points per cycle) so the dashboard does not sawtooth.

## Valve

`abrir_valvula` can be changed from the dashboard. The sketch also operates the valve on its own:

| Action | Level | Extra conditions |
| --- | --- | --- |
| Auto open | 28% or below | 3 consecutive readings, and the valve has been closed for at least 60 seconds |
| Auto close | 52% or above | Only if this sketch opened the valve. 3 consecutive readings, and it has been open for at least 2 minutes |
| Safety close | 98% or above | Closes even a manually opened valve, after 2 consecutive readings |
| Re-arm | above 38% | Allows another automatic open after the valve is closed |

`alerta_ativo` is true while the level is at or below 28%.

## Cloud variables

Wi-Fi credentials and the device key are Arduino Cloud secrets (`SECRET_SSID`, `SECRET_OPTIONAL_PASS`, `SECRET_DEVICE_KEY`). They are not stored in this repository.

| Variable | Permission | Update | Use |
| --- | --- | --- | --- |
| `nivel_percentual` | Read | On change | Integer level for the gauge, LEDs, and valve logic |
| `nivel_serie` | Read | Every 600 seconds | Float series for the spline chart |
| `distancia_instantanea` | Read | On change | Water-column height in cm, derived from the filtered level |
| `abrir_valvula` | Read/write | On change | Dashboard switch and relay |
| `alerta_ativo` | Read | On change | Low-level alarm |
| `mensagem_manha` | Read | On change | Morning message consumed by a cloud trigger |

`nivel_percentual` only changes when the integer level changes, so a spline bound to it goes quiet while the tank is stable and looks denser in the afternoon. Point the chart at `nivel_serie`. That series is written once every 10 minutes. A 0.01% alternation keeps a point on the chart when the level is unchanged; at 100% the alternation steps down to 99.99%.

## Morning notification

Municipal water normally arrives between 23:00 and 03:00. From 07:00 to 07:30 the sketch checks the level once. If it is still below 95%, `mensagem_manha` is set to a dated message saying the utility probably did not supply water that night. A full tank leaves the string unchanged, so no notification is sent. Water use later in the day does not raise this alert.

The check uses `ArduinoCloud.getLocalTime()`, so the Thing timezone has to be the tank's local zone (America/Sao_Paulo for Brazil). Without that, 7:00 is evaluated in UTC.

The phone notification is an Arduino Cloud trigger, not code in the sketch:

1. Open [Triggers](https://app.arduino.cc/triggers) and create a trigger.
2. Condition: this Thing, variable `mensagem_manha`, when the value changes.
3. Action: send a push notification. Use `{variable.value}` as the body.
4. Enable notifications in the Arduino IoT Remote app. Push notifications require the Maker plan.

If the board is offline for the whole 07:00–07:30 window, that morning is skipped.

## Garmin Venu 3

The `garmin/` folder is a personal Connect IQ app for a Venu 3. It reads `nivel_percentual` and toggles `abrir_valvula` through the Arduino IoT Cloud API, the same variable the dashboard switch uses. Build and sideload steps are in [garmin/README.md](garmin/README.md).

## Libraries

- [ArduinoIoTCloud](https://github.com/arduino-libraries/ArduinoIoTCloud)
- [NewPing](https://github.com/teckel12/NewPing)
