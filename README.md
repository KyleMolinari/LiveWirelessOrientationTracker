Kyle Molinari
May 10, 2020

**3D Axis Orientation:**

![3D Orientation Axes](https://github.com/KyleMolinari/LiveWirelessOrientationTracker/blob/master/Visualization/axesexample.png)

**Point Cloud Orientation:**
![Point Cloud Cat Example](https://github.com/KyleMolinari/LiveWirelessOrientationTracker/blob/master/Visualization/catexample.png)

**3D Axis and Point Cloud Orientation Overlay**
![3D Axis and Point Cloud Orientation Overlay](https://github.com/KyleMolinari/LiveWirelessOrientationTracker/blob/master/Visualization/bunnyexample.png)

**Hardware:**
      
- ESP32 Pico (or other Arduino compatable microcontroller with Bluetooth)

- BNO055

- Push Button

- ~10k Resistor

- Battery Pack

**Wiring Guide:**

- Connect BNO055 pins Vin to 3.3V, GND to GND, SDA to ESP32 Pico pin 21, SCL to ESP32 Pico pin 22, and ADR to GND

- Connect ESP32 Pico pin 37 to one side of the push button

- Connect the ~10k resistor from the same side of the push button to GND

- Connect the other side of the push button to 3.3V
