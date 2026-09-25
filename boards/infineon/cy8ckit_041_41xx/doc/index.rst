.. zephyr:board:: cy8ckit_041_41xx

Overview
********

The `CY8CKIT-041-41XX`_ PSOC™ 4100S Pioneer Kit is a low-cost development kit
for the PSOC™ 4100S family of Arm® Cortex®-M0+ microcontrollers, aimed at
evaluating the fourth-generation CAPSENSE™ solution.

Key features include 64 KB flash, 8 KB SRAM, an RGB LED, a user button, a
potentiometer and an onboard `KitProg`_ programmer/debugger with USB Micro-B
connectivity. A flexible CAPSENSE™ PCB with buttons, a trackpad and a
proximity loop is included.

Hardware
********

- **SoC:** PSOC™ 4100S (CY8C4146AZI-S433)
- **CPU:** Arm® Cortex®-M0+ at 48 MHz
- **Flash:** 64 KB
- **SRAM:** 8 KB
- **Peripherals:** TCPWM, SCB (UART/SPI/I2C), 12-bit SAR ADC
- **Touch Sensing:** CAPSENSE™ buttons, trackpad and proximity sensing
- **User I/O:** RGB LED (LED5), user button (SW2), potentiometer
- **Clock:** 32.768 kHz watch crystal (Y1)
- **Debug:** Onboard KitProg (SWD, UART and I2C bridges)
- **Power:** USB powered via Micro-B connector

For more information about the PSOC™ 4100S and CY8CKIT-041-41XX:

- `PSOC 4 SoC Website`_
- `CY8CKIT-041-41XX Board Website`_

Kit Contents
============

- PSOC™ 4100S Pioneer Kit board
- CAPSENSE™ flexible PCB with buttons, trackpad and proximity sensor
- USB-A to Micro-B cable

Supported Features
******************

.. zephyr:board-supported-hw::

Connections and IOs
*******************

The pin assignments below are those of the Infineon board support package
for the kit.

+--------------------+------------+----------------------------------------+
| Function           | Pin        | Notes                                  |
+====================+============+========================================+
| UART TX (console)  | P0.5       | SCB2, routed to the KitProg UART bridge|
+--------------------+------------+----------------------------------------+
| UART RX (console)  | P0.4       | SCB2, routed to the KitProg UART bridge|
+--------------------+------------+----------------------------------------+
| I2C SCL            | P3.0       | SCB1, routed to the KitProg I2C bridge |
+--------------------+------------+----------------------------------------+
| I2C SDA            | P3.1       | SCB1, routed to the KitProg I2C bridge |
+--------------------+------------+----------------------------------------+
| LED5 red           | P3.4       | Active low, ``led0``                   |
+--------------------+------------+----------------------------------------+
| LED5 green         | P2.6       | Active low, ``led1``                   |
+--------------------+------------+----------------------------------------+
| LED5 blue          | P3.6       | Active low, ``led2``                   |
+--------------------+------------+----------------------------------------+
| User button SW2    | P0.7       | Active low, ``sw0``                    |
+--------------------+------------+----------------------------------------+
| Potentiometer      | P2.4       | Analog input                           |
+--------------------+------------+----------------------------------------+
| SWDIO / SWDCK      | P3.2, P3.3 | Connected to the KitProg               |
+--------------------+------------+----------------------------------------+

The I2C bridge is described in the devicetree as ``i2c1`` but left disabled,
because SCB1 is the only block that reaches P3.0 and P3.1. Enable it from an
application overlay when needed.

UART Console
============

The Zephyr console is assigned to **SCB2** (``uart2``), which is routed
through the KitProg USB-UART bridge. Default settings are **115200 8N1**.

Building
********

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: cy8ckit_041_41xx
   :goals: build

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

The `CY8CKIT-041-41XX`_ includes an onboard programmer/debugger which can be
used to program and debug the PSOC™ 4100S Cortex-M0+ core.

Updating the KitProg Firmware
=============================

The kit ships with KitProg2 firmware. The OpenOCD configuration expects
`KitProg3`_, so update the onboard programmer once with the `fw-loader`_ tool
that comes with the ModusToolbox™ Programming Tools:

.. code-block:: shell

   fw-loader --update-kp3

Infineon OpenOCD Installation
=============================

The `ModusToolbox™ Programming Tools`_ package includes Infineon OpenOCD.
Alternatively, a standalone installation can be done by downloading the
`Infineon OpenOCD`_ release for your system and extracting the files to a
location of your choice.

.. note::

   Linux requires device access rights to be set up for KitProg3. This is
   handled automatically by the ModusToolbox™ Programming Tools installation.
   When doing a standalone OpenOCD installation, this can be done
   manually by executing the script ``openocd/udev_rules/install_rules.sh``.

Configuring a Console
=====================

Connect a USB cable from your PC to the KitProg USB Micro-B connector (J6) on
the `CY8CKIT-041-41XX`_. Use the serial terminal of your choice (minicom,
PuTTY, etc.) with the following settings:

- **Speed:** 115200
- **Data:** 8 bits
- **Parity:** None
- **Stop bits:** 1

Flashing
========

.. tabs::

   .. group-tab:: Windows

      One time, set the Infineon OpenOCD path:

      .. code-block:: shell

         west config build.cmake-args -- "-DOPENOCD=path/to/infineon/openocd/bin/openocd.exe"

      Build and flash the application:

      .. code-block:: shell

         west build -b cy8ckit_041_41xx -p always samples/hello_world
         west flash

   .. group-tab:: Linux

      One time, set the Infineon OpenOCD path:

      .. code-block:: shell

         west config build.cmake-args -- -DOPENOCD=path/to/infineon/openocd/bin/openocd

      Build and flash the application:

      .. code-block:: shell

         west build -b cy8ckit_041_41xx -p always samples/hello_world
         west flash

You should see the following message on the console:

.. code-block:: console

   *** Booting Zephyr OS build vX.Y.Z ***
   Hello World! cy8ckit_041_41xx

Debugging
=========

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: cy8ckit_041_41xx
   :goals: debug

Once the GDB console starts, you may set breakpoints and perform standard
GDB debugging on the PSOC™ 4100S Cortex-M0+ core.

References
**********

.. _CY8CKIT-041-41XX:
    https://www.infineon.com/evaluation-board/CY8CKIT-041-41XX

.. _PSOC 4 SoC Website:
    https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-4-32-bit-arm-cortex-m0-mcu/

.. _CY8CKIT-041-41XX Board Website:
    https://www.infineon.com/evaluation-board/CY8CKIT-041-41XX

.. _ModusToolbox™ Programming Tools:
    https://softwaretools.infineon.com/tools/com.ifx.tb.tool.modustoolboxprogtools

.. _Infineon OpenOCD:
    https://github.com/Infineon/openocd/releases/latest

.. _KitProg:
    https://github.com/Infineon/KitProg3

.. _KitProg3:
    https://github.com/Infineon/KitProg3

.. _fw-loader:
    https://github.com/Infineon/Firmware-loader
