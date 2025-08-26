# Wiring Connections

## Power Module

=== "PM06"

    [Holybro Micro Power Module (PM06)](https://docs.px4.io/main/en/power_module/holybro_pm06_pixhawk4mini_power_module.html)

    Information for QGroundControl Power setting:

    - Voltage Divider: 18.182
    - Amperes per Volt: 36.364

    ![PM06](../assets/pm06.jpg)

    Solder the motors to the power module and connect a wire to power the flight controller.


## Flight Controller

=== "Pixhawk 6C Mini"

    Pixhawk 6C Mini Connections:

    | **Wire/Device** | **Pixhawk Port** |
    |:------------|:-------------|
    | Power module | Power |
    | Motors | 1, 2 ,3 4  (I/O PWM OUT [MAIN] in order)  |
    | Telemetry radio | TELEM1 |
    | Companion computer | TELEM2 |
    | Receiver | PPM/SBUS RC |
    | GPS | gps2 |

    Holybro port diagram: [Pixhawk 6C Mini Ports | Holybro](https://docs.holybro.com/autopilot/pixhawk-6c-mini/pixhawk-6c-mini-ports)


=== "Pixracer Pro"

    Pixracer Pro Connections:

    | **Wire/Device** | **PixracerPro Port** |
    |:------------|:-------------|
    | Power module | Power |
    | Motors | 1, 2 ,3 4  (I/O PWM OUT [MAIN] in order)  |
    | Telemetry radio | serial 1 (Telem 1)  |
    | Companion computer | serial 2 (Telem 2)  |
    | Receiver/Radio | rc_input  |
    | GPS | gps2 |

    Resources:

    * Ardupilot documentation: [Pixracer Pro - Plane documentation](https://ardupilot.org/plane/docs/common-pixracer-pro.html)
    * mRo documentation: [Pixracer Pro | User Guides](https://docs.mrobotics.io/autopilots/pixracer-pro.html)
    * This might help: [:simple-youtube: PixRacer - Software, Firmware and Connections](https://www.youtube.com/watch?v=-GlnAPqbIrY&list=PLYsWjANuAm4p0Kwj4SfTymFsU-lR-FSVq&index=3)

    ![PixracerPro Top](../assets/pixracer-pro-top.png)
    /// caption
    Top view of PixracerPro (Notice the arrow points forward)
    ///

    ![PixracerPro Bottom](../assets/pixracer-pro-bottom.png)
    /// caption
    Bottom view of PixracerPro
    ///

## Companion Computer

=== "Raspberry Pi"

    The Raspberry Pi needs 5V and 2A. It can't get this from the flight controller because although the voltage is 5V the current is not sufficient.

    Needs a seperate power supply directly from the battery. Use a [UBEC](https://www.aliexpress.us/item/2251832349657924.html?src=google&pdp_npi=4%40dis%21USD%212.96%211.51%21%21%21%21%21%40%2158248093409%21ppc%21%21%21&src=google&albch=shopping&acnt=708-803-3821&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&albagn=888888&ds_e_adid=&ds_e_matchtype=&ds_e_device=c&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=en2251832349657924&ds_e_product_merchant_id=5520179106&ds_e_product_country=ZZ&ds_e_product_language=en&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=19108282527&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gad_campaignid=19108284222&gclid=CjwKCAjwmenCBhA4EiwAtVjzmhQNMy0Yy2JBLYHpBgHVqMRcT-3eK5jxPPSSM4vqKL3yOsGKVqFSuBoC6KIQAvD_BwE&gatewayAdapt=glo2usa) to step down the voltage to power the RPi.