# Aratu Hexapod

The brains for the Aratu Hexapod.

## Developing

- Arduino should work out of the box with no changes

- For teensy 4.1 some changes are required:
  - The Makefile needs to include `Teensy.mk` instead of `Arduino.mk`
  - `ARDUINO_CORE_PATH`  in `Teensy.mk` needs to be updated to point to the `teensy4` core
  - `boards.txt` needs to be updated:
    - Set `teensy41.upload.maximum_data_size` and `teensy41.upload.maximum_size`
    - Add `-DARDUINO_TEENSY41` to `teensy41.build.flags.defs=`
    - Set `teensy41.build.mcu` to `imxrt1062_t41`

# Credits

- [Bare Arduino Project](https://github.com/ladislas/Bare-Arduino-Project)

# Copyright and License

Released under the MIT License.

See [LICENSE](LICENSE) for more details.
