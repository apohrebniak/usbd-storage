# Firmware examples

Always build this example with `--release` because there is not enough space
for the debug symbols (in this configuration).

The example contains a partial fat image, which is written onto the flash.

The block size is 4096 because it's the block size of the flash, that way only
full flash blocks are written (and not only partially).

The build and flash the example:

- install https://github.com/raspberrypi/picotool 
- press the boot button on the Raspberry Pi Pico
- run `cargo r --release`

That should build and flash the device and after bootup it should show up as a usb-stick.

If you want to create a U2F file to manually copy/distribute:

- `cargo b --release`
- `picotool uf2 convert -t elf target/thumbv6m-none-eabi/release/rp2040_scsi_bbb rp2040_scsi_bbb.uf2`
