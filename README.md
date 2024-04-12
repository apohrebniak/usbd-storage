usbd-storage
===========

Experimental USB Mass Storage implementation for [usb-device](https://crates.io/crates/usb-device).

# Subclasses
Implemented subclasses:
* `SCSI device` - number of SCSI commands is not exhaustive. Open a PR, if you want to add one.
* `USB Floppy Interface`

It is possible to implement a vendor specific subclass.

# Transports
Currently, only `Bulk Only` transport is implemented. It is possible to implement a vendor-specific transport.

# Features
This crate has a couple of opt-in features that all could be used independently.

| Feature | Description                           |
| ------- |---------------------------------------|
| `bbb` | Include Bulk Only Transport           |
| `scsi` | Include SCSI subclass                 |
| `ufi` | Include USB Floppy Interface sublcass |
| `defmt` | Enable logging via [defmt](https://crates.io/crates/defmt) crate |

# Examples
See [examples](examples)
