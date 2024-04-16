# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2024-04-16

### Fixed

- Read data into a buffer smaller than BBB's IO buffer (https://github.com/apohrebniak/usbd-storage/pull/6).
- Cannot fail command while reading data from host (https://github.com/apohrebniak/usbd-storage/pull/5).

### Changed

- Add `#[non_exhaustive]` attribute to ScsiCommand and UfiCommand

## [0.2.0] - 2024-02-05

- Support `usb-device@0.3`.

## [0.1.1] - 2024-02-05

- Restrict `usb-device` dependency to supported versions: < 0.3.

## [0.1.0] - 2023-04-14

- Initial release.

[unreleased]: https://github.com/apohrebniak/usbd-storage/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/apohrebniak/usbd-storage/releases/tag/v1.0.0
[0.2.0]: https://github.com/apohrebniak/usbd-storage/releases/tag/v0.2.0
[0.1.1]: https://github.com/apohrebniak/usbd-storage/releases/tag/v0.1.1
[0.1.0]: https://github.com/apohrebniak/usbd-storage/releases/tag/v0.1.0
