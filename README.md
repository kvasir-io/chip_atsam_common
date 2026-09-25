# chip_atsam_common

The peripheral drivers the Kvasir ATSAM chip packages share: SERCOM (I2C, queued I2C, SPI,
USART), EIC, DMAC, EVSYS, NVMCTRL and its flash-backed EEPROM, the CAN controller, the fuse
and serial-number readers, the start-up hook, and the EIC-driven push button and rotary
encoder.

It is a submodule of each chip package at `src/chip/atsam_common`, so the drivers include each
other as `"chip/atsam_common/<name>.hpp"` and reach their part-specific tables through
`"chip/<name>_Traits.hpp"`, which the chip package supplies. That is the same arrangement
`chip_rp_common` has with chip_rp2040 and chip_rp2350.

Packages using it:

| Package | Part |
| --- | --- |
| `chip_atsamd21` | ATSAMD21G17L |
| `chip_atsamc21` | ATSAMC21E17A |

Nothing builds here: the headers are compiled by whatever firmware includes them. A driver
with no user in either package - `CAN.hpp` had none until chip_atsamc21 existed - is only as
good as the last time someone compiled it.
