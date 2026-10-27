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

## Startup resources a firmware has to provide

The drivers state what they need as Startup resources (the SDK's `kvasir/Hal/`): every SERCOM driver (I2C, queued
I2C, SPI, queued SPI, USART) and the CAN claim the generic clock their block counts - `Kvasir::Clocks::ClkSercomCore<n>`
/ `ClkCan<n>` (`Clocks.hpp`) - at their config's `clockSpeed`, and provide `Kvasir::Hal::Owns<Regs>` (one driver per
SERCOM, whatever its mode); the DmaBase provides its channels and the DMA users claim theirs. A firmware's
ClockSettings routes the generator in `peripheryClockInit()` and says so in the same struct:

```cpp
using Provides = brigand::list<Kvasir::Hal::Clk<Kvasir::Clocks::ClkSercomCore<3>, ClockSpeed / 32>>;
```

spelled with the same constant expression its driver config uses. Without the line, or with another rate, the image
does not build ("... nothing in any Startup list provides: Kvasir::Clocks::ClkSercomCore<3> ..."); there is no switch
to turn the check off.

Nothing builds here: the headers are compiled by whatever firmware includes them. A driver
with no user in either package - `CAN.hpp` had none until chip_atsamc21 existed - is only as
good as the last time someone compiled it.

## Flash stores (`NVMCTRL.hpp`)

| what | for |
| --- | --- |
| `Kvasir::Flash::CellStore<Kvasir::Nvm::Region<Base, Rows, Array>, T>` (SDK `Runtime/CellStore.hpp`) | **new code**: survives a power cut at any moment; `Nvm::Region` refuses a main-array range over the image (`Hal::FlashRegionBase::outsideImage()`, SDK `Hal/Flash.hpp`) |
| `Kvasir::EepromEmulator<Clock, T, rows, Nvm::Array>` | a value in a page ring over `rows` rows (two at least, `.eeprom`) |
| `Kvasir::FlashJournal<Clock, T, rows, Nvm::Array>` | a value written often: one page write per value |

The two `.eeprom` stores share their retry surface with CellEeprom (SDK `Runtime/Store.hpp`: `pending()`, `retry()`,
`link()`, `turn()` on `Hook::MainLoop`). Their `Nvm::Array` is the array the linker script puts `.eeprom` in (D21/C21
`chip.ld`: the RWWEE array, `Nvm::Array::rwwee`; the E5x has none). Their layout and record format are free to change:
an image reads only what an image of the same layout wrote, anything else is "no value" and the store starts from
`T{}`. Raw access: `Nvm::eraseRow<Nvm::Array::main>(addr)`, `Nvm::eraseRows`, `Nvm::Flash::writePage<Array>`.
