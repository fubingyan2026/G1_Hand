# AGENTS.md — G1_Hand

## Project

Embedded firmware for a robotic dexterous hand on **HPMicro HPM6E80** RISC-V MCU (`rv32imac`). SDK is bundled and localized — do NOT set `HPM_SDK_BASE` externally (CMakeLists.txt line 10 sets it).

## Build

```powershell
# Build directory already exists at repo root:
cd hpm6e00_ethercat_slave_flash_xip_debug
ninja
```

Board name is **`hpm6e00_ethercat_slave`** (forced by CMakeLists.txt — reconfiguring with any other `-DBOARD=` will fail). CMake generator is Ninja. Toolchain is `riscv32-unknown-elf-gcc`.

| Artifact | Path |
|----------|------|
| ELF | `output/demo.elf` |
| Binary | `output/demo.bin` |
| ASM | `output/demo.asm` |
| Map | `output/demo.map` |

Output from a fresh `ninja` at repo root:
```powershell
cd hpm6e00_ethercat_slave_flash_xip_debug
cmake .. -G Ninja
ninja
```

## Adding new files

**Counter-intuitive:** every new `.c` file must be registered in `CMakeLists.txt` via `sdk_app_src()`. Every new include path must be added to `target_include_directories(app PRIVATE ...)`. The `generate_ide_projects()` auto-discovers files only for IDE project files (Segger/IAR), not for Ninja builds.

## Architecture

```
user_app/                   — application layer, main entry
  app_main.c                — main()
  tasks/                    — cooperative poll-loop tasks
drivers/
  bsp/                      — board support (UART, GPIO HAL)
  device_driver/            — peripheral drivers (RS-485, LED)
  device/motor/             — motor control logic
middlewares/
  utils/                    — kfifo (ring buffer), clist (linked list)
  fsm/                      — generic finite state machine
  service/                  — service modules (e.g. LED FSM management)
board/hpm6e00_ethercat_slave/ — board-level pinmux and init
```

All drivers use HPM SDK peripheral HAL (`hpm_uart_drv`, `hpm_dmav2_drv`, etc.). There are no RTOS — tasks are cooperative poll-loop.

## Hardware

Peripheral pinout documented in `board_connect.md`. Key interfaces: 3× RS-485 (UART15, UART18, UART8), CAN4, EtherCAT, debug UART0 at 115200 8N1.

## Code conventions

- **Comments**: Doxygen (`@brief`, `@param`) for public API; Chinese for inline notes
- **License**: BSD-3-Clause header on all files
- **Naming**: `bsp_`, `drv_` prefixes; `snake_case` for functions/variables; `UPPER_CASE` for macros
- **Types**: `uint8_t`, `uint32_t`, `stdbool.h` — no proprietary typedefs

## Known out-of-date sources

`CLAUDE.md` contains stale claims (drive stubs, empty dirs, wrong board name). Trust `AGENTS.md` and executable sources (CMakeLists.txt, actual `.c` files) over that file.
