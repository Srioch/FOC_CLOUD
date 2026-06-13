# Repository Guidelines

## Project Structure & Module Organization

This is an STM32F407 CubeMX firmware project for a dual-motor FOC gimbal controller. `Core/Inc` and `Core/Src` contain CubeMX-generated HAL setup and `main.c`; only edit generated files inside `USER CODE BEGIN/END` blocks. `Board/Inc` and `Board/Src` contain application modules such as `foc`, `foc_drive`, `pid`, `MYADC`, `Encoder`, `uart`, and OLED support. `Drivers/` is vendor CMSIS/HAL code and should be treated as read-only. `MDK-ARM/FOC_CLOUD.uvprojx` is the Keil project, with VS Code EIDE metadata under `MDK-ARM/EIDE/`. `tests/` holds compile-only API smoke tests and is not linked into the firmware image.

## Build, Test, and Development Commands

- `start MDK-ARM\FOC_CLOUD.uvprojx`: open the Keil MDK-ARM project; use Build (F7) and Flash (F8).
- `UV4.exe -b MDK-ARM\FOC_CLOUD.uvprojx -t FOC_CLOUD`: command-line Keil build when `UV4.exe` is on `PATH`.
- `FOC_CLOUD.ioc`: open in STM32CubeMX to change peripheral configuration, then regenerate code and re-check user sections.
- Compile `tests\*_compile_test.c` manually with the same include paths as the firmware after changing public headers.

## Coding Style & Naming Conventions

Use C99 style with 4-space indentation. Public APIs use a module prefix, for example `FocMotor_Init`, `FOC_Drive_ControlTick`, and `ADC_ChannelFilter_Process`; keep helpers `static` inside their `.c` file. Prefer fixed-width integer types for registers, flags, and wire data, and `float` for control-loop math. Keep each `Board/Src/<module>.c` paired with `Board/Inc/<module>.h`.

## Testing Guidelines

Tests are compile-time API checks, not hardware simulations. Name new tests `tests\<module>_api_compile_test.c` and include the public header first. Add or update a compile test whenever a public `Board/Inc` interface changes.

## Commit & Pull Request Guidelines

Recent commits use short, direct subjects describing the changed subsystem. Follow that pattern, for example `update adc sampling stability` or `fix foc drive limits`. PRs should summarize firmware behavior changes, list touched peripherals or timers, mention CubeMX regeneration, and include build or compile-test evidence.

## Agent-Specific Instructions

Do not edit `Drivers/` unless explicitly requested. For CubeMX-generated files, preserve user markers and avoid unrelated formatting churn.
