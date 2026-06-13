# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dual-motor FOC (Field-Oriented Control) positioning system on STM32F407 (168 MHz). Two brushless motors ("up" / "down") drive a cloud-platform gimbal, with AS5600 magnetic encoders per axis and phase-current sensing via ADC1. An external vision computer sends target coordinates over USART2; the MCU runs cascaded PID position-speed-current loops at 1 kHz and outputs SVPWM over TIM2/TIM3.

## Build

- **IDE**: Keil MDK-ARM 5. Open `MDK-ARM/FOC_CLOUD.uvprojx`, click Build (F7), Flash (F8). EIDE (VS Code plugin) is also configured under `MDK-ARM/EIDE/`.
- **CubeMX**: `FOC_CLOUD.ioc` — regenerate code with STM32CubeMX, then restore user-app code between `USER CODE BEGIN/END` markers.
- **Compile-only tests**: The `tests/` directory contains `main()` functions that exercise the FOC API without hardware. They are **not linked into the firmware** — they only verify that public headers and APIs compile. Use the Keil compiler on these files manually to check API integrity after changing headers.

## Architecture

```
main.c (event loop)
  └── foc_drive.c          ← application facade: two-motor orchestration
        ├── foc.c          ← FOC motor model (Clarke/Park, SVPWM, PID cascade)
        ├── pid.c          ← generic PID, angle-wrapped PID, cloud-loop PID
        ├── MYADC.c        ← ADC1 α-filter + offset tracking + 3-phase sampler
        ├── Encoder.c      ← AS5600 I2C encoder (I2C1=up, I2C2=down)
        ├── uart.c         ← printf redirect, VisionData_t parser, command handler
        ├── uartdma.c      ← DMA ring-buffer TX
        └── kalman_filter.c
```

### Layers

| Directory | Role | Edit rules |
|---|---|---|
| `Core/` | CubeMX-generated HAL init (adc, tim, i2c, gpio, usart, dma) + `main.c` | Only write between `USER CODE BEGIN/END` markers |
| `Board/` | User application code (FOC, PID, encoder, ADC filters, UART) | Free editing |
| `Drivers/` | STM32 HAL + CMSIS (vendor, read-only) | Never edit |

### Control flow

1. `TIM1` overflows at 1 kHz → `HAL_TIM_PeriodElapsedCallback` in `main.c` sets three pending flags:
   - `control_tick_pending` (1 kHz, every interrupt)
   - `vision_task_pending` (200 Hz, every 5th)
   - `telemetry_task_pending` (100 Hz, every 10th)
2. `main()` while-loop polls flags and dispatches to `FOC_Drive_ControlTick()`, `FOC_Drive_VisionTask()`, and `FOC_Drive_TelemetryTask()`.
3. `FOC_Drive_Service()` runs every loop iteration, manages ADC offset tracking.
4. `UART_ProcessPendingCommand()` handles incoming serial commands each loop.

### PID cascade (innermost → outermost)

```
Vision/Position PID → Speed PID → Current (d/q) PID → SVPWM voltage → TIM2/TIM3 PWM
```

- Vision mode: external PC sends `VisionData_t` (ID, x, y, find) over USART2. The cloud-loop PID minimizes `(center - measure)` error, clamped to speed limit, then fed into speed and current loops.
- Position mode (`turn_flag != 0`): direct angle target.
- All modes zero the d-axis current (`id_target = 0`).

### ADC phase current sensing

Two-phase sensing (Ia, Ib) per motor, Ic computed as `-Ia - Ib`. ADC channels:
- Motor UP: PA5 (CH5), PA6 (CH6)
- Motor DOWN: PB0 (CH8), PB1 (CH9)

Per-channel α-filter with optional offset tracking (enabled when motor speed < 1 rad/s). Raw ADC → `ADC_ChannelFilter_Read()` → `PhaseVoltageSampler_Update()` → voltage-to-current conversion via `FOC_PHASE_CURRENT_V_TO_A_DIVISOR`.

### Key globals (extern in headers / static in foc_drive.c)

- `hadc1`, `htim1`, `htim2`, `htim3`, `huart1`, `huart2`, `hi2c1`, `hi2c2` — HAL handles (Core headers)
- `encoder_up`, `encoder_down` (Encoder.h)
- `pid_speed`, `pid_angle`, `pid_cloud_x`, `pid_cloud_y` (pid.h)
- `target_angle`, `turn_flag` (global, used by foc_drive.c)
- `vision_data` (uart.h — raw incoming frame)

## Pin map (quick reference)

| Peripheral | Pins | Motor |
|---|---|---|
| TIM2 PWM CH1-3 | PA0, PA1, PA2 | UP |
| TIM3 PWM CH2-4 | PA7, PC8, PC9 | DOWN |
| I2C1 | PB6 (SCL), PB7 (SDA) | UP encoder |
| I2C2 | PB10 (SCL), PB11 (SDA) | DOWN encoder |
| USART1 (telemetry) | PA9 (TX), PA10 (RX) | — |
| USART2 (vision) | PD5 (TX), PD6 (RX) | — |
| ADC1 IN5/6 | PA5, PA6 | UP phase A/B |
| ADC1 IN8/9 | PB0, PB1 | DOWN phase A/B |

## Code conventions

- C99; all `.c` files include only their matching `.h` plus dependency headers.
- `float` throughout the control path; `uint32_t` for registers and timestamps; `uint8_t` for flags.
- Public functions use `ModuleName_FunctionName` prefix; static helpers are file-local.
- PID `dt_s` must match the actual call period (1 ms). Changing TIM1 prescaler/period requires updating `FOC_LOOP_DT_S`.
- When adding a new `Board/` module, add its compile test under `tests/` to keep API surface visible.
