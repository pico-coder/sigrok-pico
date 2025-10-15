UF2s from the original release (circa 2023)
pico_original.uf2 - "Original" UF2 file of the release circa 2023.  
  Supports RP2040/PICO only, with 21 DIG and 3 analog and D0/D1 as UART.
pico_original_serial.uf2 - Build from https://github.com/pico-coder/sigrok-pico/pull/63/.    
  Supports RP2040/PICO only, with 21 DIG and 3 analog and D0/D1 as UART, with override for TUD serial configs

UF2s for the major 2025 rewrite - redo DMA controls, add RP2350 support.
Note: Current pulseview release always starts digital pin names at "D2".
So even though the dig26 and dig32 enable D0 and D1 as digital inputs, they will show up starting at D2.
  pico_baseline.uf2 - Baseline 21 DIG and 3 analog for RP2040/PICO
  pico_dig26.uf2 - RP2040/PICO with 26 digital for RP2040/PICO, including D0 and D1 as digital inputs.
  pico_dig32.uf2 - RP2040/PICO with 32 digital for RP2040/PICO, including D0 and D1 as digital inputs.
  pico2_baseline.uf2 - Baseline 21 DIG and 3 analog for RP2350/PICO2
  pico_dig26.uf2 - RP2040/PICO with 26 digital  RP2350/PICO2, including D0 and D1 as digital inputs.
  pico_dig32.uf2 - RP2040/PICO with 32 digital  RP2350/PICO2, including D0 and D1 as digital inputs.
