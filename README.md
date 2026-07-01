# pic0rick 4-channel TX-short scan

This is a clean release folder for the current pic0rick 4-channel MAX14866 MUX experiment.

It is for the original Raspberry Pi Pico / RP2040 board. The ready-to-flash firmware is:

`firmware/tx_short_extra_gnd_rp2040.uf2`

## What This Runs

The live scan uses four piezo channels and measures all directed TX/RX pairs:

`TX1 -> RX2/RX3/RX4`  
`TX2 -> RX1/RX3/RX4`  
`TX3 -> RX1/RX2/RX4`  
`TX4 -> RX1/RX2/RX3`

Then it repeats continuously.

The current hardware setup uses:

- `TR1-TR4`: piezo channels.
- `TR5`: common short-to-GND path used after each transmit pulse.
- `TR6-TR8`: unused in this release.

The intent is to reduce TX ringdown/feedthrough by briefly connecting the TX side to ground after the pulse.

## Folder Contents

- `serve_array_tx_short.py`: live Flask browser interface.
- `host/run_array_test.py`: serial frame reader and helper code.
- `firmware/`: RP2040 firmware source.
- `firmware/tx_short_extra_gnd_rp2040.uf2`: compiled firmware for the original Pico / RP2040.
- `requirements.txt`: Python dependencies.

## Flash The Pico

Use the RP2040 UF2:

`firmware/tx_short_extra_gnd_rp2040.uf2`

Put the Pico into BOOTSEL mode, then copy this UF2 file to the mounted Pico drive.

## Run The Web Interface

From this folder:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python serve_array_tx_short.py
```

Open:

```text
http://127.0.0.1:5176/
```

## Default Settings

The live UI currently defaults to:

- DAC gain: `150`
- pon ns: `167`
- poff ns: `167`
- damp ns: `10000`
- samples/path: `2000`
- MUX settle us: `10`
- TX short delay us: `3`

The page sends 12 paths per scan. Each path has 2000 ADC samples, and the firmware sends the samples as packed 10-bit binary data.

## Host-To-Pico Command Flow

The browser does not directly control the Pico. The flow is:

1. The browser UI is defined inside `serve_array_tx_short.py`.

   The visible controls are in the HTML section of that file. The important defaults are stored in `SETTINGS_DEFAULTS`:

   ```python
   SETTINGS_DEFAULTS = {
       "dac": 150,
       "pon": 167,
       "poff": 167,
       "damp": 10000,
       "samples": 2000,
       "settle_us": 10,
       "blank_us": 0,
       "short_delay_us": 3,
       "short_hold_us": 50,
   }
   ```

2. When Start live is pressed, the browser calls:

   ```text
   POST /stream/start
   ```

   This endpoint is implemented in `serve_array_tx_short.py`.

3. `serve_array_tx_short.py` starts `hardware_worker(settings)`.

   This worker opens the Pico USB serial port:

   ```python
   SERIAL_BAUD = 921600
   serial.Serial(port, SERIAL_BAUD, timeout=2, write_timeout=2)
   ```

4. Before starting the stream, the server sends the DAC command:

   ```text
   write dac <dac_gain>
   ```

   Example:

   ```text
   write dac 150
   ```

5. Then the server sends one scan command:

```text
four common <samples> <pon_ns> <poff_ns> <damp_ns> <mux_settle_us> <rx_blank_us> <tx_short_delay_us> <tx_short_hold_us>
```

With the current defaults, this is approximately:

```text
four common 2000 167 167 10000 10 0 3 50
```

6. The Pico receives this text command in `firmware/main.c`.

   `process_command()` splits the first two words into a command key. For this release:

   ```c
   {"four common", adc_four_common_short_stream},
   ```

7. `adc_four_common_short_stream()` is implemented in `firmware/adc/adc.c`.

   It defines the 12 directed paths:

   ```c
   static const uint8_t tx_channels[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
   static const uint8_t rx_channels[12] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};
   ```

   These are zero-based inside firmware:

   - `0` means channel 1.
   - `1` means channel 2.
   - `2` means channel 3.
   - `3` means channel 4.

8. `adc_four_common_short_stream()` calls:

   ```c
   adc_mux_pairwise_short_stream(data, tx_channels, rx_channels, 12, true);
   ```

   The final `true` enables the common `TR5` short-to-GND path after each pulse.

9. The Pico continuously sends one binary frame per path.

   `serve_array_tx_short.py` reads those frames using `read_frame()` from `host/run_array_test.py`.

10. The server waits until it has all 12 frames, then publishes one complete scan to the browser through:

   ```text
   GET /stream/latest.bin
   ```

11. The browser unpacks the 10-bit samples and draws the 12 plots.

## Per-Path Firmware Sequence

For each directed path, for example `TX1 -> RX2`, the firmware does this:

The core function is:

```c
adc_mux_pairwise_short_stream()
```

in:

```text
firmware/adc/adc.c
```

### 1. Convert TX/RX channel into MAX14866 switch bits

The firmware uses these helper functions:

```c
static uint16_t mux_rx_bit(uint8_t channel)
{
    return (uint16_t)(1u << (channel * 2u));
}

static uint16_t mux_tx_bit(uint8_t channel)
{
    return (uint16_t)(1u << (channel * 2u + 1u));
}

static uint16_t mux_tx_short_bit(uint8_t channel)
{
    return mux_tx_bit((uint8_t)(channel + 4u));
}
```

For `TX1 -> RX2`:

- firmware TX channel is `0`
- firmware RX channel is `1`
- `TX1` switch bit is `0x0002`
- `RX2` switch bit is `0x0004`
- common `TR5` short bit is `0x0200`

So the masks are:

```text
mux_mask     = TX1 + RX2       = 0x0006
short_mask   = TX1 + RX2 + TR5 = 0x0206
release_mask = RX2 only        = 0x0004
```

### 2. Select the normal TX/RX path

   Example:

   ```text
   TX1 + RX2
   ```

   The firmware writes this mask using `max14866_write(mux_mask)`.

Code:

```c
max14866_write(mux_mask);
sleep_us(settle_us);
```

`max14866_write()` is in `firmware/max/max14866.c`:

```c
void max14866_write(uint16_t data)
{
    max14866_shift(data);
    max14866_latch();
}
```

This means:

- shift 16 bits into the MAX14866 shift register
- pulse LE low to copy the shift register into the output latch

### 3. Wait for MUX settling

   This is controlled by `MUX settle us`.

The current default is:

```text
MUX settle us = 10
```

The firmware also enforces a minimum:

```c
#define ARRAY_MIN_SETTLE_US 5
```

### 4. Preload the TX-short state before the pulse

   This next state keeps the RX path connected and also enables the common TX short path through `TR5`.

   Example:

   ```text
   TX1 + RX2 + TR5 short-to-GND
   ```

   This uses `max14866_shift(short_mask)`, which clocks the data in while LE is still high.

Code:

```c
max14866_shift(short_mask);
```

Important detail:

- `max14866_shift()` only clocks the new bits into the shift register.
- It does not latch the new state yet.
- LE remains high, so the active switches are still the normal TX/RX path.

This is done before the transmit pulse so the firmware does not need to send SPI clock data during the high-voltage pulse.

### 5. Start ADC capture, pulse generation, and timed LE

   The ADC and pulse timing are generated by PIO state machines in `firmware/adc/adc.pio`.

The C function is:

```c
run_adc_acquisition_tx_short()
```

in `firmware/adc/adc.c`.

This function:

1. clears the ADC PIO FIFO
2. sets up DMA for `sample_count`
3. loads PIO FIFO values for sample count, pulse width, pulse off time, and damp time
4. arms the timed LE latch PIO
5. starts all related PIO state machines together

Relevant code:

```c
dma_channel_configure(dma_chan, &dma_chan_cfg, buffer, &pio_adc->rxf[sm], sample_count, true);
load_adc_pulse_fifos(sample_count, pon_cycles, poff_cycles, damp_cycles);
arm_le_latch_pio(short_delay_us);
start_adc_pulse_le_sms_sync();
```

The synchronized start is important:

```c
pio_enable_sm_mask_in_sync(pio_adc, adc_pulse_le_sm_mask());
```

That starts:

- ADC sampling state machine
- pulse state machines
- LE latch state machine

at the same time.

### 6. PIO generates the ADC sampling and pulse waveforms

The PIO programs are in:

```text
firmware/adc/adc.pio
```

Relevant PIO programs:

- `.program adc`: samples the ADC pins
- `.program pulse1`: generates the first part of the pulse
- `.program pulse2`: generates the second/damping-related pulse timing
- `.program le_latch`: generates the timed LE pulse for the MAX14866

The ADC sample count comes from:

```c
pio_sm_put_blocking(pio_adc, sm, sample_count);
```

The pulse timing values are converted from ns to PIO cycles in `adc_mux_pairwise_short_stream()`:

```c
pon_ns / 8
poff_ns / 8
damp_ns / 8
```

This is because the pulse timing state machine runs with 8 ns cycle units.

### 7. Pulse LE after `TX short delay us`

   This is important: the LE timing is not done by slow Python or normal CPU timing. The firmware arms a dedicated PIO state machine with:

   ```c
   arm_le_latch_pio(short_delay_us);
   ```

   Then the ADC, pulse, and LE state machines are started together.

The PIO code is:

```pio
.program le_latch
.side_set 1

pull block side 1
mov x, osr side 1

le_delay:
    jmp x-- le_delay side 1

nop side 0 [15]
...
nop side 1
```

In MAX14866 logic:

- LE high: latch is frozen
- LE low: latch updates from the shift register

So the firmware shifts the `short_mask` early, then lets PIO pulse LE low at the desired time.

### 8. TR5 short-to-GND becomes active

   This enables the common `TR5` short-to-GND path after the pulse.

For `TX1 -> RX2`, this changes MAX14866 from:

```text
TX1 + RX2
```

to:

```text
TX1 + RX2 + TR5 short-to-GND
```

The goal is to bleed the TX-side residual charge/ringdown through the common short path.

### 9. Release TX and leave RX connected

   The release state is:

   ```text
   RX only
   ```

The firmware does this after:

```c
release_us = short_delay_us + short_hold_us + 2u;
```

Then:

```c
restore_le_gpio();
max14866_shift(release_mask);
max14866_latch();
```

For `TX1 -> RX2`, `release_mask = 0x0004`, so only RX2 remains connected.

### 10. Finish collecting ADC samples

The DMA waits until the ADC sample buffer is full:

```c
dma_wait_timeout(dma_chan, DMA_TIMEOUT_MS)
```

The current UI requests `2000` samples/path. At 60 MHz sample rate:

```text
2000 samples / 60 MHz = 33.33 us
```

### 11. Pack the 10-bit ADC samples and send one frame

The frame is written by:

```c
write_array_frame()
```

in `firmware/adc/adc.c`.

Each frame contains:

- magic bytes: `MUX10A1`
- mode
- TX channel
- RX mask
- sample count
- sequence number
- MUX mask
- packed 10-bit ADC samples

The firmware only calls `stdio_flush()` after the last path in a 12-path scan:

```c
write_array_frame(..., path + 1u == path_count);
```

This reduces USB overhead compared with flushing after every path.

## Binary Frame Format

The Python reader is in:

```text
host/run_array_test.py
```

It expects:

```python
MAGIC = b"MUX10A1"
HEADER = struct.Struct("<BBBHIH")
```

After the magic bytes, the header is:

| Field | Type | Meaning |
| --- | --- | --- |
| `mode` | `uint8` | scan mode, pairwise mode is `0` |
| `tx` | `uint8` | 1-based TX channel sent to the host |
| `rx_mask` | `uint8` | enabled RX channel mask |
| `count` | `uint16` | samples in this path |
| `sequence` | `uint32` | frame sequence counter |
| `mux_mask` | `uint16` | MAX14866 mask used for the frame |

The payload size is:

```python
payload_size = math.ceil(count * 10 / 8)
```

For `2000` samples:

```text
2000 * 10 / 8 = 2500 bytes/path
2500 bytes/path * 12 paths = 30000 bytes/scan
```

This is why the full 12-path live scan is mainly limited by USB serial throughput.

## Browser Rendering Flow

The browser receives one full scan from:

```text
GET /stream/latest.bin
```

The server response starts with:

```python
struct.pack("<IH", latest_cycle_id + 1, count)
```

Then it appends 12 packed traces.

In the browser JavaScript:

```javascript
const count = view.getUint16(4, true);
const packedBytes = Math.ceil(count * 10 / 8);
const samples = unpack10bit(packed, count);
draw(canvas, samples);
```

So the browser does not receive JSON waveform arrays. It receives compact binary data and unpacks it locally before drawing.

## Important Code Locations

- Serial command selection: `serve_array_tx_short.py`
- Command dispatch: `firmware/main.c`
- Main 4-channel loop: `firmware/adc/adc.c`, `adc_four_common_short_stream()`
- Path switching and TX short logic: `firmware/adc/adc.c`, `adc_mux_pairwise_short_stream()`
- Timed LE pulse setup: `firmware/adc/adc.c`, `arm_le_latch_pio()`
- ADC / pulse / LE PIO programs: `firmware/adc/adc.pio`
- MAX14866 bit shifting and latch helper: `firmware/max/max14866.c`

## Build Firmware

From the `firmware` folder:

```bash
./build.sh
```

This builds only the RP2040 firmware and updates:

```text
tx_short_extra_gnd_rp2040.uf2
```
