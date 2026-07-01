//!
//! \file       adc.c
//! \author     Abdelrahman Ali
//! \date       2024-01-20
//!
//! \brief      adc dac pio.
//!

//---------------------------------------------------------------------------
// INCLUDES
//---------------------------------------------------------------------------

#include "adc.h"

#include "adc.pio.h"
#include "../max/max14866.h"
#include "pico/stdio.h"
#include "hardware/sync.h"

//---------------------------------------------------------------------------
// GLOBAL VARIABLES
//---------------------------------------------------------------------------

PIO pio_adc;
uint sm;
uint offset;
uint sm2; // pulse GPIO11/12
uint offset2;
uint sm3; // pulse GPIO16/17
uint offset3;
uint sm_le; // timed MAX14866 LE latch
uint offset_le;
uint dma_chan;
dma_channel_config dma_chan_cfg;
uint16_t buffer[SAMPLE_COUNT];
static uint8_t binary_tx[7 + 2 + SAMPLE_COUNT * 2];

#define ARRAY_MODE_PAIRWISE 0
#define ARRAY_MIN_SETTLE_US 5
#define ARRAY_SAMPLE_RATE_MHZ 60u

static uint32_t sample_window_us_ceil(uint16_t count)
{
    return (count + ARRAY_SAMPLE_RATE_MHZ - 1u) / ARRAY_SAMPLE_RATE_MHZ;
}

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

static uint32_t adc_pulse_sm_mask(void)
{
    return ((1u << sm) | (1u << sm2) | (1u << sm3));
}

static uint32_t adc_pulse_le_sm_mask(void)
{
    return (adc_pulse_sm_mask() | (1u << sm_le));
}

static void reset_adc_pulse_sms_disabled(void)
{
    uint sms[] = { sm, sm2, sm3 };
    for (int i = 0; i < 3; i++)
    {
        pio_sm_set_enabled(pio_adc, sms[i], false);
        pio_sm_restart(pio_adc, sms[i]);
    }
}

static void reset_adc_pulse_le_sms_disabled(void)
{
    uint sms[] = { sm, sm2, sm3, sm_le };
    for (int i = 0; i < 4; i++)
    {
        pio_sm_set_enabled(pio_adc, sms[i], false);
        pio_sm_restart(pio_adc, sms[i]);
    }
}

static void load_adc_pulse_fifos(uint32_t sample_count, uint32_t pon_cycles, uint32_t poff_cycles, uint32_t damp_cycles)
{
    pio_sm_put_blocking(pio_adc, sm, sample_count);
    pio_sm_put_blocking(pio_adc, sm3, damp_cycles);
    pio_sm_put_blocking(pio_adc, sm2, pon_cycles);
    pio_sm_put_blocking(pio_adc, sm2, poff_cycles);
}

static void start_adc_pulse_sms_sync(void)
{
    pio_enable_sm_mask_in_sync(pio_adc, adc_pulse_sm_mask());
}

static void start_adc_pulse_le_sms_sync(void)
{
    pio_enable_sm_mask_in_sync(pio_adc, adc_pulse_le_sm_mask());
}

static uint32_t le_delay_cycles(uint32_t delay_us)
{
    const uint32_t cycles_per_us = PULSE_CLK / 1000000u;
    uint32_t cycles = delay_us * cycles_per_us;
    if (cycles > 4)
    {
        cycles -= 4;
    }
    return cycles;
}

static void arm_le_latch_pio(uint32_t delay_us)
{
    pio_sm_set_enabled(pio_adc, sm_le, false);
    pio_sm_restart(pio_adc, sm_le);
    pio_sm_clear_fifos(pio_adc, sm_le);
    gpio_set_function(MAX14866_SPI_LE, GPIO_FUNC_PIO0);
    pio_sm_put_blocking(pio_adc, sm_le, le_delay_cycles(delay_us));
}

static void restore_le_gpio(void)
{
    pio_sm_set_enabled(pio_adc, sm_le, false);
    gpio_init(MAX14866_SPI_LE);
    gpio_set_dir(MAX14866_SPI_LE, GPIO_OUT);
    gpio_put(MAX14866_SPI_LE, 1);
}

static void parse_ns_values(const char *data, uint32_t numbers[3])
{
    char *token;
    int i = 0;
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    token = strtok(data_copy, " ");
    while (i < 3)
    {
        if (token != NULL)
            numbers[i] = (atoi(token) / 8); // divide by 8 as one cycle is 8 nanoseconds
        else
            numbers[i] = 125; // default value
        i++;
        token = strtok(NULL, " ");
    }
}

static void run_adc_acquisition(uint32_t pon_cycles, uint32_t poff_cycles, uint32_t damp_cycles, bool verbose)
{
    pio_interrupt_clear(pio_adc, (1u<<0)|(1u<<1)|(1u<<2));
    pio_adc_clear_fifo();
    reset_adc_pulse_sms_disabled();
    memset(buffer, 0x00, SAMPLE_COUNT);
    if (verbose)
    {
        printf("Acquisition of %d samples started\n", SAMPLE_COUNT);
    }
    dma_channel_configure(dma_chan, &dma_chan_cfg, buffer, &pio_adc->rxf[sm], SAMPLE_COUNT, true);
    load_adc_pulse_fifos(SAMPLE_COUNT, pon_cycles, poff_cycles, damp_cycles);
    start_adc_pulse_sms_sync();
    if (!dma_wait_timeout(dma_chan, DMA_TIMEOUT_MS))
    {
        if (verbose)
        {
            printf("ADC timeout occured\n");
        }
    }
    if (verbose)
    {
        printf("Acquisition ended\n");
    }
}

static void run_adc_acquisition_rx_blank(
    uint32_t pon_cycles,
    uint32_t poff_cycles,
    uint32_t damp_cycles,
    uint32_t blank_us,
    bool verbose)
{
    pio_interrupt_clear(pio_adc, (1u<<0)|(1u<<1)|(1u<<2));
    pio_adc_clear_fifo();
    reset_adc_pulse_sms_disabled();
    memset(buffer, 0x00, SAMPLE_COUNT);
    if (verbose)
    {
        printf("Acquisition of %d samples started\n", SAMPLE_COUNT);
    }
    dma_channel_configure(dma_chan, &dma_chan_cfg, buffer, &pio_adc->rxf[sm], SAMPLE_COUNT, true);
    load_adc_pulse_fifos(SAMPLE_COUNT, pon_cycles, poff_cycles, damp_cycles);
    start_adc_pulse_sms_sync();

    if (blank_us > 0)
    {
        gpio_put(RX_BLANK_GPIO, 1);
        sleep_us(blank_us);
        gpio_put(RX_BLANK_GPIO, 0);
    }

    if (!dma_wait_timeout(dma_chan, DMA_TIMEOUT_MS))
    {
        gpio_put(RX_BLANK_GPIO, 0);
        if (verbose)
        {
            printf("ADC timeout occured\n");
        }
    }
    if (verbose)
    {
        printf("Acquisition ended\n");
    }
}

static void run_adc_acquisition_tx_short(
    uint16_t sample_count,
    uint32_t pon_cycles,
    uint32_t poff_cycles,
    uint32_t damp_cycles,
    uint32_t blank_us,
    uint32_t short_delay_us,
    uint32_t short_hold_us,
    uint16_t short_mask,
    uint16_t release_mask,
    bool verbose)
{
    pio_interrupt_clear(pio_adc, (1u<<0)|(1u<<1)|(1u<<2));
    pio_adc_clear_fifo();
    reset_adc_pulse_le_sms_disabled();
    if (sample_count == 0 || sample_count > SAMPLE_COUNT)
    {
        sample_count = SAMPLE_COUNT;
    }
    memset(buffer, 0x00, sample_count * sizeof(buffer[0]));
    if (verbose)
    {
        printf("Acquisition of %d samples started\n", sample_count);
    }
    dma_channel_configure(dma_chan, &dma_chan_cfg, buffer, &pio_adc->rxf[sm], sample_count, true);
    load_adc_pulse_fifos(sample_count, pon_cycles, poff_cycles, damp_cycles);
    arm_le_latch_pio(short_delay_us);

    uint32_t irq_state = save_and_disable_interrupts();
    if (blank_us > 0)
    {
        gpio_put(RX_BLANK_GPIO, 1);
    }
    start_adc_pulse_le_sms_sync();
    (void)short_mask;

    bool blank_active = (blank_us > 0);
    bool release_active = (short_hold_us > 0);
    uint32_t elapsed_us = 0;
    uint32_t release_us = short_delay_us + short_hold_us + 2u;

    while (blank_active || release_active)
    {
        uint32_t next_us = 1000000u;
        if (blank_active && blank_us < next_us)
        {
            next_us = blank_us;
        }
        if (release_active && release_us < next_us)
        {
            next_us = release_us;
        }
        if (next_us > elapsed_us)
        {
            busy_wait_us_32(next_us - elapsed_us);
            elapsed_us = next_us;
        }
        if (blank_active && elapsed_us >= blank_us)
        {
            gpio_put(RX_BLANK_GPIO, 0);
            blank_active = false;
        }
        if (release_active && elapsed_us >= release_us)
        {
            restore_le_gpio();
            max14866_shift(release_mask);
            max14866_latch();
            release_active = false;
        }
    }
    gpio_put(RX_BLANK_GPIO, 0);
    restore_interrupts(irq_state);

    if (!dma_wait_timeout(dma_chan, DMA_TIMEOUT_MS))
    {
        gpio_put(RX_BLANK_GPIO, 0);
        restore_le_gpio();
        if (verbose)
        {
            printf("ADC timeout occured\n");
        }
    }
    restore_le_gpio();
    if (verbose)
    {
        printf("Acquisition ended\n");
    }
}

static void write_binary_samples(uint16_t count)
{
    static const uint8_t magic[] = {'B', 'I', 'N', 'A', 'C', 'Q', '1'};
    uint8_t *out = binary_tx;

    if (count > SAMPLE_COUNT)
    {
        count = SAMPLE_COUNT;
    }

    memcpy(out, magic, sizeof(magic));
    out += sizeof(magic);
    *out++ = (uint8_t)(count & 0xFF);
    *out++ = (uint8_t)((count >> 8) & 0xFF);

    for (uint16_t i = 0; i < count; ++i)
    {
        uint16_t sample = ((buffer[i] >> 1) & 0x3FF);
        *out++ = (uint8_t)(sample & 0xFF);
        *out++ = (uint8_t)((sample >> 8) & 0xFF);
    }

    stdio_put_string((const char *)binary_tx, sizeof(magic) + 2 + count * 2, false, false);
    stdio_flush();
}

static void write_packed10_samples(uint16_t count)
{
    static const uint8_t magic[] = {'P', 'K', '1', '0', 'A', 'C', 'Q'};
    uint8_t *out = binary_tx;
    uint32_t bitbuf = 0;
    uint8_t bits = 0;

    if (count > SAMPLE_COUNT)
    {
        count = SAMPLE_COUNT;
    }

    memcpy(out, magic, sizeof(magic));
    out += sizeof(magic);
    *out++ = (uint8_t)(count & 0xFF);
    *out++ = (uint8_t)((count >> 8) & 0xFF);

    for (uint16_t i = 0; i < count; ++i)
    {
        uint16_t sample = ((buffer[i] >> 1) & 0x3FF);
        bitbuf |= ((uint32_t)sample << bits);
        bits += 10;
        while (bits >= 8)
        {
            *out++ = (uint8_t)(bitbuf & 0xFF);
            bitbuf >>= 8;
            bits -= 8;
        }
    }

    if (bits > 0)
    {
        *out++ = (uint8_t)(bitbuf & 0xFF);
    }

    stdio_put_string((const char *)binary_tx, (uint32_t)(out - binary_tx), false, false);
    stdio_flush();
}

static void write_array_frame(
    uint8_t mode,
    uint8_t tx_channel,
    uint8_t rx_mask,
    uint16_t mux_mask,
    uint16_t count,
    uint32_t sequence,
    bool flush_now)
{
    static const uint8_t magic[] = {'M', 'U', 'X', '1', '0', 'A', '1'};
    uint8_t *out = binary_tx;
    uint32_t bitbuf = 0;
    uint8_t bits = 0;

    memcpy(out, magic, sizeof(magic));
    out += sizeof(magic);
    *out++ = mode;
    *out++ = tx_channel + 1u;
    *out++ = rx_mask;
    *out++ = (uint8_t)(count & 0xFF);
    *out++ = (uint8_t)((count >> 8) & 0xFF);
    *out++ = (uint8_t)(sequence & 0xFF);
    *out++ = (uint8_t)((sequence >> 8) & 0xFF);
    *out++ = (uint8_t)((sequence >> 16) & 0xFF);
    *out++ = (uint8_t)((sequence >> 24) & 0xFF);
    *out++ = (uint8_t)(mux_mask & 0xFF);
    *out++ = (uint8_t)((mux_mask >> 8) & 0xFF);

    for (uint16_t i = 0; i < count; ++i)
    {
        uint16_t sample = ((buffer[i] >> 1) & 0x3FF);
        bitbuf |= ((uint32_t)sample << bits);
        bits += 10;
        while (bits >= 8)
        {
            *out++ = (uint8_t)(bitbuf & 0xFF);
            bitbuf >>= 8;
            bits -= 8;
        }
    }

    if (bits > 0)
    {
        *out++ = (uint8_t)(bitbuf & 0xFF);
    }

    stdio_put_string((const char *)binary_tx, (uint32_t)(out - binary_tx), false, false);
    if (flush_now)
    {
        stdio_flush();
    }
}

//---------------------------------------------------------------------------
// ADC INIT FUNCTION
//---------------------------------------------------------------------------

void pio_adc_init()
{
    pio_adc = pio0;
    sm = pio_claim_unused_sm(pio_adc, true);
    offset = pio_add_program(pio_adc, &adc_program);
    adc_program_init(pio_adc, sm, offset, PIN_BASE, ADC_CLK);
    sm2 = pio_claim_unused_sm(pio_adc, true);
    offset2 = pio_add_program(pio_adc, &pulse1_program);
    pulse1_program_init(pio_adc, sm2, offset2, GPIO11, PULSE_CLK);
    sm3 = pio_claim_unused_sm(pio_adc, true);
    offset3 = pio_add_program(pio_adc, &pulse2_program);
    pulse2_program_init(pio_adc, sm3, offset3, GPIO16, PULSE_CLK);
    sm_le = pio_claim_unused_sm(pio_adc, true);
    offset_le = pio_add_program(pio_adc, &le_latch_program);
    le_latch_program_init(pio_adc, sm_le, offset_le, MAX14866_SPI_LE, PULSE_CLK);
    restore_le_gpio();
    dma_chan = dma_claim_unused_channel(true);
    dma_chan_cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dma_chan_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_chan_cfg, false);
    channel_config_set_write_increment(&dma_chan_cfg, true);
    channel_config_set_dreq(&dma_chan_cfg, pio_get_dreq(pio_adc, sm, false));
    gpio_init(RX_BLANK_GPIO);
    gpio_set_dir(RX_BLANK_GPIO, GPIO_OUT);
    gpio_put(RX_BLANK_GPIO, 0);
    // pio_enable_sm_mask_in_sync(pio_adc, ((1u << sm) | (1u << sm2) | (1u << sm3)));
}

//---------------------------------------------------------------------------
// ADC DMA FUNCTION
//---------------------------------------------------------------------------
void pio_adc_dma()
{
    dma_channel_configure(dma_chan, &dma_chan_cfg, buffer, &pio_adc->rxf[sm], SAMPLE_COUNT, true);
    dma_channel_wait_for_finish_blocking(dma_chan);
}

//---------------------------------------------------------------------------
// ADC CLEAN FIFO FUNCTION
//---------------------------------------------------------------------------

void pio_adc_clear_fifo()
{
    pio_sm_clear_fifos(pio_adc, sm);
    pio_sm_clear_fifos(pio_adc, sm2);
    pio_sm_clear_fifos(pio_adc, sm3);
    pio_sm_clear_fifos(pio_adc, sm_le);
}


//---------------------------------------------------------------------------
// PULSE ADC DMA TIMEOUT
//---------------------------------------------------------------------------
bool dma_wait_timeout(uint chan, uint32_t ms) {
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (dma_channel_is_busy(chan)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) >= 0) {
            return false;
        }
    }
    return true;
}

//---------------------------------------------------------------------------
// PULSE ADC RESTART SMs
//---------------------------------------------------------------------------
void reset_all_sms() {
    uint sms[] = { sm, sm2, sm3, sm_le };
    for (int i = 0; i < 4; i++) {
        pio_sm_set_enabled(pio_adc, sms[i], false);
        pio_sm_restart(pio_adc, sms[i]);
        pio_sm_set_enabled(pio_adc, sms[i], true);
    }
}

//---------------------------------------------------------------------------
// PULSE ADC TRIGGER FUNCTION
//---------------------------------------------------------------------------
void pulse_adc_trigger(const char *data)
{
    uint32_t numbers[3];
    parse_ns_values(data, numbers);
    run_adc_acquisition(numbers[0], numbers[1], numbers[2], true);
}

//---------------------------------------------------------------------------
// ADC MAIN FUNCTION
//---------------------------------------------------------------------------

void adc(const char *data)
{
    printf("----------Start of ACQ----------\n");

    for (uint16_t i = 0; i < SAMPLE_COUNT; ++i)
    {
        printf("%X,", ((buffer[i] >> 1) & 0x3FF));
    }

    printf("\n-----------End of ACQ-----------\n");
}

//---------------------------------------------------------------------------
// ADC BINARY READ FUNCTION
//---------------------------------------------------------------------------
void adc_bin(const char *data)
{
    uint16_t count = SAMPLE_COUNT;

    int requested = atoi(data);
    if (requested > 0 && requested < SAMPLE_COUNT)
    {
        count = (uint16_t)requested;
    }

    write_binary_samples(count);
}

//---------------------------------------------------------------------------
// ADC LIVE BINARY ACQUIRE+READ FUNCTION
//---------------------------------------------------------------------------
void adc_live_bin(const char *data)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t values[4] = {SAMPLE_COUNT, 125, 125, 125};
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    for (int i = 0; i < 4 && token != NULL; ++i)
    {
        values[i] = (uint32_t)atoi(token);
        token = strtok(NULL, " ");
    }

    if (values[0] > 0 && values[0] < SAMPLE_COUNT)
    {
        count = (uint16_t)values[0];
    }

    run_adc_acquisition(values[1] / 8, values[2] / 8, values[3] / 8, false);
    write_binary_samples(count);
}

//---------------------------------------------------------------------------
// ADC STREAM BINARY ACQUIRE+READ FUNCTION
//---------------------------------------------------------------------------
void adc_stream_bin(const char *data)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t values[4] = {SAMPLE_COUNT, 125, 125, 125};
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    for (int i = 0; i < 4 && token != NULL; ++i)
    {
        values[i] = (uint32_t)atoi(token);
        token = strtok(NULL, " ");
    }

    if (values[0] > 0 && values[0] < SAMPLE_COUNT)
    {
        count = (uint16_t)values[0];
    }

    while (true)
    {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT)
        {
            break;
        }

        run_adc_acquisition(values[1] / 8, values[2] / 8, values[3] / 8, false);
        write_binary_samples(count);
    }
}

//---------------------------------------------------------------------------
// ADC STREAM PACKED 10-BIT ACQUIRE+READ FUNCTION
//---------------------------------------------------------------------------
void adc_stream_packed_bin(const char *data)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t values[4] = {SAMPLE_COUNT, 125, 125, 125};
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    for (int i = 0; i < 4 && token != NULL; ++i)
    {
        values[i] = (uint32_t)atoi(token);
        token = strtok(NULL, " ");
    }

    if (values[0] > 0 && values[0] < SAMPLE_COUNT)
    {
        count = (uint16_t)values[0];
    }

    while (true)
    {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT)
        {
            break;
        }

        run_adc_acquisition(values[1] / 8, values[2] / 8, values[3] / 8, false);
        write_packed10_samples(count);
    }
}

//---------------------------------------------------------------------------
// TWO-CHANNEL ALTERNATING MUX STREAM FUNCTION
//---------------------------------------------------------------------------
static void adc_mux_pairwise_stream(
    const char *data,
    const uint8_t *tx_channels,
    const uint8_t *rx_channels,
    uint8_t path_count)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t pon_ns = 100;
    uint32_t poff_ns = 100;
    uint32_t damp_ns = 10;
    uint32_t settle_us = 8;
    uint32_t blank_us = 7;
    uint32_t sequence = 0;
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    if (token != NULL)
    {
        int requested = atoi(token);
        if (requested > 0 && requested <= SAMPLE_COUNT)
        {
            count = (uint16_t)requested;
        }
        token = strtok(NULL, " ");
    }
    if (token != NULL) { pon_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { poff_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { damp_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { settle_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { blank_us = (uint32_t)atoi(token); }

    if (settle_us < ARRAY_MIN_SETTLE_US)
    {
        settle_us = ARRAY_MIN_SETTLE_US;
    }

    max14866_write(0);
    sleep_us(settle_us);

    while (true)
    {
        for (uint8_t path = 0; path < path_count; ++path)
        {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT)
            {
                max14866_write(0);
                return;
            }

            uint8_t tx = tx_channels[path];
            uint8_t rx = rx_channels[path];
            uint8_t rx_mask = (uint8_t)(1u << rx);
            uint16_t mux_mask = mux_tx_bit(tx) | mux_rx_bit(rx);
            max14866_write(mux_mask);
            sleep_us(settle_us);
            run_adc_acquisition_rx_blank(pon_ns / 8, poff_ns / 8, damp_ns / 8, blank_us, false);
            write_array_frame(ARRAY_MODE_PAIRWISE, tx, rx_mask, mux_mask, count, sequence++, path + 1u == path_count);
        }
    }
}

void adc_dual_stream(const char *data)
{
    static const uint8_t tx_channels[2] = {0, 1};
    static const uint8_t rx_channels[2] = {1, 0};
    adc_mux_pairwise_stream(data, tx_channels, rx_channels, 2);
}

void adc_four_stream(const char *data)
{
    static const uint8_t tx_channels[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    static const uint8_t rx_channels[12] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};
    adc_mux_pairwise_stream(data, tx_channels, rx_channels, 12);
}

static void adc_mux_pairwise_short_stream(
    const char *data,
    const uint8_t *tx_channels,
    const uint8_t *rx_channels,
    uint8_t path_count,
    bool common_short)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t pon_ns = 100;
    uint32_t poff_ns = 100;
    uint32_t damp_ns = 10;
    uint32_t settle_us = 8;
    uint32_t blank_us = 7;
    uint32_t short_delay_us = 3;
    uint32_t short_hold_us = 0;
    bool short_hold_given = false;
    uint32_t sequence = 0;
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    if (token != NULL)
    {
        int requested = atoi(token);
        if (requested > 0 && requested <= SAMPLE_COUNT)
        {
            count = (uint16_t)requested;
        }
        token = strtok(NULL, " ");
    }
    if (token != NULL) { pon_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { poff_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { damp_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { settle_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { blank_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { short_delay_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { short_hold_us = (uint32_t)atoi(token); short_hold_given = true; }
    if (!short_hold_given) { short_hold_us = sample_window_us_ceil(count); }

    if (settle_us < ARRAY_MIN_SETTLE_US)
    {
        settle_us = ARRAY_MIN_SETTLE_US;
    }

    max14866_write(0);
    sleep_us(settle_us);

    while (true)
    {
        for (uint8_t path = 0; path < path_count; ++path)
        {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT)
            {
                max14866_write(0);
                return;
            }

            uint8_t tx = tx_channels[path];
            uint8_t rx = rx_channels[path];
            uint8_t rx_mask = (uint8_t)(1u << rx);
            uint16_t rx_bit = mux_rx_bit(rx);
            uint16_t tx_bit = mux_tx_bit(tx);
            uint16_t short_bit = common_short ? mux_tx_short_bit(0) : mux_tx_short_bit(tx);
            uint16_t mux_mask = tx_bit | rx_bit;
            uint16_t short_mask = tx_bit | rx_bit | short_bit;
            uint16_t release_mask = rx_bit;
            max14866_write(mux_mask);
            sleep_us(settle_us);
            max14866_shift(short_mask);
            run_adc_acquisition_tx_short(
                count,
                pon_ns / 8,
                poff_ns / 8,
                damp_ns / 8,
                blank_us,
                short_delay_us,
                short_hold_us,
                short_mask,
                release_mask,
                false);
            write_array_frame(ARRAY_MODE_PAIRWISE, tx, rx_mask, short_mask, count, sequence++, path + 1u == path_count);
        }
    }
}

void adc_dual_short_stream(const char *data)
{
    static const uint8_t tx_channels[2] = {0, 1};
    static const uint8_t rx_channels[2] = {1, 0};
    adc_mux_pairwise_short_stream(data, tx_channels, rx_channels, 2, false);
}

void adc_four_short_stream(const char *data)
{
    static const uint8_t tx_channels[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    static const uint8_t rx_channels[12] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};
    adc_mux_pairwise_short_stream(data, tx_channels, rx_channels, 12, false);
}

void adc_four_common_short_stream(const char *data)
{
    static const uint8_t tx_channels[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    static const uint8_t rx_channels[12] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};
    adc_mux_pairwise_short_stream(data, tx_channels, rx_channels, 12, true);
}

static void adc_mux_pairwise_mux_blank_short_stream(
    const char *data,
    const uint8_t *tx_channels,
    const uint8_t *rx_channels,
    uint8_t path_count)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t pon_ns = 100;
    uint32_t poff_ns = 100;
    uint32_t damp_ns = 10;
    uint32_t settle_us = 8;
    uint32_t mux_blank_us = 7;
    uint32_t short_delay_us = 3;
    uint32_t short_hold_us = 0;
    bool short_hold_given = false;
    uint32_t sequence = 0;
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    if (token != NULL)
    {
        int requested = atoi(token);
        if (requested > 0 && requested <= SAMPLE_COUNT)
        {
            count = (uint16_t)requested;
        }
        token = strtok(NULL, " ");
    }
    if (token != NULL) { pon_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { poff_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { damp_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { settle_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { mux_blank_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { short_delay_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { short_hold_us = (uint32_t)atoi(token); short_hold_given = true; }
    if (!short_hold_given) { short_hold_us = sample_window_us_ceil(count); }

    if (settle_us < ARRAY_MIN_SETTLE_US)
    {
        settle_us = ARRAY_MIN_SETTLE_US;
    }

    max14866_write(0);
    sleep_us(settle_us);

    while (true)
    {
        for (uint8_t path = 0; path < path_count; ++path)
        {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT)
            {
                max14866_write(0);
                return;
            }

            uint8_t tx = tx_channels[path];
            uint8_t rx = rx_channels[path];
            uint8_t rx_mask = (uint8_t)(1u << rx);
            uint16_t rx_bit = mux_rx_bit(rx);
            uint16_t tx_bit = mux_tx_bit(tx);
            uint16_t common_tx_short_bit = mux_tx_short_bit(0);
            uint16_t mux_mask = tx_bit | rx_bit;
            uint16_t blank_mask = mux_blank_us > 0 ? tx_bit : mux_mask;
            uint16_t receive_short_mask = tx_bit | rx_bit | common_tx_short_bit;
            uint16_t release_mask = rx_bit;
            uint32_t latch_delay_us = mux_blank_us > 0 ? mux_blank_us : short_delay_us;

            max14866_write(blank_mask);
            sleep_us(settle_us);
            max14866_shift(receive_short_mask);
            run_adc_acquisition_tx_short(
                count,
                pon_ns / 8,
                poff_ns / 8,
                damp_ns / 8,
                0,
                latch_delay_us,
                short_hold_us,
                receive_short_mask,
                release_mask,
                false);
            write_array_frame(ARRAY_MODE_PAIRWISE, tx, rx_mask, receive_short_mask, count, sequence++, path + 1u == path_count);
        }
    }
}

void adc_four_mux_blank_common_short_stream(const char *data)
{
    static const uint8_t tx_channels[12] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3};
    static const uint8_t rx_channels[12] = {1, 2, 3, 0, 2, 3, 0, 1, 3, 0, 1, 2};
    adc_mux_pairwise_mux_blank_short_stream(data, tx_channels, rx_channels, 12);
}

void adc_four_noise_short_stream(const char *data)
{
    static const uint8_t tx_channels[16] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
    static const uint8_t rx_channels[16] = {5, 1, 2, 3, 5, 0, 2, 3, 5, 0, 1, 3, 5, 0, 1, 2};
    adc_mux_pairwise_short_stream(data, tx_channels, rx_channels, 16, true);
}

//---------------------------------------------------------------------------
// MUX DIAGNOSTIC STREAM FUNCTION
//---------------------------------------------------------------------------
void adc_mux_diagnostic_stream(const char *data)
{
    uint16_t count = SAMPLE_COUNT;
    uint32_t pon_ns = 100;
    uint32_t poff_ns = 100;
    uint32_t damp_ns = 10;
    uint32_t settle_us = 8;
    uint32_t blank_us = 7;
    uint32_t sequence = 0;
    char data_copy[strlen(data) + 1];
    strcpy(data_copy, data);

    char *token = strtok(data_copy, " ");
    if (token != NULL)
    {
        int requested = atoi(token);
        if (requested > 0 && requested <= SAMPLE_COUNT)
        {
            count = (uint16_t)requested;
        }
        token = strtok(NULL, " ");
    }
    if (token != NULL) { pon_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { poff_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { damp_ns = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { settle_us = (uint32_t)atoi(token); token = strtok(NULL, " "); }
    if (token != NULL) { blank_us = (uint32_t)atoi(token); }
    if (settle_us < ARRAY_MIN_SETTLE_US) { settle_us = ARRAY_MIN_SETTLE_US; }

    static const uint16_t masks[4] = {
        0x0000, // All MUX switches open.
        0x0002, // TX1 only: SW1.
        0x0004, // RX2 only: SW2.
        0x0006, // TX1 and RX2: SW1 + SW2.
    };

    max14866_write(0);
    sleep_us(settle_us);
    while (true)
    {
        for (uint8_t test = 0; test < 4; ++test)
        {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT)
            {
                max14866_write(0);
                return;
            }
            max14866_write(masks[test]);
            sleep_us(settle_us);
            run_adc_acquisition_rx_blank(pon_ns / 8, poff_ns / 8, damp_ns / 8, blank_us, false);
            write_array_frame(2, test, 0, masks[test], count, sequence++, true);
        }
    }
}

//---------------------------------------------------------------------------
// END OF FILE
//---------------------------------------------------------------------------
