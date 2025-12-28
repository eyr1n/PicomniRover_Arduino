#include <hardware/clocks.h>
#include <hardware/irq.h>

#include "FeetechBus.h"
#include "feetech_bus.pio.h"

static FeetechBus *feetech_bus_map[NUM_PIOS][NUM_PIO_STATE_MACHINES];

static bool pio_can_add_two_programs(PIO pio, const pio_program_t *program0, const pio_program_t *program1) {
  if (!pio_can_add_program(pio, program0)) {
    return false;
  }
  uint offset0 = pio_add_program(pio, program0);
  bool success = pio_can_add_program(pio, program1);
  pio_remove_program(pio, program0, offset0);
  return success;
}

static bool pio_claim_two_free_sms_and_add_programs(const pio_program_t *program0, const pio_program_t *program1,
                                                    PIO *pio, uint *sm_base, uint *offset0, uint *offset1) {
#if defined(PICO_RP2350)
  static PIO pios[] = {pio0, pio1, pio2};
#elif defined(PICO_RP2040)
  static PIO pios[] = {pio0, pio1};
#endif
  for (uint i = 0; i < count_of(pios); ++i) {
    PIO p = pios[i];
    if (!pio_can_add_two_programs(p, program0, program1)) {
      continue;
    }
    for (uint sm = 0; sm + 1 < NUM_PIO_STATE_MACHINES; ++sm) {
      if (!pio_sm_is_claimed(p, sm) && !pio_sm_is_claimed(p, sm + 1)) {
        *pio = p;
        *sm_base = sm;
        *offset0 = pio_add_program(p, program0);
        *offset1 = pio_add_program(p, program1);
        pio_sm_claim(p, sm);
        pio_sm_claim(p, sm + 1);
        return true;
      }
    }
  }
  return false;
}

FeetechBus::FeetechBus(pin_size_t pin) : pin_{pin} {}

void FeetechBus::begin(uint32_t baudrate, size_t rxQueueSize) {
  rx_queue_.init(rxQueueSize);

  uint tx_offset;
  uint rx_offset;
  pio_claim_two_free_sms_and_add_programs(&feetech_bus_tx_program, &feetech_bus_rx_program, &pio_, &tx_sm_, &tx_offset,
                                          &rx_offset);
  rx_sm_ = tx_sm_ + 1;

  float div = static_cast<float>(clock_get_hz(clk_sys)) / (8 * baudrate);

  pio_sm_set_pins_with_mask64(pio_, tx_sm_, 0ull << pin_, 1ull << pin_);
  pio_sm_set_pindirs_with_mask64(pio_, tx_sm_, 0ull << pin_, 1ull << pin_);
  pio_gpio_init(pio_, pin_);

  pio_sm_config tx_config = feetech_bus_tx_program_get_default_config(tx_offset);
  sm_config_set_out_shift(&tx_config, true, false, 32);
  sm_config_set_out_pins(&tx_config, pin_, 1);
  sm_config_set_set_pins(&tx_config, pin_, 1);
  sm_config_set_clkdiv(&tx_config, div);
  pio_sm_init(pio_, tx_sm_, tx_offset, &tx_config);

  pio_sm_config rx_config = feetech_bus_rx_program_get_default_config(rx_offset);
  sm_config_set_in_shift(&rx_config, true, true, 8);
  sm_config_set_in_pins(&rx_config, pin_);
  sm_config_set_jmp_pin(&rx_config, pin_);
  sm_config_set_clkdiv(&rx_config, div);
  pio_sm_init(pio_, rx_sm_, rx_offset, &rx_config);

  feetech_bus_map[pio_get_index(pio_)][rx_sm_] = this;

  uint pio_irq = pio_get_irq_num(pio_, 0);
  pio_set_irq0_source_enabled(pio_, pio_get_rx_fifo_not_empty_interrupt_source(rx_sm_), true);
  irq_set_exclusive_handler(pio_irq, rxIrqHandler);
  irq_set_enabled(pio_irq, true);

  pio_sm_set_enabled(pio_, tx_sm_, true);
  pio_sm_set_enabled(pio_, rx_sm_, true);
}

size_t FeetechBus::available() { return rx_queue_.get_level(); }

uint8_t FeetechBus::read() { return rx_queue_.remove_blocking(); }

void FeetechBus::write(uint8_t data) { pio_sm_put_blocking(pio_, tx_sm_, data); }

void FeetechBus::rxIrqHandler() {
  for (int p = 0; p < NUM_PIOS; ++p) {
    for (int sm = 0; sm < NUM_PIO_STATE_MACHINES; ++sm) {
      FeetechBus *feetech_bus = feetech_bus_map[p][sm];
      if (feetech_bus) {
        while (!pio_sm_is_rx_fifo_empty(feetech_bus->pio_, feetech_bus->rx_sm_)) {
          uint8_t data = pio_sm_get(feetech_bus->pio_, feetech_bus->rx_sm_) >> 24;
          feetech_bus->rx_queue_.try_add(data);
        }
      }
    }
  }
}
