#include <stdint.h>
#include <stdbool.h>
#include <qemu/typedefs.h>
#include "stm32f2xx.h"
#include "stm32f4xx.h"
#include "stm32f7xx.h"

typedef enum {
    PBL_BUTTON_ID_NONE = -1,
    PBL_BUTTON_ID_BOOT = 0,
    PBL_NUM_BUTTONS = 1
} PblButtonID;

typedef struct {
    int gpio;
    int pin;
    bool active_high;
} PblButtonMap;

typedef struct {
  int dbgserial_uart_index;
  int pebble_control_uart_index;

  PblButtonMap button_map[PBL_NUM_BUTTONS];
  uint32_t gpio_idr_masks[STM32F4XX_GPIO_COUNT];

  // memory sizes
  uint32_t flash_size;
  uint32_t ram_size;
  // screen sizes
  uint32_t num_rows;
  uint32_t num_cols;
  uint32_t num_border_rows;
  uint32_t num_border_cols;
  bool row_major;
  bool row_inverted;
  bool col_inverted;
  bool round_mask;
} PblBoardConfig;


DeviceState *pebble_init_board(Stm32Gpio *gpio[], qemu_irq display_vibe);

