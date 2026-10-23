#pragma once
#include <stdint.h>
#define ESP_MAC_BT 2
static inline int esp_read_mac(uint8_t* m, int) { for (int i = 0; i < 6; i++) m[i] = 0; return 0; }
