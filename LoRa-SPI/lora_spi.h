#ifndef __LORA_SPI_H__
#define __LORA_SPI_H__

#include "lora.h"

extern int lora_device_add(struct lora_struct *);
extern int lora_device_remove(struct lora_struct *);
extern int lora_register_driver(struct lora_driver *);
extern int lora_unregister_driver(struct lora_driver *);

#endif
