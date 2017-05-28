#ifndef __USE_EXAMPLE_H__
#define __USE_EXAMPLE_H__

#include "../LoRa/lora.h"

extern int lora_device_add(struct lora_struct *);
extern int lora_device_remove(struct lora_struct *);
extern int lora_register_driver(struct lora_driver *);
extern int lora_unregister_driver(struct lora_driver *);

#endif
