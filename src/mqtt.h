#ifndef _MQTT_H_
#define _MQTT_H_

#include <twr.h>

// helper functions
int hex2int(char c);

// topic callback headers
void config_set(uint64_t *id, const char *topic, void *value, void *param);
void config_get(uint64_t *id, const char *topic, void *value, void *param);

void publish_temperatures(void);

#endif // _MQTT_H_