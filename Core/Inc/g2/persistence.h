#ifndef PERSISTENCE_H_ONCE
#define PERSISTENCE_H_ONCE

#include "config.h"						// needed for nvObj_t definition

void persistence_init(void);
stat_t read_persistent_value(nvObj_t *nv);
stat_t write_persistent_value(nvObj_t *nv);

#endif // End of include guard: PERSISTENCE_H_ONCE
