#include "tinyg.h"
#include "persistence.h"
#include "report.h"
#include "canonical_machine.h"
#include "util.h"


/************************************************************************************
 * read_persistent_value()	- return value (as float) by index
 * write_persistent_value() - write to NVM by index, but only if the value has changed
 *
 *	It's the responsibility of the caller to make sure the index does not exceed range
 */

stat_t read_persistent_value(nvObj_t *nv){
	nv->value = 0;
	return (STAT_OK);
}

stat_t write_persistent_value(nvObj_t *nv){
	if (cm.cycle_state != CYCLE_OFF)
        return(rpt_exception(STAT_COMMAND_NOT_ACCEPTED));	// can't write when machine is moving

	return (STAT_OK);
}
