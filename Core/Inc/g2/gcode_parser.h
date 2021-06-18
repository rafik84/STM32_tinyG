#ifndef GCODE_PARSER_H_ONCE
#define GCODE_PARSER_H_ONCE

/*
 * Global Scope Functions
 */
stat_t gc_gcode_parser(char_t *block);
stat_t gc_get_gc(nvObj_t *nv);
stat_t gc_run_gc(nvObj_t *nv);

#endif
