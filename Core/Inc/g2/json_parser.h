#ifndef _JSON_PARSER_H_ONCE
#define _JSON_PARSER_H_ONCE

#ifdef __cplusplus
extern "C"{
#endif

/**** Configs, Definitions and Structures ****/

/* JSON array definitions / revisions */
// for now there is only one JSON array in use - the footer
// if you add these make sure there are no collisions w/present or past numbers

#define FOOTER_REVISION 1

#define JSON_OUTPUT_STRING_MAX (OUTPUT_BUFFER_LEN)

enum jsonVerbosity {
	JV_SILENT = 0,					// no response is provided for any command
	JV_FOOTER,						// returns footer only (no command echo, gcode blocks or messages)
	JV_MESSAGES,					// returns footer, messages (exception and gcode messages)
	JV_CONFIGS,						// returns footer, messages, config commands
	JV_LINENUM,						// returns footer, messages, config commands, gcode line numbers if present
	JV_VERBOSE						// returns footer, messages, config commands, gcode blocks
};

enum jsonFormats {					// json output print modes
	JSON_NO_PRINT = 0,				// don't print anything if you find yourself in JSON mode
	JSON_OBJECT_FORMAT,				// print just the body as a json object
	JSON_RESPONSE_FORMAT			// print the header/body/footer as a response object
};

enum jsonSyntaxMode {
	JSON_SYNTAX_RELAXED = 0,		// Does not require quotes on names
	JSON_SYNTAX_STRICT				// requires quotes on names
};

typedef struct jsSingleton {

	/*** config values (PUBLIC) ***/
	uint8_t json_verbosity;			// see enum in this file for settings
	uint8_t json_footer_depth;		// 0=footer is peer to response 'r', 1=child of response 'r'
//	uint8_t json_footer_style;		// select footer style
	uint8_t json_syntax;			// 0=relaxed syntax, 1=strict syntax

	uint8_t echo_json_footer;		// flags for JSON responses serialization
	uint8_t echo_json_messages;
	uint8_t echo_json_configs;
	uint8_t echo_json_linenum;
	uint8_t echo_json_gcode_block;

	/*** runtime values (PRIVATE) ***/

} jsSingleton_t;

/**** Externs - See report.c for allocation ****/

extern jsSingleton_t js;

/**** Function Prototypes ****/

void json_parser(char_t *str);
uint16_t json_serialize(nvObj_t *nv, char_t *out_buf, uint16_t size);
void json_print_object(nvObj_t *nv);
void json_print_response(uint8_t status);
void json_print_list(stat_t status, uint8_t flags);

stat_t json_set_jv(nvObj_t *nv);

#ifdef __TEXT_MODE

	void js_print_ej(nvObj_t *nv);
	void js_print_jv(nvObj_t *nv);
	void js_print_js(nvObj_t *nv);
	void js_print_fs(nvObj_t *nv);

#else

	#define js_print_ej tx_print_stub
	#define js_print_jv tx_print_stub
	#define js_print_js tx_print_stub
	#define js_print_fs tx_print_stub

#endif // __TEXT_MODE

#ifdef __cplusplus
}
#endif

#endif // End of include guard: JSON_PARSER_H_ONCE
