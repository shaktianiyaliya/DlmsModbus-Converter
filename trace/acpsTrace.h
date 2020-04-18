#ifndef __ACPSTRACE_H__
#define __ACPSTRACE_H__

/*****************************************************************************/

#define ACPS_DEBUG_TRACE

/************************* System trace messages *****************************/

#ifdef ACPS_DEBUG_TRACE
/* size of trace buffer */
	#define TRACE_BUFFSIZE (128)
	extern char trace_buffer[TRACE_BUFFSIZE];
#endif

/*****************************************************************************/

void acpsTrace_init(void);
void acps_trace(char *msg);
uint32_t acps_trace_input(char *read_input, uint32_t length, uint16_t time_out);
void diagnostic_trace(char *msg);
void error_trace(char *msg);


#endif
