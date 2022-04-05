

#if RMII_DEBUG
void debug_printf(char *format, ...);
//#define debug_printf(...)		printf(__VA_ARGS__)

/**
 * @brief Print useful packet information for received packets (during debugging)
 * 
 * This will output size, fcs and then the first 16 and last 8 bytes in any
 * recevied packet.
 * 
 * @param cmnt 
 * @param p 
 * @param length 
 * @param fcs 
 */
static inline void dump_pkt_info(char *cmnt, uint8_t *p, int length, uint32_t fcs) {
    debug_printf("%s: len=%-4.4d fcs=%08x: ", cmnt, length, fcs);
    for(int i=0; i < 16; i++) { debug_printf("%02x ", p[i]); }
    debug_printf("... ");
    for(int i=length-8; i < length; i++) { debug_printf("%02x ", p[i]); }
    debug_printf("\r\n");
}

#else
#define debug_printf(...)
#endif
