

#if RMII_DEBUG
#define debug_printf(...)		printf(__VA_ARGS__)

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
    printf("%s: len=%-4.4d fcs=%08x: ", cmnt, length, fcs);
    for(int i=0; i < 16; i++) { printf("%02x ", p[i]); }
    printf("... ");
    for(int i=length-8; i < length; i++) { printf("%02x ", p[i]); }
    printf("\r\n");
}

#else
#define debug_printf(...)
#endif
