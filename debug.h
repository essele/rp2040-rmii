

#if RMII_DEBUG
#define debug_printf(...)		printf(__VA_ARGS__)

#else
#define debug_printf(...)
#endif
