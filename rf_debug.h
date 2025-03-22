#ifndef _rf_debug_h
#define _rf_debug_h

// Structure mapping nRF registers to name, size, and offset
typedef struct sRFRegInfo {
     char name[16];
     int size;
     int offset;
} tRFRegInfo;

#endif
