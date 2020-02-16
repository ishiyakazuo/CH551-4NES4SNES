#ifndef __FOURNESFOURSNES
#define __FOURNESFOURSNES

// Maximum number of supported gamepads is 4 (only 4 endpoints available)
#define NUM_GAMEPADS 4
// Maximum number of buttons an SNES pad can provide is 12.  Actual SNES controllers only will use the first 8.
// Setting this to 8 will reduce the number of bytes sent over USB, and save a byte of each HID report that gets copied into the endpoint.
#define NUM_BUTTONS 12

void fournsnesInit(void);
void fournsnesUpdate(void);
char fournsnesChanged(unsigned char report_id);
char fournsnesBuildReport(unsigned char *reportBuffer, unsigned char report_id);
void enableLiveAutodetect(void);
void disableLiveAutodetect(void);

#endif
