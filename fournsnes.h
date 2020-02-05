#ifndef __FOURNESFOURSNES
#define __FOURNESFOURSNES

#define NUM_BUTTONS 12

void fournsnesInit(void);
void fournsnesUpdate(void);
char fournsnesChanged(unsigned char report_id);
char fournsnesBuildReport(unsigned char *reportBuffer, unsigned char report_id);

#endif
