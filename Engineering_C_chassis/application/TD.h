#ifndef _TD_H
#define _TD_H

typedef struct
{
	float xx1;
	float xx2;
	float fh1;
	float r;
	float h1;
	float h2;
} TD;

extern TD Y_td;
extern TD X_td;

void TD_INIT(void);
void td_task(void);

#endif