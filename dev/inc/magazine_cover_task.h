#ifndef _MAGAZINE_COVER_TASK_H_
#define _MAGAZINE_COVER_TASK_H_
#include "stdbool.h"
#define Maga_UPDATE_FREQ 50
#define Maga_UPDATE_PERIOD_US 1000000/Maga_UPDATE_FREQ
void pwm_magazine_cover_init(void);
void magCoverOpen(void);
void magCoverClose(void);
void magCoverToggle(void);

typedef struct {
	bool internalState;
	int* bitmap_for_magCover;
	bool R_press;
} magCoverStruct_t;

magCoverStruct_t* getMagCover(void);
bool getMagCoverInitStatus(void);

#endif
