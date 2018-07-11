#include "custom_data.h"
#include "judge.h"
#include "bullet_tracker_task.h"
#include "magazine_cover_task.h"
#include <stdbool.h>

static Custom_Data_t customData;
static bool allInited = false;
static size_t sizeout;
static Bullet_Tracker_t* pBT;
// static projectile_fb_t projectile;
static judge_fb_t* pJudge;
static magCoverStruct_t* pMC;


bool checkInit(void){
	bool bulletCountInit = pBT->inited;
	bool judgeDataInit = getJudgeInitStatus(); 
	bool magCoverInit = getMagCoverInitStatus(); 

	if(bulletCountInit & judgeDataInit & magCoverInit){
		allInited = true;
		return true;
	}else{
		return false;
	}
}

#define  CUSTOM_DATA_UPDATE_PERIOD      100U // the update frequency is 100ms
static THD_WORKING_AREA(custom_data_thread_wa, 1024);

static THD_FUNCTION(custom_data_thread, p)
{
    Custom_Data_t *d = (Custom_Data_t *) p;
    chRegSetThreadName("Update Custom Data");
    // msg_t rxmsg;
    // systime_t timeout = MS2ST(CUSTOM_DATA_UPDATE_PERIOD);

    while (!chThdShouldTerminateX()) {
    	if(allInited){
	    	d->data1 = (float)(pBT->bullet_tracker.bulletCount);
	    	d->data2 = (float)(pJudge->projectileInfo.bulletSpeed); // edit
	    	d->data3 = (float)(-1.0); // edit
	    	d->lights8 = 0b00000000 | (uint8_t)(pMC->internalState);  // edit
    	}else{
    		checkInit();
    		d->data1 = -1.0f;
    		d->data2 = -1.0f;
    		d->data3 = -1.0f;
    		d->lights8 = 0b00000000;
    	}
    	sizeout = judgeDataWrite(d->data1, d->data2, d->data3, d->lights8); 
    	chThdSleepMilliseconds(CUSTOM_DATA_UPDATE_PERIOD);
    }
}

void customData_init(void){

    pBT = bulletTracker_get();
    pJudge = judgeDataGet();
    pMC = getMagCover();

	customData.data1 = 0.0f;
	customData.data2 = 0.0f;
	customData.data3 = 0.0f;
	customData.lights8 = 0;

    chThdCreateStatic(custom_data_thread_wa, sizeof(custom_data_thread_wa),
                  NORMALPRIO + 7,
                  custom_data_thread, &customData);

}