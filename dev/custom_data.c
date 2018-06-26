#include "custom_data.h"
#include "judge.h"
#include "bullet_tracker_task.h"

static Custom_Data_t customData;
static bool allInited = false;

bool checkInit(void){
	if(allInited){
		return true;
	}
	Bullet_Tracker_t* pBT = bulletTracker_get();
	bool bulletCountInit = pBT->inited;
	bool data2Init = false; // edit
	bool data3Init = false; // edit
	bool lights8Init = false; // edit

	if(bulletCountInit & data2Init & data3Init & lights8Init){
		allInited = true;
		return true;
	}else{
		return false;
	}
}

#define  CUSTOM_DATA_UPDATE_PERIOD      100U // the update frequency is 100ms
static THD_WORKING_AREA(custom_data_thread_wa, 512);

static THD_FUNCTION(custom_data_thread, p)
{
    Custom_Data_t *d = (Custom_Data_t *) p;
    chRegSetThreadName("Update Custom Data");
    // msg_t rxmsg;
    // systime_t timeout = MS2ST(CUSTOM_DATA_UPDATE_PERIOD);

    while (!chThdShouldTerminateX()) {
    	if(checkInit()){
    		Bullet_Tracker_t* pBT = bulletTracker_get();
	    	d->data1 = (float)(pBT->bullet_tracker.bulletCount);
	    	d->data2 = 0.0f; // edit
	    	d->data3 = 0.0f; // edit
	    	d->lights8 = 0;  // edit
    	}else{
    		d->data1 = 0.12345;
    		d->data2 = 0.12345;
    		d->data3 = 0.12345;
    		d->lights8 = 0b11111111;
    	}
    	judgeDataWrite(d->data1, d->data2, d->data3, d->lights8); 
    	chThdSleepMilliseconds(CUSTOM_DATA_UPDATE_PERIOD);
    }
}

void customData_init(void){
	customData.data1 = 0.0f;
	customData.data2 = 0.0f;
	customData.data3 = 0.0f;
	customData.lights8 = 0;

    chThdCreateStatic(custom_data_thread_wa, sizeof(custom_data_thread_wa),
                  NORMALPRIO + 7,
                  custom_data_thread, &customData);

}