#include "power_module.h"
#include "keyboard.h"
#include "canBusProcess.h"
#include "dbus.h"


static powerModuleStruct_t powerModule;
static volatile Gimbal_Send_Dbus_canStruct* pRC;

#define POWER_MODULE_UPDATE_PERIOD_US 1000000/1000

static void powerMode_txCan(CANDriver *const CANx, const uint16_t SID)
{
  CANTxFrame txmsg;
  powerModule_canTransmitStruct txCan;

  txmsg.IDE = CAN_IDE_STD;
  txmsg.SID = SID;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x02;

  chSysLock();
  txCan.power_mode = powerModule.power_mode;
  txCan.robotType = powerModule.robotType;

  memcpy(&(txmsg.data8), &txCan ,sizeof(powerModule_canTransmitStruct));
  chSysUnlock();

  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}


// void can_send_powerMode(CANDriver *const CANx,
//   const uint16_t EID, uint8_t powerMode, uint8_t robotType)
// {
//     CANTxFrame txmsg;

//     txmsg.IDE = CAN_IDE_STD;
//     txmsg.EID = EID;
//     txmsg.RTR = CAN_RTR_DATA;
//     txmsg.DLC = 0x08;

//     chSysLock();
//     txmsg.data8[0] = (uint8_t)powerMode;
//     txmsg.data8[1] = (uint8_t)robotType;
//     chSysUnlock();

//     canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
// }



static THD_WORKING_AREA(power_module_wa, 1024);
static THD_FUNCTION(power_module, p)
{

  (void)p;
  chRegSetThreadName("power_module");
  uint32_t tick = chVTGetSystemTimeX();
  pRC = can_get_sent_dbus();
  while(!chThdShouldTerminateX())
  {
    tick += US2ST(POWER_MODULE_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }
    if((pRC->s1 == MI) & (pRC->s2 == DN)){
	    if(powerModule.bitmap_for_powerModule[KEY_X]){
	      if(!powerModule.X_press){
	        if(powerModule.power_mode == PJUDGE){
	        	powerModule.power_mode = BOOST;
	        }else if(powerModule.power_mode == BOOST){
	        	powerModule.power_mode = PJUDGE;
	        }
	      }
	      powerModule.X_press = true;

	    }
    	else{
      	powerModule.X_press = false;
    	}
	}else if((pRC->s1 == MI)){
		if(pRC->s2 == UP){ // Power from Judge 
			powerModule.power_mode = BOOST;
		}else if(pRC->s2 == MI){ // Power from PowerModule
			powerModule.power_mode = PJUDGE;
		}
	}
	powerMode_txCan(POWER_MODULE_CAN,CAN_POWER_MODULE_SEND_MODE_ID);
	// can_send_powerMode(POWER_MODULE_CAN, CAN_POWER_MODULE_SEND_MODE_ID, powerModule.power_mode, powerModule.robotType);
  }
}

void resetPowerModuleInfo(void){
	powerModule.power_mode = PJUDGE;
	powerModule.robotType = INFANTRY_T;
	powerModule.bitmap_for_powerModule = Bitmap_get();
	powerModule.X_press = 0;
}

void power_module_init(void){
	resetPowerModuleInfo();
    chThdCreateStatic(power_module_wa, sizeof(power_module_wa),
                               NORMALPRIO, power_module, NULL);
}
