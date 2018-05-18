#ifndef ROBOCONF_H_
#define ROBOCONF_H_

//#define RM_INFANTRY
//#define RM_HERO

#ifdef RM_INFANTRY
  #define CHASSIS_POWER_MAX_W        80U
  #define CHASSIS_POWER_BUFFER_J     60U
#endif //RM_INFANTRY

#ifdef RM_HERO
  #define CHASSIS_POWER_MAX_W       120U
  #define CHASSIS_POWER_BUFFER_J     60U
#endif //RM_HERO

#endif //ROBOCONF_H_
