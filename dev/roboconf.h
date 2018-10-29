#ifndef ROBOCONF_H_
#define ROBOCONF_H_

#define RM_INFANTRY
//#define RM_HERO
//#define RM_DEBUG

#ifdef RM_INFANTRY
  #define CHASSIS_POWER_MAX_W        80U
  #define CHASSIS_POWER_BUFFER_J     60U

  #define LEVEL1_HEATLIMIT 130
  #define LEVEL2_HEATLIMIT 250
  #define LEVEL3_HEATLIMIT 490
#endif //RM_INFANTRY

#ifdef RM_HERO
  #define CHASSIS_POWER_MAX_W       120U
  #define CHASSIS_POWER_BUFFER_J     60U
#endif //RM_HERO

#ifdef RM_DEBUG
  #define CHASSIS_POWER_MAX_W        80U
  #define CHASSIS_POWER_BUFFER_J     60U
  #define LEVEL1_HEATLIMIT    90U
  #define LEVEL2_HEATLIMIT    180U
  #define LEVEL3_HEATLIMIT    360U
#endif //RM_DEBUG

#endif //ROBOCONF_H_
