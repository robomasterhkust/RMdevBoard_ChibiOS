//
// Created by logan on 10/1/18.
//

#ifndef RM_CHIBIOS_COMPLEMENTARY_FILTER_H
#define RM_CHIBIOS_COMPLEMENTARY_FILTER_H




#define Filter_Frequency 1000U


#define Filter_PERIOD_US     (1000000U/Filter_Frequency)
#define Filter_PERIOD_ST     (US2ST(Filter_PERIOD_US))


typedef struct{
    float filter_output;
    uint32_t error;
} filter_send_struct;



void filter_init(void);
float get_filter_output(void);




#endif //RM_CHIBIOS_COMPLEMENTARY_FILTER_H
