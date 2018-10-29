//
// Created by beck on 23/10/18.
//

#ifndef OLD_BOARD_CAN_COMM_H
#define OLD_BOARD_CAN_COMM_H


#define CAN_COMM_PORT &CAND2
#define CAN_END_EFFECTOR_OMEGA_SID  0x215

#define CAN_COMM_FREQ 100U

#define CAN_COMM_FREQ_US     (1000000U/CAN_COMM_FREQ)
#define CAN_COMM_FREQ_ST     (US2ST(CAN_COMM_FREQ_US))

void can_transmit(CANDriver *const CANx, const uint16_t SID,
             const int16_t val_0, const int16_t val_1,
             const int16_t val_2, const int16_t val_3);

void can_comm_init(void);

#endif //OLD_BOARD_CAN_COMM_H
