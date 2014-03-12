#ifndef HARDHOME_H
#define HARDHOME_H

#define LMT_MA    PORTBbits.RB0
#define LMT_MB    PORTBbits.RB1
#define LMT_MC    PORTBbits.RB2
#define LMT_MD    PORTBbits.RB3
#define LMT_ME    PORTBbits.RB4
#define LMT_MF    PORTBbits.RB5

inline void lmtswitch_setup(void);
inline void hardhome(void);
void hardhome_motor_a(void);
void hardhome_motor_b(void);
void hardhome_motor_c(void);
void hardhome_motor_d(void);
void hardhome_motor_e(void);
void hardhome_motor_f(void);

#endif
