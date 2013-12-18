#ifndef MOTORCTL_H
#define MOTORCTL_H

inline void motorctl_setup(void);
void motorctl_enable_pid(unsigned char motors);
void motorctl_disable_pid(unsigned char motors);

#endif
