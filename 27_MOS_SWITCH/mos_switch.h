#ifndef MOS_SWITCH_H_
#define MOS_SWITCH_H_

#include <stdbool.h>

/*
 * Active-high MOS module driver.
 * The GPIO starts low, so the load remains off during initialization.
 */
void MOS_Switch_Init(void);
void MOS_Switch_On(void);
void MOS_Switch_Off(void);
void MOS_Switch_Set(bool enabled);
bool MOS_Switch_IsOn(void);

#endif /* MOS_SWITCH_H_ */
