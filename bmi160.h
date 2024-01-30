#ifndef __BMI160_H__
#define __BMI160_H__

#include <stdint.h>
#include <stdbool.h>

bool bmi160_init(void);
bool bmi160_set_sensivity(void);
void bmi160_enable_data_ready_interrupt(void);
void bmi160_update(void);
void bmi160_get_acc(int16_t *ax, int16_t *ay, int16_t *az);
void bmi160_get_gyr(int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* __BMI160_H__ */
