
#ifndef __LINUX_I2C_SX865x_H
#define __LINUX_I2C_SX865x_H

/* linux/i2c/sx865x.h */

struct sx865x_platform_data {
        u16     model;                          /* 8650/1 */
        u16     y_plate_ohms;

		u16	x_max;
		u16	y_max;

        int     (*get_pendown_state)(void);
        void    (*clear_penirq)(void);          /* If needed, clear 2nd level
                                                   interrupt source */
        int     (*init_platform_hw)(void);
        void    (*exit_platform_hw)(void);
};
extern void __imapx_register_batt(int (*func)(void));
#endif




