

#ifndef __EP0700M01_H__
#define __EP0700M01_H__



struct ep07_setup_data {
         int i2c_bus;
         unsigned short i2c_address;
};



#define EP07_X1_H	(0x0)
#define EP07_X1_L	(0x1)
#define EP07_Y1_H	(0x2)
#define EP07_Y1_L	(0x3)
#define EP07_X2_H	(0x4)
#define EP07_X2_L	(0x5)
#define EP07_Y2_H	(0x6)
#define EP07_Y2_L	(0x7)

#define EP07_FINGER_NUM	(0x9)

#define EP07_VERSION	(0xA)

#define EP07_IDLE	(0xB)
#define EP07_TIMEOUT	(0xC)












#endif

