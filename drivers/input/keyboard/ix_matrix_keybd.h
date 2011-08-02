#ifndef __IX_MATRIX_KEYBD_H
#define __IX_MATRIX_KEYBD_H

#define MAX_MATRIX_KEY_ROWS	(18)
#define MAX_MATRIX_KEY_COLS	(8)
#define MAX_MATRIX_KEY_NUM	(MAX_MATRIX_KEY_ROWS * MAX_MATRIX_KEY_COLS)


struct imapx200_keybd_platform_data{
	unsigned int matrix_key_rows;
	unsigned int matrix_key_cols;

	unsigned int *matrix_key_map;

	int matrix_key_map_size;
};

#define KEY(row, col, val)	(((row) << 24) | ((col) << 20) | (val))

#if defined(CONFIG_INPUT_KEYBOARD_SPANISH)
static unsigned int imapx200_matrix_keys[] = {
	KEY(0, 0, KEY_KP1), KEY(0, 1, KEY_RIGHTALT),
	KEY(0, 2, KEY_KP2), KEY(0, 3, KEY_KP3),
	KEY(0, 4, KEY_KP4), KEY(0, 5, KEY_KP6),
	KEY(0, 6, KEY_I), KEY(0, 7, KEY_O),

	KEY(1, 0, KEY_F8), KEY(1, 1, KEY_L),
	KEY(1, 2, KEY_F7), KEY(1, 3, KEY_9),
	KEY(1, 4, KEY_8), KEY(1, 5, KEY_K),
	KEY(1, 6, KEY_I), KEY(1, 7, KEY_O),

	KEY(2, 0, KEY_KP9), KEY(2, 1, KEY_KP0),
	KEY(2, 2, KEY_Z), KEY(2, 3, KEY_Q),
	KEY(2, 4, KEY_TAB), KEY(2, 5, KEY_X),
	KEY(2, 6, KEY_S), KEY(2, 7, KEY_KPSLASH),

	KEY(3, 0, KEY_KPMINUS), KEY(3, 1, KEY_LEFTSHIFT),
	KEY(3, 2, KEY_X), KEY(3, 3, KEY_RIGHTSHIFT),
	KEY(3, 4, KEY_Q), KEY(3, 5, KEY_TAB),
	KEY(3, 6, KEY_6), KEY(3, 7, KEY_GRAVE),

	KEY(4, 0, KEY_F6), KEY(4, 1, KEY_F),
	KEY(4, 2, KEY_F5), KEY(4, 3, KEY_U),
	KEY(4, 4, KEY_7), KEY(4, 5, KEY_B),
	KEY(4, 6, KEY_J), KEY(4, 7, KEY_N),

	KEY(5, 0, KEY_F10), KEY(5, 1, KEY_COMMA),
	KEY(5, 2, KEY_F9), KEY(5, 3, KEY_APOSTROPHE),
	KEY(5, 4, KEY_0), KEY(5, 5, KEY_M),
	KEY(5, 6, KEY_P), KEY(5, 7, KEY_F16),

	KEY(6, 0, KEY_F13), KEY(6, 1, KEY_H),
	KEY(6, 2, KEY_BACK), KEY(6, 3, KEY_2),
	KEY(6, 4, KEY_1), KEY(6, 5, KEY_C),
	KEY(6, 6, KEY_W), KEY(6, 7, KEY_D),
#ifndef CONFIG_KEYBOARD_FOR_TEST
	KEY(7, 0, KEY_KPDOT), KEY(7, 1, KEY_MENU),
#else
	KEY(7, 0, KEY_KPDOT), KEY(7, 1, KEY_COMPOSE),
#endif
	KEY(7, 2, KEY_PAUSE), KEY(7, 3, KEY_UP),
	KEY(7, 4, KEY_8), KEY(7, 5, KEY_SPACE),
	KEY(7, 6, KEY_MINUS), KEY(7, 7, KEY_LEFT),

	KEY(8, 0, KEY_F18), KEY(8, 1, KEY_L),
	KEY(8, 2, KEY_P), KEY(8, 3, KEY_O),
	KEY(8, 4, KEY_0), KEY(8, 5, KEY_F19),
	KEY(8, 6, KEY_F9), KEY(8, 7, KEY_F8),

	KEY(9, 0, KEY_F4), KEY(9, 1, KEY_V),
	KEY(9, 2, KEY_F3), KEY(9, 3, KEY_6),
	KEY(9, 4, KEY_5), KEY(9, 5, KEY_H),
	KEY(9, 6, KEY_T), KEY(9, 7, KEY_Y),

	KEY(10, 0, KEY_B), KEY(10, 1, KEY_LEFTALT),
	KEY(10, 2, KEY_APOSTROPHE), KEY(10, 3, KEY_END),
	KEY(10, 4, KEY_VOLUMEUP), KEY(10, 5, KEY_EQUAL),
	KEY(10, 6, KEY_NUMLOCK), KEY(10, 7, KEY_F12),

	KEY(11, 0, KEY_VOLUMEDOWN), KEY(11, 1, KEY_HOME),
	KEY(11, 2, KEY_N), KEY(11, 3, KEY_DOT),
	KEY(11, 4, KEY_F20), KEY(11, 5, 0),
	KEY(11, 6, KEY_PAUSE), KEY(11, 7, KEY_INSERT),

	KEY(12, 0, KEY_SPACE), KEY(12, 1, KEY_LEFTCTRL),
	KEY(12, 2, 0), KEY(12, 3, KEY_SYSRQ),
	KEY(12, 4, KEY_RIGHTBRACE), KEY(12, 5, KEY_KP7),
	KEY(12, 6, KEY_LEFT), KEY(12, 7, KEY_PRINT),

	KEY(13, 0, KEY_LEFTSHIFT), KEY(13, 1, 0),
	KEY(13, 2, 0), KEY(13, 3, 0),
	KEY(13, 4, 0), KEY(13, 5, 0),
	KEY(13, 6, 0), KEY(13, 7, KEY_RIGHTSHIFT),

	KEY(14, 0, KEY_F12), KEY(14, 1, KEY_DOT),
	KEY(14, 2, KEY_F11), KEY(14, 3, KEY_GRAVE),
	KEY(14, 4, KEY_F14), KEY(14, 5, KEY_FN),
	KEY(14, 6, KEY_KPPLUS), KEY(14, 7, KEY_F17),

	KEY(15, 0, KEY_F2), KEY(15, 1, KEY_F),
	KEY(15, 2, KEY_F1), KEY(15, 3, KEY_4),
	KEY(15, 4, KEY_3), KEY(15, 5, KEY_G),
	KEY(15, 6, KEY_E), KEY(15, 7, KEY_R),

	KEY(16, 0, KEY_KP8), KEY(16, 1, KEY_KP9),
	KEY(16, 2, KEY_KPASTERISK), KEY(16, 3, KEY_CAPSLOCK),
	KEY(16, 4, KEY_KPPLUS), KEY(16, 5, KEY_Z),
	KEY(16, 6, KEY_A), KEY(16, 7, 0),

	KEY(17, 0, KEY_DELETE), KEY(17, 1, KEY_RIGHT),
	KEY(17, 2, KEY_SYSRQ), KEY(17, 3, KEY_F15),
	KEY(17, 4, KEY_BACKSPACE), KEY(17, 5, KEY_KP5),
	KEY(17, 6, KEY_ENTER), KEY(17, 7, KEY_DOWN),
};
#elif defined(CONFIG_INPUT_KEYBOARD_AMERICA)
static unsigned int imapx200_matrix_keys[] = {
        KEY(0, 0, KEY_END), KEY(0, 1, KEY_RIGHTALT),
        KEY(0, 2, KEY_SYSRQ), KEY(0, 3, 0),
        KEY(0, 4, KEY_F18), KEY(0, 5, 0),
        KEY(0, 6, 0), KEY(0, 7, 0),

        KEY(1, 0, KEY_F8), KEY(1, 1, KEY_L),
        KEY(1, 2, KEY_F7), KEY(1, 3, KEY_9),
        KEY(1, 4, KEY_8), KEY(1, 5, KEY_K),
        KEY(1, 6, KEY_I), KEY(1, 7, KEY_O),

        KEY(2, 0, 0), KEY(2, 1, 0),
        KEY(2, 2, 0), KEY(2, 3, KEY_Q),
        KEY(2, 4, KEY_TAB), KEY(2, 5, KEY_X),
        KEY(2, 6, KEY_S), KEY(2, 7, 0),

        KEY(3, 0, 0), KEY(3, 1, KEY_LEFTSHIFT),
        KEY(3, 2, 0), KEY(3, 3, KEY_RIGHTSHIFT),
        KEY(3, 4, 0), KEY(3, 5, 0),
        KEY(3, 6, 0), KEY(3, 7, 0),
 
        KEY(4, 0, KEY_F6), KEY(4, 1, 0),
        KEY(4, 2, KEY_F5), KEY(4, 3, KEY_U),
        KEY(4, 4, KEY_7), KEY(4, 5, KEY_B),
        KEY(4, 6, KEY_J), KEY(4, 7, KEY_N),
 
        KEY(5, 0, KEY_F10), KEY(5, 1, KEY_COMMA),
        KEY(5, 2, KEY_F9), KEY(5, 3, KEY_MINUS),
        KEY(5, 4, KEY_0), KEY(5, 5, KEY_M),
	KEY(5, 6, KEY_P), KEY(5, 7, KEY_SEMICOLON),
 
        KEY(6, 0, KEY_GRAVE), KEY(6, 1, 0),
        KEY(6, 2, KEY_BACK), KEY(6, 3, KEY_2),
        KEY(6, 4, KEY_1), KEY(6, 5, KEY_C),
        KEY(6, 6, KEY_W), KEY(6, 7, KEY_D),

        KEY(7, 0, 0), KEY(7, 1, KEY_MENU),
        KEY(7, 2, KEY_PAUSE), KEY(7, 3, KEY_UP),
        KEY(7, 4, 0), KEY(7, 5, KEY_SPACE),
        KEY(7, 6, KEY_SLASH), KEY(7, 7, KEY_LEFT),

        KEY(8, 0, KEY_KP7), KEY(8, 1, KEY_KP8),
        KEY(8, 2, KEY_KP9), KEY(8, 3, KEY_KPMINUS),
        KEY(8, 4, KEY_KP4), KEY(8, 5, KEY_KP5),
        KEY(8, 6, KEY_KP6), KEY(8, 7, KEY_KPPLUS),

        KEY(9, 0, KEY_F4), KEY(9, 1, KEY_V),
        KEY(9, 2, KEY_F3), KEY(9, 3, KEY_6),
        KEY(9, 4, KEY_5), KEY(9, 5, KEY_H),
        KEY(9, 6, KEY_T), KEY(9, 7, KEY_Y),

        KEY(10, 0,0), KEY(10, 1,KEY_LEFTALT),
        KEY(10, 2,0), KEY(10, 3,0),
        KEY(10, 4,0), KEY(10, 5,0),
        KEY(10, 6,0), KEY(10, 7,0),
 
        KEY(11, 0,0), KEY(11, 1,KEY_HOME),
        KEY(11, 2,0), KEY(11, 3,0),
        KEY(11, 4,0), KEY(11, 5,0),
        KEY(11, 6,0), KEY(11, 7,0),
 
        KEY(12, 0,KEY_POWER), KEY(12, 1,KEY_LEFTCTRL),
        KEY(12, 2,KEY_VOLUMEDOWN), KEY(12, 3,KEY_VOLUMEUP),
        KEY(12, 4,0), KEY(12, 5,0),
	KEY(12, 6,0), KEY(12, 7,0),
	
        KEY(13, 0,KEY_KP1), KEY(13, 1,KEY_KP2),
        KEY(13, 2,KEY_KP3), KEY(13, 3,KEY_KP0),
        KEY(13, 4,KEY_KPDOT), KEY(13, 5,KEY_KPSLASH),
        KEY(13, 6,KEY_KPASTERISK), KEY(13, 7,KEY_NUMLOCK),

        KEY(14, 0,KEY_F12), KEY(14, 1,KEY_DOT),
        KEY(14, 2,KEY_F11), KEY(14, 3,KEY_LEFTBRACE),
        KEY(14, 4,KEY_EQUAL), KEY(14, 5,KEY_FN),
        KEY(14, 6,KEY_RIGHTBRACE), KEY(14, 7,KEY_APOSTROPHE),

        KEY(15, 0,KEY_F2), KEY(15, 1,KEY_F),
        KEY(15, 2,KEY_F1), KEY(15, 3, KEY_4),
        KEY(15, 4, KEY_3), KEY(15, 5,KEY_G),
        KEY(15, 6,KEY_E), KEY(15, 7,KEY_R),

        KEY(16, 0,0), KEY(16, 1,0),
        KEY(16, 2,0), KEY(16, 3,KEY_CAPSLOCK),
        KEY(16, 4,0), KEY(16, 5,KEY_Z),
        KEY(16, 6,KEY_A), KEY(16, 7,0),
 
        KEY(17, 0,KEY_BACKSPACE), KEY(17, 1,KEY_RIGHT),
        KEY(17, 2,KEY_SYSRQ), KEY(17, 3,KEY_BACKSLASH),
        KEY(17, 4,KEY_BACKSPACE), KEY(17, 5,0),
        KEY(17, 6,KEY_ENTER), KEY(17, 7,KEY_DOWN),
 }; 
#else
static unsigned int imapx200_matrix_keys[] = {
        KEY(0, 0, KEY_END), KEY(0, 1, KEY_RIGHTALT),
        KEY(0, 2, KEY_SYSRQ), KEY(0, 3, 0),
        KEY(0, 4, KEY_F18), KEY(0, 5, 0),
        KEY(0, 6, 0), KEY(0, 7, 0),

        KEY(1, 0, KEY_F8), KEY(1, 1, KEY_L),
        KEY(1, 2, KEY_F7), KEY(1, 3, KEY_9),
        KEY(1, 4, KEY_8), KEY(1, 5, KEY_K),
        KEY(1, 6, KEY_I), KEY(1, 7, KEY_O),

        KEY(2, 0, 0), KEY(2, 1, 0),
        KEY(2, 2, 0), KEY(2, 3, KEY_Q),
        KEY(2, 4, KEY_TAB), KEY(2, 5, KEY_X),
        KEY(2, 6, KEY_S), KEY(2, 7, 0),

        KEY(3, 0, 0), KEY(3, 1, KEY_LEFTSHIFT),
        KEY(3, 2, 0), KEY(3, 3, KEY_RIGHTSHIFT),
        KEY(3, 4, 0), KEY(3, 5, 0),
        KEY(3, 6, 0), KEY(3, 7, 0),
 
        KEY(4, 0, KEY_F6), KEY(4, 1, 0),
        KEY(4, 2, KEY_F5), KEY(4, 3, KEY_U),
        KEY(4, 4, KEY_7), KEY(4, 5, KEY_B),
        KEY(4, 6, KEY_J), KEY(4, 7, KEY_N),
 
        KEY(5, 0, KEY_F10), KEY(5, 1, KEY_COMMA),
        KEY(5, 2, KEY_F9), KEY(5, 3, KEY_MINUS),
        KEY(5, 4, KEY_0), KEY(5, 5, KEY_M),
	KEY(5, 6, KEY_P), KEY(5, 7, KEY_SEMICOLON),
 
        KEY(6, 0, KEY_GRAVE), KEY(6, 1, 0),
        KEY(6, 2, KEY_BACK), KEY(6, 3, KEY_2),
        KEY(6, 4, KEY_1), KEY(6, 5, KEY_C),
        KEY(6, 6, KEY_W), KEY(6, 7, KEY_D),

        KEY(7, 0, 0), KEY(7, 1, KEY_MENU),
        KEY(7, 2, KEY_PAUSE), KEY(7, 3, KEY_UP),
        KEY(7, 4, 0), KEY(7, 5, KEY_SPACE),
        KEY(7, 6, KEY_SLASH), KEY(7, 7, KEY_LEFT),

        KEY(8, 0, KEY_KP7), KEY(8, 1, KEY_KP8),
        KEY(8, 2, KEY_KP9), KEY(8, 3, KEY_KPMINUS),
        KEY(8, 4, KEY_KP4), KEY(8, 5, KEY_KP5),
        KEY(8, 6, KEY_KP6), KEY(8, 7, KEY_KPPLUS),

        KEY(9, 0, KEY_F4), KEY(9, 1, KEY_V),
        KEY(9, 2, KEY_F3), KEY(9, 3, KEY_6),
        KEY(9, 4, KEY_5), KEY(9, 5, KEY_H),
        KEY(9, 6, KEY_T), KEY(9, 7, KEY_Y),

        KEY(10, 0,0), KEY(10, 1,KEY_LEFTALT),
        KEY(10, 2,0), KEY(10, 3,0),
        KEY(10, 4,0), KEY(10, 5,0),
        KEY(10, 6,0), KEY(10, 7,0),
 
        KEY(11, 0,0), KEY(11, 1,KEY_HOME),
        KEY(11, 2,0), KEY(11, 3,0),
        KEY(11, 4,0), KEY(11, 5,0),
        KEY(11, 6,0), KEY(11, 7,0),
 
        KEY(12, 0,KEY_POWER), KEY(12, 1,KEY_LEFTCTRL),
        KEY(12, 2,KEY_VOLUMEDOWN), KEY(12, 3,KEY_VOLUMEUP),
        KEY(12, 4,0), KEY(12, 5,0),
	KEY(12, 6,0), KEY(12, 7,0),
	
        KEY(13, 0,KEY_KP1), KEY(13, 1,KEY_KP2),
        KEY(13, 2,KEY_KP3), KEY(13, 3,KEY_KP0),
        KEY(13, 4,KEY_KPDOT), KEY(13, 5,KEY_KPSLASH),
        KEY(13, 6,KEY_KPASTERISK), KEY(13, 7,KEY_NUMLOCK),

        KEY(14, 0,KEY_F12), KEY(14, 1,KEY_DOT),
        KEY(14, 2,KEY_F11), KEY(14, 3,KEY_LEFTBRACE),
        KEY(14, 4,KEY_EQUAL), KEY(14, 5,KEY_FN),
        KEY(14, 6,KEY_RIGHTBRACE), KEY(14, 7,KEY_APOSTROPHE),

        KEY(15, 0,KEY_F2), KEY(15, 1,KEY_F),
        KEY(15, 2,KEY_F1), KEY(15, 3, KEY_4),
        KEY(15, 4, KEY_3), KEY(15, 5,KEY_G),
        KEY(15, 6,KEY_E), KEY(15, 7,KEY_R),

        KEY(16, 0,0), KEY(16, 1,0),
        KEY(16, 2,0), KEY(16, 3,KEY_CAPSLOCK),
        KEY(16, 4,0), KEY(16, 5,KEY_Z),
        KEY(16, 6,KEY_A), KEY(16, 7,0),
 
        KEY(17, 0,KEY_BACKSPACE), KEY(17, 1,KEY_RIGHT),
        KEY(17, 2,KEY_SYSRQ), KEY(17, 3,KEY_BACKSLASH),
        KEY(17, 4,KEY_BACKSPACE), KEY(17, 5,0),
        KEY(17, 6,KEY_ENTER), KEY(17, 7,KEY_DOWN),
 }; 
#endif	

struct imapx200_keybd_platform_data imapx200_keybd_info = {
	/* code map for the matrix keys */
	.matrix_key_rows	= 18,
	.matrix_key_cols	= 8,
	.matrix_key_map		= imapx200_matrix_keys,
	.matrix_key_map_size	= ARRAY_SIZE(imapx200_matrix_keys),
};
 
#endif 
