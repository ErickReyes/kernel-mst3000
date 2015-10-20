


#define ENABLE_INT		0	// 0->Polling, 1->Interupt, 2->Hybrid
#define EdgeDisable		1	// if Edge Disable, set it to 1, else reset to 0
#define RunningAverageMode	2	//{0,8},{5,3},{6,2},{7,1}
#define RunningAverageDist	4	// Threshold Between two consecutive points
#define MicroTimeTInterupt	12000000// 83.33Hz - 12,000,000us
#define FINGERNO		10

#define SCREEN_MAX_X    1664
#define SCREEN_MAX_Y    1025


struct ChipSetting ssd253xcfgTable[]={
		{2, 0x06, 0x19, 0x0F},	// 24 Driving lines, 16 sensing lines

		{2, 0x07, 0x00, 0xE0},  // Pin select for drive line #1
		{2, 0x08, 0x00, 0xE1},	// Pin select for drive line #2
		{2, 0x09, 0x00, 0xE2},	// Pin select for drive line #3
		{2, 0x0A, 0x00, 0xE3},	// Pin select for drive line #4
		{2, 0x0B, 0x00, 0xE4},	// Pin select for drive line #5
		{2, 0x0C, 0x00, 0xE5},	// Pin select for drive line #6
		{2, 0x0D, 0x00, 0xE6},	// Pin select for drive line #7
		{2, 0x0E, 0x00, 0xE7},	// Pin select for drive line #8
		{2, 0x0F, 0x00, 0xE8},	// Pin select for drive line #9
		{2, 0x10, 0x00, 0xE9},	// Pin select for drive line #10
		{2, 0x11, 0x00, 0xEA},	// Pin select for drive line #11
		{2, 0x12, 0x00, 0xEB},	// Pin select for drive line #12
		{2, 0x13, 0x00, 0xEC},	// Pin select for drive line #13
		{2, 0x14, 0x00, 0xED},	// Pin select for drive line #14
		{2, 0x15, 0x00, 0xEE},	// Pin select for drive line #15
		{2, 0x16, 0x00, 0xEF},	// Pin select for drive line #16
		{2, 0x17, 0x00, 0xF0},	// Pin select for drive line #17
		{2, 0x18, 0x00, 0xF1},	// Pin select for drive line #18
		{2, 0x19, 0x00, 0xF2},	// Pin select for drive line #19
		{2, 0x1A, 0x00, 0xF3},	// Pin select for drive line #20
		{2, 0x1B, 0x00, 0xF4},	// Pin select for drive line #21
		{2, 0x28, 0x00, 0x14},  // Sense offset = 20

		{2, 0x30, 0x08, 0x0F},  // Integration window timing 1uS - 1.875uS

		{2, 0xD7, 0x00, 0x03},	// ADC Vref range. VrefH = VCI/2+0.50 VrefL = VCI/2-0.50
		{2, 0xD8, 0x00, 0x06},	// Bias resistance = 23k
		{2, 0xDB, 0x00, 0x03},	// Integrator cap value (??)

		{2, 0x33, 0x00, 0x03},  // Min finger area = 3
		{2, 0x34, 0xC6, 0x60},  // Min finger level (Invalid value?)
		{2, 0x36, 0x00, 0x20},	// Max finger area = 32
		{2, 0x37, 0x07, 0xC4},	// Invalid register (R37h)?

		{2, 0x40, 0x10, 0xC8},	// Invalid register (R40h)?
		{2, 0x41, 0x00, 0x30},	// Invalid register (R41h)?
		{2, 0x42, 0x00, 0x50},	// Invalid register (R42h)?
		{2, 0x43, 0x00, 0x30},	// Invalid register (R43h)?
		{2, 0x44, 0x00, 0x50},	// Invalid register (R44h)?
		{2, 0x45, 0x00, 0x00},	// Invalid register (R45h)?
		{2, 0x46, 0x10, 0x1F},	// Invalid register (R45h)?

		{2, 0x56, 0x80, 0x10},	// Invalid register (R56h)?
		{2, 0x59, 0x80, 0x10},	// Invalid register (R59h)?

		{2, 0x65, 0x00, 0x05},	// Orientation register = Transpose + X Invert
		{2, 0x66, 0x1E, 0x00},	// X scaling = 7680
		{2, 0x67, 0x1E, 0xC4},	// Y scaling = 7876

		{2, 0x7A, 0xFF, 0xFF},	// Command removed from datasheet
		{2, 0x7B, 0x00, 0x03}, 	// IRQ mask = All disabled?


};

struct ChipSetting Reset[]={
		{ 1, 0x01, 0x00, 0x00},	// Software reset command
};

struct ChipSetting Resume[]={
		{ 1, 0x04, 0x00, 0x00},	// System enable command
		{ 2, 0x25, 0x00, 0x0C}, // Set Operation Mode (12 ms)
};

struct ChipSetting Suspend[] ={
		{ 2, 0x25, 0x00, 0x00}, // Set Operation Mode (idle)
		{ 1, 0x05, 0x00, 0x00},	// System disable command
};

