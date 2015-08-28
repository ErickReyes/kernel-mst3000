


#define ENABLE_INT		0	// 0->Polling, 1->Interupt, 2->Hybrid
#define EdgeDisable		1	// if Edge Disable, set it to 1, else reset to 0
#define RunningAverageMode	2	//{0,8},{5,3},{6,2},{7,1}
#define RunningAverageDist	4	// Threshold Between two consecutive points
#define MicroTimeTInterupt	10000000// 100Hz - 10,000,000us
#define FINGERNO		10

#define SCREEN_MAX_X    1600
#define SCREEN_MAX_Y    1024


struct ChipSetting ssd253xcfgTable[3]={
		{2, 0x06, 0x18, 0x0F},	// 24 Driving lines, 16 sensing lines
		{2, 0x28, 0x00, 0x14},  // Sense offset = 20
		{2, 0x65, 0x00, 0x05},  // Orientation register = Transpose + X Invert
};

struct ChipSetting Reset[]={
		{ 1, 0x01, 0x00, 0x00},	// Software reset 33command
};

struct ChipSetting Resume[]={
		{ 1, 0x04, 0x00, 0x00},	// System enable command
		{ 2, 0x25, 0x00, 0x0A}, // Set Operation Mode (10 ms)
};

struct ChipSetting Suspend[] ={
		{ 2, 0x25, 0x00, 0x00}, // Set Operation Mode (idle)
		{ 1, 0x05, 0x00, 0x00},	// System disable command
};

