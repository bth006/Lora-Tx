//BW
#define BW62_5Khz 0x06
#define BW125Khz 0x07
#define BW250Khz 0x08
#define BW500Khz 0x09
//CR
#define CR45 0x01
#define CR46 0x02
#define CR47 0x03
#define CR48 0x04
//ImplicitHeaderMode
#define ImplicitHeaderModeON 0x00
#define ImplicitHeaderModeOFF 0x01
//SF
#define SF6 0x06
#define SF7 0x07
#define SF8 0x08
#define SF9 0x09
#define SF10 0x0A
#define SF11 0x0B
#define SF12 0x0C
//CRC
#define CRCON 0x01
#define CRCOFF 0x00
//LowDataRateOptimize
#define LowDataRateOptimizeON 0x01
#define LowDataRateOptimizeOFF 0x00
//LowDataRateOptimize
#define ACGAUTO_ON 0x01
#define ACGAUTO_OFF 0x00

//these are the settings to be used
#define BW_SETTING BW250Khz
#define CR_SETTING CR48
#define SF_SETTING SF12
#define CRC_SETTING CRCON
#define ImplicitHeaderMode_SETTING ImplicitHeaderModeON
#define LowDataRateOptimize_SETTING LowDataRateOptimizeON
#define ACGAUTO_SETTING ACGAUTO_ON
