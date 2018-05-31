#ifndef _PS_TYPE_H_
#define _PS_TYPE_H_

#define CCT (+8)  
#define PI 3.1415926

typedef long time_t;      /* time value */  
typedef unsigned char u8;

typedef struct ANPP_HEAD
{
	u8 LRC;
	u8 ID;
	u8 packet_size;
	u8 CRC[2]; 
} ps_anpp_head_t;

typedef struct ANPP_DATA{  // this can malloc memory
	u8 system_state[2];
	u8 filter_state[2];
	u8 unix_time[4];
	u8 milisecond[4];
	u8 lat[8];
	u8 lon[8];
	u8 hei[8];
	u8 vel_n[4];
	u8 vel_e[4];
	u8 vel_g[4];
	u8 acc_x[4];
	u8 acc_y[4];
	u8 acc_z[4];
	u8 gravity[4];
	u8 roll[4];
	u8 pitch[4];
	u8 heading[4];
	u8 ang_x[4];
	u8 ang_y[4];
	u8 ang_z[4];
	u8 lat_SD[4];
	u8 lon_SD[4];
	u8 hei_SD[4];
	
} ps_anpp_data_t; // this can't malloc memery

typedef struct ANPP_GPS_MESSAGES
{
	struct ANPP_HEAD header;
	struct ANPP_DATA	  data;
} ps_anpp_gps_msg_t;


struct CTRL_MESSAGES{
	u8 speed;
	u8 steer;
	u8 power;
};

#endif

