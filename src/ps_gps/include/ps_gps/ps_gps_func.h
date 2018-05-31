#ifndef PS_GPS_FUNC_H_
#define PS_GPS_FUNC_H_


#include<string.h>
#include<stdio.h>


#include"../include/ps_gps/ps_type.h"
#include"ps_gps/gps.h"


#define LONGITUDE_MAX 180.0f
#define DATA_INVALID -360.0

bool is_valid(const char[], double);
double check_valid(const char* var_name, double var);
void gps_message_parse(ps_gps::gps &GPS, ps_anpp_gps_msg_t *gps_msg);
void usage(int argc, char* argv[]);

#endif
