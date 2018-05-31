#include"../include/ps_gps/ps_gps_func.h"


// parse the GPS message with ANPP protocol
void gps_message_parse(ps_gps::gps &GPS, ps_anpp_gps_msg_t *gps_msg)
{
	GPS.lat = *((double *)gps_msg->data.lat)*180/PI;
	GPS.lat = check_valid("lat", GPS.lat);
			
	GPS.lon = *((double *)gps_msg->data.lon)*180/PI;
	GPS.lon = check_valid("lon", GPS.lon);
			
	GPS.hei = *((double *)gps_msg->data.hei);
	GPS.hei = check_valid("hei", GPS.hei);	
			
	GPS.vel_n = *((float *)gps_msg->data.vel_n);
	GPS.vel_n = check_valid("vel_n", GPS.vel_n);
			
	GPS.vel_e = *((float *)gps_msg->data.vel_e);
	GPS.vel_e = check_valid("vel_e", GPS.vel_e);
			
	GPS.vel_g = *((float *)gps_msg->data.vel_g);
	GPS.vel_g = check_valid("vel_g", GPS.vel_g);
			
	GPS.acc_x = *((float *)gps_msg->data.acc_x);
	GPS.acc_x = check_valid("acc_x", GPS.acc_x);
			
	GPS.acc_y = *((float *)gps_msg->data.acc_y);
	GPS.acc_y = check_valid("acc_y", GPS.acc_y);
			
	GPS.acc_z = *((float *)gps_msg->data.acc_z);
	GPS.acc_z = check_valid("acc_z", GPS.acc_z);
		
	GPS.gravity = *((float *)gps_msg->data.gravity) * 10;
	GPS.gravity = check_valid("gravity", GPS.gravity);
			
	GPS.roll = *((float *)gps_msg->data.roll)*180/PI;
	GPS.roll = check_valid("roll", GPS.roll);
			
	GPS.pitch = *((float *)gps_msg->data.pitch)*180/PI;
	GPS.pitch = check_valid("pitch", GPS.pitch);
			
	GPS.heading = *((float *)gps_msg->data.heading)*180/PI;
	GPS.heading = check_valid("heading", GPS.heading);
			
	GPS.ang_x = *((float *)gps_msg->data.ang_x)*180/PI;
	GPS.ang_x = check_valid("ang_x", GPS.ang_x);
			
	GPS.ang_y = *((float *)gps_msg->data.ang_y)*180/PI;
	GPS.ang_y = check_valid("ang_y", GPS.ang_y);
			
	GPS.ang_z = *((float *)gps_msg->data.ang_z)*180/PI;
	GPS.ang_z = check_valid("ang_z", GPS.ang_z);

}


// check var_name with var is valid?
double check_valid(const char* var_name, double var)
{
	if(!is_valid(var_name, var))
	{
		printf("%s error\n", var_name);
		return DATA_INVALID;
	}
	else
		return var;
}


// check the var with var_name is valid?
bool is_valid(const char* var_name, double var)
{
	if(!strcmp(var_name, "lat"))
	{
		if( var >= 0 && var <=90)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "lon"))
	{
		if( var >= 0 && var <=180)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "hei"))
	{
		if( var >= 0 && var <= 999)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "vel_e"))
	{
		if( var >= -50 && var <=50)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "vel_n"))
	{
		if( var >= -50 && var <=50)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "vel_g"))
	{
		if( var >= -10 && var <=10)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "acc_x"))
	{
		if( var >= -30 && var <=30)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "acc_y"))
	{
		if( var >= -30 && var <=30)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "acc_z"))
	{
		if( var >= -10 && var <=10)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "gravity"))
	{
		if( var >= 9 && var <=11)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "roll"))
	{
		if( var >= -180 && var <=360)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "pitch"))
	{
		if( var >= -180 && var <=360)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "heading"))
	{
		if( var >= 0 && var <=360)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "ang_x"))
	{
		if( var >= -180 && var <= 180)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "ang_y"))
	{
		if( var >= -180 && var <= 180)
			return 1;
		else
			return 0;
	}
	else if(!strcmp(var_name, "ang_z"))
	{
		if( var >= -180 && var <= 180)
			return 1;
		else
			return 0;
	}
}


// print usage of the node ps_serial
void usage(int argc, char* argv[])
{
	printf("maybe parameters is too many or have errors\n");
	printf("for example:\n 1. rosrun ps_gps ps_serial\n");
	printf("for example:\n 2. rosrun ps_gps ps_serial /dev/ttyUSB0\n");
	printf("for example:\n 3. roslaunch ps_gps gps.launch\n\n");
	printf("all parameters:\n");
	for (int i=0; i < argc; i++)
		printf("%s\n", argv[i]);
	printf("\n\n");
	exit(1);
}











