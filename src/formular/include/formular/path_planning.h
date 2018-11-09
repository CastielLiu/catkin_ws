#ifndef __PATH_PLANNING_H_
#define __PATH_PLANNING_H_

#include <iostream>
#include <vector>
#include<ctime>  
#include<math.h>  
#include<fstream>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#define step 50
#define car_size 10
#define search_region 200  
#define big_M 1000
#define max(a,b)  ((a >= b ) ? a : b)
#define min(a,b)  ((a <= b ) ? a : b)
#define random_int(a,b) (rand()%(b-a+1)+a)  
#define random_double(a,b) (rand()/double(RAND_MAX))  




/************************º¯ÊýÉùÃ÷************************/
void bias_extend_tree(int(&x_picture), int(&y_picture), Mat(&map), const int endnode[2], double(&newpoint)[2], vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&greed_flag), int(&x_start), int(&x_end));
void find_path(vector<int>(&path_x), vector<int>(&path_y), vector<vector<int> >(&tree), int(&startnode)[2], int(&x_start), int(&x_end));
int collision(double(&newpoint)[2], Mat(&map), int x_picture, int y_picture, vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&x_start), int(&x_end));
int search_nearnode(vector<vector<int> >(&tree), vector<int>(&path_x), vector<int>(&path_y), int(&startnode)[2], int(&x_start), int(&x_end));
int fit_num(vector<Point>data_set);
Mat polyfit(vector<Point>&(point_set), int n);
void string_to_num(string s_temp, int* input_data);
/*************************º¯Êý¶šÒå*************************/
/**žÄœøºóµÄÆ«ÏòÄ¿±êÐÍRRTËã·š**/
void bias_extend_tree(int(&x_picture), int(&y_picture), Mat(&map), const int endnode[2], double(&newpoint)[2], vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&greed_flag), int(&x_start), int(&x_end))
{
	int find_onenode_flag = 0;
	int randpoint[2] = { 0,0 };
	while (find_onenode_flag != 1)  // Ã¿ŽÎwhileÑ­»·£¬¶Œ»áÕÒµœÒ»žö¿ÉÐÐµÄœÚµã£¬ŽÓ¶øÍØÕ¹Â·Ïß
	{
		double rand_num = random_double(0, 1);
		if ((greed_flag == 1) || (rand_num < 0.5))  //ÉÏŽÎÍùÄ¿±êµã·œÏòµÄÍØÕ¹ÎŽÊÜµœ×è°­,»ò²úÉúµÄËæ»úÊýÐ¡ÓÚÆ«ÖÃžÅÂÊ£¬Ö±œÓÒÔÖÕµãÎªÄ¿±êµãœøÐÐÍØÕ¹
		{
			randpoint[0] = endnode[0];
			randpoint[1] = endnode[1];
		}
		else
		{
			randpoint[0] = random_int(tree[filled_num - 1][0], x_end);
			int a = y1[randpoint[0]], b = y2[randpoint[0]];  //°Ñdouble×ª»¯³Éint
			randpoint[1] = random_int(a, b);
		}
		double x_deviation = 0;
		double y_deviation = 0;
		if ((randpoint[0] == tree[filled_num - 1][0]) || (randpoint[1] == tree[filled_num - 1][1]))
		{
			x_deviation = (randpoint[0] + 2) - tree[filled_num - 1][0];
			y_deviation = (randpoint[1] + 2) - tree[filled_num - 1][1];
		}
		else
		{
			x_deviation = randpoint[0] - tree[filled_num - 1][0];
			y_deviation = randpoint[1] - tree[filled_num - 1][1];
		}
		double x_deviation_abs = fabs(x_deviation);
		double y_deviation_abs = fabs(y_deviation);
		newpoint[0] = tree[filled_num - 1][0] + (x_deviation / x_deviation_abs)*(random_int(1, step));  //ŽÓËÑË÷µœµÄÏàÁÚœÚµãÏòËæ»úœÚµã·œÏòÍØÕ¹Ëæ»ú²œ³€ŸàÀë
		newpoint[1] = tree[filled_num - 1][1] + (y_deviation / y_deviation_abs)*(random_int(1, step));
		if (collision(newpoint, map, x_picture, y_picture, tree, y1, y2, filled_num, x_start, x_end) == 0)
		{
			for (int k = 0; k != tree.size(); ++k)
			{
				if ((tree[k][0] == 0) && (tree[k][1] == 0))  //Ã¿ÕÒµœÒ»žö¿ÉÐÐµÄœÚµã£¬œ«ÆäŒÓÈëtreeÖÐ
				{
					tree[k][0] = newpoint[0];
					tree[k][1] = newpoint[1];
					break;
				}
			}
			if ((randpoint[0] == endnode[0]) && (randpoint[1] == endnode[1]))
			{
				greed_flag = 1;
			}
			else
			{
				greed_flag = 0;
			}
			find_onenode_flag = 1;  //ÕÒµœÒ»žöÐÂœÚµã£¬œáÊøÉÏÃæµÄwhileÑ­»·
		}
		else
		{
			find_onenode_flag = 0;  //ÖØžŽÑ­»·Ö±µœÕÒµœ¿ÉÐÐœÚµã
		}
	}
}
/**ÐÂœÚµãÑ¡Ôñº¯Êý**/
//×ÛºÏ¿ŒÂÇŸàÀëÓëÇúÂÊ£¬Ñ¡È¡ÇúÂÊÓëŸàÀëŒÓÈšºÍ×îÐ¡µÄœÚµã×÷ÎªÐÂœÚµã
int search_nearnode(vector<vector<int> >(&tree), vector<int>(&path_x), vector<int>(&path_y), int(&startnode)[2], int(&x_start), int(&x_end))
{
	vector<double>distance;
	vector<int>d_num;
	double distance_MIN = big_M;
	int lastnode_num = 0;
	for (int i = 0; i != tree.size(); ++i)
	{
		if ((tree[i][0] != 0) && (tree[i][1] != 0))
		{
			lastnode_num = lastnode_num + 1;  //Í³ŒÆtreeÐŽµœÄÄÒ»ÐÐ
		}
	}
	int path_num = 0;
	for (int i = 0; i != path_x.size(); ++i)
	{
		path_num = path_num + 1;  //Í³ŒÆpath_xÐŽµœÄÄÒ»ÐÐ
	}
	int block_r = path_x[path_num - 1];
	int block_l = max(block_r - search_region, x_start);
	for (int i = 1; i != lastnode_num; ++i)
	{
		if ((tree[i][0]>block_l) && (tree[i][0] <= block_r))
		{
			double temp_distance = sqrt((tree[i][0] - startnode[0])*(tree[i][0] - startnode[0]) + (tree[i][1] - startnode[1])*(tree[i][1] - startnode[1]));
			distance.push_back(temp_distance);
			d_num.push_back(i);
		}
	}
	int MIN_row_num = 0;
	for (int i = 0; i != distance.size(); ++i)
	{
		if (distance[i] < distance_MIN)
		{
			distance_MIN = distance[i];
			MIN_row_num = d_num[i];
		}
	}
	return MIN_row_num;  //·µ»ØŸàÀë×îœüµÄµãÔÚtreeÖÐ¶ÔÓŠµÄÐÐÊý
}
/**Åö×²Œì²âº¯Êý**/
int collision(double(&newpoint)[2], Mat(&map), int x_picture, int y_picture, vector<vector<int> >(&tree), vector<double>y1, vector<double>y2, int(&filled_num), int(&x_start), int(&x_end))
{
	int collision_flag_1 = 0;  //ÅÐ¶ÏÐÂµãžœœüÊÇ·ñŽæÔÚÕÏ°­Îï
	int x_temp = newpoint[0];
	int y_temp = newpoint[1];
	for (int i = (max(x_temp - car_size, x_start)); i <= (min(x_temp + car_size, x_end)); ++i)
	{
		if (((y_temp - car_size) > y1[i]) && ((y_temp + car_size) < y2[i]))
		{
			collision_flag_1 = 0;  //²»»áÅö×²£¬žÃµã¿ÉÐÐ
		}
		else
		{
			collision_flag_1 = 1;
			break;
		}
	}
	return collision_flag_1;
}
/**Ëæ»úËÑË÷Íê³Éºó£¬ŽÓÖÕµã×·ËÝ¿ÉÐÐÂ·Ÿ¶**/
void find_path(vector<int>(&path_x), vector<int>(&path_y), vector<vector<int> >(&tree), int(&startnode)[2], int(&x_start), int(&x_end))
{
	int MIN_row_num = 0;
	double distance = big_M;
	while (distance>step)
	{
		MIN_row_num = search_nearnode(tree, path_x, path_y, startnode, x_start, x_end);  //ŒÓÈë×·ËÝÂ·Ÿ¶µÄœÚµãÐòºÅ
		path_x.push_back(tree[MIN_row_num][0]);
		path_y.push_back(tree[MIN_row_num][1]);
		int path_num = 0;
		for (int i = 0; i != path_x.size(); ++i)
		{
			path_num = path_num + 1;  //Í³ŒÆpath_xÐŽµœÄÄÒ»ÐÐ
		}
		distance = sqrt((startnode[0] - path_x[path_num - 1])*(startnode[0] - path_x[path_num - 1]) + (startnode[1] - path_y[path_num - 1])*(startnode[1] - path_y[path_num - 1]));
	}
	double temp_distance = sqrt((path_x[path_x.size() - 1] - startnode[0])*(path_x[path_x.size() - 1] - startnode[0]) + (path_y[path_y.size() - 1] - startnode[1])* (path_y[path_y.size() - 1] - startnode[1]));
	if (temp_distance <40)
	{
		path_x.pop_back();
		path_y.pop_back();
	}
	cout << "Â·Ÿ¶»ØËÝœáÊø£¡" << endl;
}
int fit_num(vector<Point>data_set)
{
	int fit_num = 0;
	if (data_set.size() >= 8)  //žùŸÝÄâºÏµãÊýÁ¿Ÿö¶š¶àÏîÊœµÄÄâºÏœ×Êý
	{
		fit_num = 8;
	}
	else
	{
		if (data_set.size() <= 2)
		{
			fit_num = 1;
		}
		else
		{
			fit_num = 2;
		}
	}
	return fit_num;
}
Mat polyfit(vector<Point>&(point_set), int n)
{
	int size = point_set.size();  //ÄâºÏµãµÄÊýÄ¿
	int x_num = n + 1;
	Mat mat_u(size, x_num, CV_64F);  //¹¹ÔìŸØÕóUºÍY
	Mat mat_y(size, 1, CV_64F);
	for (int i = 0; i < mat_u.rows; ++i)
	{
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(point_set[i].x, j);
		}
	}
	for (int i = 0; i < mat_y.rows; ++i)
	{
		mat_y.at<double>(i, 0) = point_set[i].y;
	}
	Mat mat_k(x_num, 1, CV_64F); //ŸØÕóÔËËã£¬»ñµÃÏµÊýŸØÕóK 
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	cout << mat_k << endl;
	return mat_k;
}
void string_to_num(string s_temp, int* input_data)
{
	bool temp = false;		//¶ÁÈ¡Ò»žöÊýŸÝ±êÖŸÎ»
	int data = 0;				//·ÖÀëµÄÒ»žöÊýŸÝ
	int m = 0;				//Êý×éË÷ÒýÖµ
	for (int i = 0; i<s_temp.length(); i++)
	{
		while ((s_temp[i] >= '0') && (s_temp[i] <= '9'))		//µ±Ç°×Ö·ûÊÇÊýŸÝ£¬²¢Ò»Ö±¶ÁºóÃæµÄÊýŸÝ£¬Ö»ÒªÓöµœ²»ÊÇÊý×ÖÎªÖ¹
		{
			temp = true;		//¶ÁÊýŸÝ±êÖŸÎ»ÖÃÎ»
			data *= 10;
			data += (s_temp[i] - '0');		//×Ö·ûÔÚÏµÍ³ÒÔASCIIÂëŽæŽ¢£¬ÒªµÃµœÆäÊµŒÊÖµ±ØÐëŒõÈ¥¡®0¡¯µÄASCIIÖµ
			i++;
		}
		if (temp)		//ÅÐ¶ÏÊÇ·ñÍêÈ«¶ÁÈ¡Ò»žöÊýŸÝ
		{
			input_data[m] = data;		//ž³Öµ
			m++;
			data = 0;
			temp = false;		//±êÖŸÎ»žŽÎ»
		}
	}
}

#endif// !Â·Ÿ¶¹æ»®Í·ÎÄŒþ

