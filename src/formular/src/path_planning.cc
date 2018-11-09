//#include "formular/path_planning.h"
#include "formular/formular.h"

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
			for(int k = 0; k != tree.size(); ++k)
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
	for(int i = 0; i != tree.size(); ++i)
	{
		if ((tree[i][0] != 0) && (tree[i][1] != 0))
		{
			lastnode_num = lastnode_num + 1;  //Í³ŒÆtreeÐŽµœÄÄÒ»ÐÐ
		}
	}
	int path_num = 0;
	for(int i = 0; i != path_x.size(); ++i)
	{
		path_num = path_num + 1;  //Í³ŒÆpath_xÐŽµœÄÄÒ»ÐÐ
	}
	int block_r = path_x[path_num - 1];
	int block_l = max(block_r - search_region, x_start);
	for(int i = 1; i != lastnode_num; ++i)
	{
		if ((tree[i][0]>block_l) && (tree[i][0] <= block_r))
		{
			double temp_distance = sqrt((tree[i][0] - startnode[0])*(tree[i][0] - startnode[0]) + (tree[i][1] - startnode[1])*(tree[i][1] - startnode[1]));
			distance.push_back(temp_distance);
			d_num.push_back(i);
		}
	}
	int MIN_row_num = 0;
	for(int i = 0; i != distance.size(); ++i)
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
	for(int i = (max(x_temp - car_size, x_start)); i <= (min(x_temp + car_size, x_end)); ++i)
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
		for(int i = 0; i != path_x.size(); ++i)
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
	for(int i = 0; i < mat_u.rows; ++i)
	{
		for(int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(point_set[i].x, j);
		}
	}
	for(int i = 0; i < mat_y.rows; ++i)
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
	for(int i = 0; i<s_temp.length(); i++)
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

/**************/
ros::Publisher pub_steer;
void cloud_cb(const sensor_msgs::PointCloud2 &cloud_msg)
{
	PointCloud cloud_center;
	pcl::fromROSMsg(cloud_msg, cloud_center);
	
	/**路径边缘拟合**/
	Mat map(500, 500, CV_8UC3, Scalar::all(0));  //用于显示的图像
	string s_temp;
	vector<Point>side_1, side_2;
	
	double x = 0;
	double y = 0;
	double z = 0;
	int len = cloud_center.points.size();
  	for(int i = 0; i < len - 1; i ++){
  		if(i == len - 2){
  			if(cloud_center.points[i].x < cloud_center.points[i+1].x){
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				side_1.push_back(Point(x, y));
				x = cloud_center.points[i+1].x;
				y = cloud_center.points[i+1].y;
				side_2.push_back(Point(x, y));
  			}
  			else{
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				side_2.push_back(Point(x, y));
				x = cloud_center.points[i+1].x;
				y = cloud_center.points[i+1].y;
				side_1.push_back(Point(x, y));
  			}
  		}
  		else {
  			if(cloud_center.points[i].x < cloud_center.points[i+1].x){
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				side_1.push_back(Point(x, y));
  			}
  			else{
  				x = cloud_center.points[i].x;
				y = cloud_center.points[i].y;
				side_2.push_back(Point(x, y));
  			}
  		}
	} 
	vector<Point> point_set_1(side_1.begin(), side_1.end()), point_set_2(side_2.begin(), side_2.end());
	int num = fit_num(side_1);  //多项式阶数，如路径点数少，而阶数过高，拟合会出错！
	Mat mat_k1 = polyfit(point_set_1, fit_num(side_1));
	Mat mat_k2 = polyfit(point_set_2, fit_num(side_2));
	for(int i = 0; i < side_1.size(); ++i)  //画出上部路径边缘的离散点
	{
		Point center_1 = side_1[i];
		circle(map, center_1, 4, Scalar(50,100,255), CV_FILLED, CV_AA);
	}
	for(int i = 0; i < side_2.size(); ++i)  //画出下部路径边缘的离散点
	{
		Point center_2 = side_2[i];
		circle(map, center_2, 4, Scalar(255,100,50), CV_FILLED, CV_AA);
	}
	vector<double>x1, y1;
	for(int i = 0; i < map.cols; ++i)  //上部路径边缘拟合曲线的坐标，i决定多项式的底数
	{
		Point2d center;
		center.x = i;  //要画的点的x,y坐标值，圆心坐标
		center.y = 0;
		for(int j = 0; j < num + 1; ++j)  //j决定多项式中每一项的幂次
		{
			center.y += mat_k1.at<double>(j, 0)*pow(i, j);
		}
		x1.push_back(center.x);
		y1.push_back(center.y);
		circle(map, center, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
	}
	vector<double>x2, y2;  //y1,y2的下标对应0-499，共500个数
	for(int i = 0; i < map.cols; ++i)  //下部路径边缘拟合曲线的坐标
	{
		Point2d center;
		center.x = i;  //要画的点的x,y坐标值，圆心坐标
		center.y = 0;
		for(int j = 0; j < num + 1; ++j)
		{
			center.y += mat_k2.at<double>(j, 0)*pow(i, j);
		}
		x2.push_back(center.x);
		y2.push_back(center.y);
		circle(map, center, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
	}
	/**路径区域划分**/
	int x_start = (side_1[0].x + side_2[0].x) / 2;
	int y_start = (side_1[0].y + side_2[0].y) / 2;
	int startnode[2] = { x_start,y_start };  //起点
	int x_end = (side_1[side_1.size() - 1].x + side_2[side_2.size() - 1].x) / 2;
	int y_end = (side_1[side_1.size() - 1].y + side_2[side_2.size() - 1].y) / 2;
	int endnode[2] = { x_end,y_end };  //终点
	for(int col = 0; col <map.cols; ++col)
	{
		for(int row = 0; row < map.rows; ++row)
		{
			if (col < x_start)
			{
				map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //x_start左边区域标红
			}
			else
			{
				if (col <= x_end)
				{
					if (row <y1[col])
					{
						map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //BGR，上部障碍线以上区域标红
					}
					if (row > y2[col])
					{
						map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //下部障碍线以下区域标红
					}
				}
				else
				{
					map.at<Vec3b>(row, col) = Vec3b(0, 0, 255);  //x_end右边区域标红
				}
			}
		}
	}
									   /**路径关键参数设定**/
	clock_t start, finish;
	start = clock();  //开始计时
	srand((unsigned)time(NULL));  //这句不能放在主函数外面;如没有这句，每次编译产生的随机数是一样的！
	int filled_num = 0;
	int greed_flag = 1;
	double newpoint[2] = { 0,0 };
	vector<vector<int> >tree(1000);
	for(int i = 0; i < 1000; ++i)
	{
		tree[i].resize(2);
	}
	tree[0][0] = startnode[0];
	tree[0][1] = startnode[1];
	int x_picture = map.cols;  //图像大小
	int y_picture = map.rows;
	//**贪婪RRT路径规划**//
	vector<int>path_x, path_y;  //路径节点的x,y坐标
	int success = 0;
	while (success == 0)
	{
		filled_num = 0;  //每次统计前先归零
		for(int i = 0; i != tree.size(); ++i)
		{
			if ((tree[i][0] != 0) & (tree[i][1] != 0))
			{
				filled_num = filled_num + 1;  //统计tree写到哪一行了
			}
		}
		for(int i = 0; i != 2; ++i)  //每次产生两个可行点
		{
			bias_extend_tree(x_picture, y_picture, map, endnode, newpoint, tree, y1, y2, filled_num, greed_flag, x_start, x_end);
		}
		for(int i = 0; i != tree.size(); ++i)
		{
			double x_final_deviation = tree[i][0] - endnode[0];
			double y_final_deviation = tree[i][1] - endnode[1];  //计算每次找到的新节点与终点的距离，决定是否继续寻找新节点
			double final_deviation = sqrt(x_final_deviation*x_final_deviation + y_final_deviation * y_final_deviation);
			if (final_deviation < step)
			{
				cout << "已到达终点！" << endl;
				success = 1;  //说明已经到达终点，不必再继续拓展节点
				break;
			}
			else
			{
				success = 0;  //返回0后，主函数中会重新进入这个while循环，从而搜索下一个节点
			}
		}
	}
	finish = clock();  //结束计时
	double time_consuming = double(finish - start) / CLOCKS_PER_SEC;  //计算执行程序所用时间（秒）
	cout << "本次求解时间为：" << time_consuming << "秒" << endl;
	cout << "开始最优路径回溯" << endl;
	path_x.push_back(endnode[0]);  //记录终点坐标
	path_y.push_back(endnode[1]);
	find_path(path_x, path_y, tree, startnode, x_start, x_end);
	path_x.push_back(startnode[0]);  //记录起点坐标
	path_y.push_back(startnode[1]);
	cout << "最优路径回溯结束！" << endl;
	//**最终路径绘制**//
	Point p_start, p_end;
	p_start.x = startnode[0];
	p_start.y = startnode[1];
	circle(map, p_start, 5, Scalar(255, 255, 255), -1);   //画出起点
	p_end.x = endnode[0];
	p_end.y = endnode[1];
	circle(map, p_end, 5, Scalar(255, 255, 255), -1);  //画出终点
	int lastnode_num = 0;
	for(int i = 0; i != tree.size(); ++i)
	{
		if ((tree[i][0] != 0) && (tree[i][1] != 0))
		{
			lastnode_num = lastnode_num + 1;  //统计tree写到哪一行
		}
	}
	for(int i = 0; i != lastnode_num-1; ++i)  //画出搜索树的所有节点
	{
		Point p;
		p.x = tree[i][0];
		p.y = tree[i][1];    
		circle(map, p, 4, Scalar(200, 200, 0), -1); //画点：第三个参数为线宽，第五个参数设为-1，表明是个实点
		Point start_point = Point(tree[i][0], tree[i][1]);
		Point end_point = Point(tree[i + 1][0], tree[i + 1][1]);
	}
	for(int i = 0; i != path_x.size()-1;++i)  //画出回溯得到的最优路径
	{
		Point p;
		p.x = path_x[i];
		p.y = path_y[i];   
		circle(map, p, 2, Scalar(255, 50, 0), -1); //画点：第三个参数为线宽，第五个参数设为-1，表明是个实点
		Point start_point = Point(path_x[i], path_y[i]);
		Point end_point = Point(path_x[i + 1], path_y[i + 1]);
		line(map, start_point, end_point, Scalar(255, 0, 0), 3);  //画线
	}
	//**计算导向线**//
	vector<Point> point_set_line;
	for(int i = path_x.size() - 3; i < path_x.size(); ++i)
	{
		int x = path_x[i];
		int y = path_y[i];
		point_set_line.push_back(Point(x, y));
	}
	int num_line = fit_num(point_set_line);  //初始点开始的几个点的拟合
	Mat mat_k_line = polyfit(point_set_line, num_line);
	vector<double>x_direction_line, y_direction_line;
	for(int i = path_x[path_x.size() - 3]; i >= path_x[path_x.size() - 1]; --i)  //画出上部路径边缘拟合曲线，i决定多项式的底数
	{
		Point2d center;
		center.x = i;  //要画的点的x,y坐标值，圆心坐标
		center.y = 0;
		for(int j = 0; j < num_line + 1; ++j)  //j决定多项式中每一项的幂次
		{
			center.y += mat_k_line.at<double>(j, 0)*pow(i, j);
		}
		x_direction_line.push_back(center.x);
		x_direction_line.push_back(center.y);
		circle(map, center, 1, Scalar(160, 160, 0), CV_FILLED, CV_AA);
	}
	cout << "程序运行结束！" << endl;
	imshow("rrt", map);//贪婪RRT得到的路径
	waitKey(0);
	
	
	std_msgs::Float64 steer;
	steer.data = cloud_center.points[0].x;
	pub_steer.publish(steer);

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "steer");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("cluster_points", 10, cloud_cb);
	pub_steer = n.advertise<std_msgs::Float64> ("formular_steer", 10);
	ros::spin();
}
