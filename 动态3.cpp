#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iomanip> //////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <WINSOCK2.H>
#include "Ws2tcpip.h"
#include "Winsock2.h"
#include "zss_cmd.pb.h"
#include "vision_detection.pb.h"
#include "zss_debug.pb.h"
#include <random>
#include <ctime>
#include <vector>
#include <cmath>
#include <algorithm> 
#include <cstdio> 
#include <cstdlib> 
using std::cin;
using std::cout;
using std::cerr;
using std::endl;
double ROBO[30][2];
int NumPoint = 0, IniSizeBlue,Score=-1;
double cost_path[500001];//500*500的地图共250000个点，留有冗余
double rrt_point[10000][2];
#define PI 3.14159265
#define ME 0
#define MAX_V 300
#define mid_JS_range 100
#define final_JS_range 120
double dis_lev = 0;
int Count = 0;
double D_heading[2000];
Vision_DetectionFrame frame;
Vision_DetectionBall  balls;
Vision_DetectionRobot robots_yellow;
void getROBO()
{
	for (int ii = 0; ii < frame.robots_blue_size(); ++ii)
	{
		int i = frame.robots_blue(ii).robot_id();
		//cout << "RBid=   " << i << endl;
		Vision_DetectionRobot robots_blue = frame.robots_blue(ii);
		ROBO[i][1] = robots_blue.x() / 10.0 + 300;
		ROBO[i][0] = 450 - (robots_blue.y() / 10.0 + 225);
		//cout << ROBO[i][0] << " " << ROBO[i][1] << endl;
		//cout << /*robots_blue.raw_vel_x() << robots_blue.raw_vel_y() << */robots_blue.orientation() << endl;
	}
	for (int ii = 0; ii < frame.robots_yellow_size(); ++ii)
	{
		int i = frame.robots_yellow(ii).robot_id();
		//cout << "RBid=   " << i << endl;
		Vision_DetectionRobot robots_yellow = frame.robots_yellow(ii);
		ROBO[i + 16][1] = robots_yellow.x() / 10.0 + 300;
		ROBO[i + 16][0] = 450 - (robots_yellow.y() / 10.0 + 225);
		//cout << ROBO[i][0] << " " << ROBO[i][1] << endl;
	}
}
double barrier_x[35];
double barrier_y[35];
double barrier_angle[35];
double distance[35];
double barrier_V[35];


double board_angle[8];
double board_dis[8];
double board_V[8];

void getBarrier()
{
	for (int i = 0; i <= 31; ++i) barrier_x[i] = barrier_y[i] = -1;

	for (int ii = 0; ii < frame.robots_blue_size(); ++ii)
	{
		int i = frame.robots_blue(ii).robot_id();
		//cout << "RBid=   " << i << endl;
		barrier_x[i] = frame.robots_blue(ii).x() / 10;
		barrier_y[i] = -frame.robots_blue(ii).y() / 10;
		//cout << barrier_x[i] << "  " << barrier_y[i] << endl;
	}
	for (int ii = 0; ii < frame.robots_yellow_size(); ++ii)
	{
		int i = frame.robots_yellow(ii).robot_id();
		//cout << "RBid=   " << i << endl;
		barrier_x[i + 16] = frame.robots_yellow(ii).x() / 10;
		barrier_y[i + 16] = -frame.robots_yellow(ii).y() / 10;
		//cout << barrier_x[i+8] << "  " << barrier_y[i+8] << endl;
	}

}

double MAX(double a, double b)
{
	return (a > b) ? a : b;
}
struct point
{
	int parent;
	double pos_x, pos_y;
};
int getIndex(double x, double y)
{
	return int(int(x - 1) * 600 + int(y));
}

char* getIP()
{
	BYTE minorVer = 2;
	BYTE majorVer = 2;
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(minorVer, majorVer);
	WSAStartup(sockVersion, &wsaData);
	int i;
	char szHost[256];
	gethostname(szHost, 256);
	hostent *pHost = gethostbyname(szHost);
	in_addr addr;

	char *p = pHost->h_addr_list[0];
	memcpy(&addr.S_un.S_addr, p, pHost->h_length);
	char *szIp = inet_ntoa(addr);
	return (szIp);
}

double measure_distance(point x, point y)
{
	return sqrt(pow(fabs(x.pos_x - y.pos_x), 2) + pow(fabs(x.pos_y - y.pos_y), 2));
}
int WALL(double x, double y)
{
	for (int i = 0; i <= 31; ++i)
		if ((ROBO[i][0] != -1 || ROBO[i][1] != -1) && i != ME)
			if (sqrt(pow(fabs(x - ROBO[i][0]), 2) + pow(fabs(y - ROBO[i][1]), 2)) < 32.0)
				return 1;
	return 0;
}
int argmin(std::vector<point>& seeds, point seed)
{
	int result = 0;
	double minimun = 1.0e10;
	for (int i = 0; i < seeds.size(); i++)
	{
		double dis = measure_distance(seeds[i], seed);
		if (dis < minimun)
		{
			result = i;
			minimun = dis;
		}
	}
	return result;
}
int detect(point a, point b)
{
	double deltx = (b.pos_x - a.pos_x) / 100;
	double delty = (b.pos_y - a.pos_y) / 100;
	for (double i = 1; i <= 100; i += 1.0) {
		if (WALL(a.pos_x + i * deltx, a.pos_y + i * delty) > 0) return 1;
	}
	return 0;
}
#pragma comment(lib, "ws2_32.lib")
double Measure_Dis(double x, double y, double x1, double Y1) 
{
	return (sqrt(pow(x - x1, 2) + pow(y - Y1, 2)));
}
bool has_arrive(double x, double y, double x1, double y1, double dis_lev) {

	if (sqrt(pow(x - x1, 2) + pow(y - y1, 2)) < dis_lev)
		return 1;
	else
		return 0;
}
double compute_heading(double x, double y, double head_x, double head_y, double theta) {
	double bias = 0.0;
	if ((head_x - x) < 0.0) bias = 3.14159265;
	//printf(" jiajiao: %.4lf\n", atan((head_y - y) / (head_x - x))+bias);
	return atan((head_y - y) / (head_x - x)) + bias - theta;
}
double compute_angle(double x, double y, double headx, double heady) {
	double bias = 0.0;
	if ((headx - x) < 0.0) bias = 3.14159265;
	//printf(" jiajiao: %.4lf\n", atan((head_y - y) / (head_x - x))+bias);
	return atan((heady - y) / (headx - x)) + bias;
}
int SOCKADDR_IN_SIZE = sizeof(SOCKADDR_IN);
const u_short DEFAULT_PORT = 23333;
const size_t MSG_BUF_SIZE = 2048;
const size_t IP_BUF_SIZE = 256;
//const int Port = 50001;

int RRTstar(double start_x, double start_y, double end_x, double end_y, double beta)
{
	int xs_ = 450, ys_ = 600;
	int ns_ = xs_ * ys_;
	int lethal_cost_ = 253; //barrier height 
	double step = 25.0; //调节每步最小步长 
	std::vector<point> seeds; //随机生成点的集合 
	point end_point; //point struct记录其xy坐标与父结点在seeds中的下标 
	end_point.pos_x = end_x;
	end_point.pos_y = end_y;
	point current;
	current.pos_x = start_x;
	current.pos_y = start_y;
	current.parent = -1; //出发点的父结点为源点-1 
	//cout << "SXSY EXEY ";
	//cout << current.pos_x << " " << current.pos_y << " " << end_point.pos_x << " " << end_point.pos_y;    // set x and set y  

	int Xmin;
	seeds.push_back(current); //加入vector，进行广搜  
	int cycles = 0;
	//for (int i = 0; i < 500000; ++i) cost_path[i] = 5000;
	cost_path[getIndex(current.pos_x, current.pos_y)] = 0;//除起点外距离均为max，max为防止累加导致溢出不宜过大，如0x3f3f3f3，本地图5000足够大。 
	//std::default_random_engine generator(time(NULL));//依系统时间为种子生成随机数 

	clock_t st; st = clock();
	int Able = 0;
	while (cycles < 5000)
	{
		cycles++;
		//std::uniform_int_distribution<int> random_gen_y(0, 600);
		//std::uniform_int_distribution<int> random_gen_x(0, 450);
		int ran_x = rand() % 400 + 25;
		int ran_y = rand() % 550 + 25;

		current.pos_x = ran_x;
		current.pos_y = ran_y; //X.current<-X.rand
		int nearest = argmin(seeds, current); //get X.nearest
		double minimum = measure_distance(seeds[nearest], current);
		if (measure_distance(seeds[nearest], current) < step)
		{
			cycles--; //离X.nearest距离不足最小步长，则重新随机取点
			continue;
		}
		else {
			double a = 1.0;//前往随机采样点方向的增长系数
			double distance_x_end = end_point.pos_x - seeds[nearest].pos_x;
			double distance_y_end = end_point.pos_y - seeds[nearest].pos_y;
			double delta_x = current.pos_x - seeds[nearest].pos_x;
			double delta_y = current.pos_y - seeds[nearest].pos_y;

			double rate = sqrt(pow(step, 2) / (pow(fabs(delta_x), 2) + pow(fabs(delta_y), 2)));
			double rate_end = sqrt(pow(step, 2) / (pow(fabs(distance_x_end), 2) + pow(fabs(distance_y_end), 2)));
			//以线性的步长/直线距离为扩展率，离得越近“拉力”越大
			double forward_x = seeds[nearest].pos_x + delta_x * rate * a + distance_x_end * rate_end * beta;
			double forward_y = seeds[nearest].pos_y + delta_y * rate * a + distance_y_end * rate_end * beta;
			//依系数加权合成X.new位置
			if (WALL(forward_x, forward_y) > 0)
			{
				//发现目标点撞墙
				//cout << "shit" << forward_x << " " << forward_y << endl;
				double forward_x_again = seeds[nearest].pos_x + delta_x * rate * a;
				double forward_y_again = seeds[nearest].pos_y + delta_y * rate * a; //去掉目标点dx、dy对选点的影响
				if (WALL(forward_x_again, forward_y_again) > 0) //还是撞墙，则“弃疗”，否则保留
					continue;
				else
				{
					forward_x = forward_x_again;
					forward_y = forward_y_again;
				}
			}

			current.pos_x = forward_x;
			current.pos_y = forward_y;
			current.parent = nearest; //更新X.current为X.Now（已在vector的最后一位）
			seeds.push_back(current);
			if (measure_distance(end_point, current) < step)
			{
				Able = 1;
				break;//到达终点附近，结束
			}
		}
	}
	end_point.parent = seeds.size() - 1;//vector的最后一位即current的下标
	seeds.push_back(end_point);

	cout << "Time: " << (double)clock() - st << endl;
	//std::pair<float, float> now;
	NumPoint = 0;
	rrt_point[++NumPoint][0] = end_point.pos_x;
	rrt_point[NumPoint][1] = end_point.pos_y;
	//cout << "END P_xy:  " << end_point.pos_x << " " << end_point.pos_y << endl;
	while (current.parent != -1)
	{
		//now.first = current.pos_x;
		//now.second = current.pos_y;
		int lzmax = 2;
		if (seeds[current.parent].parent != -1)
			while (detect(seeds[seeds[current.parent].parent], current) == 0)
			{
				lzmax--;
				if (lzmax == 0) break;
				//cout << "lazy" << endl;
				current.parent = seeds[current.parent].parent;
				if (seeds[current.parent].parent == -1) break;
			}
		rrt_point[++NumPoint][0] = current.pos_x;
		rrt_point[NumPoint][1] = current.pos_y;
		//cout << current.pos_x << " " << current.pos_y << endl;
		//path.push_back(now); //path记录路径上的结点并输出
		current = seeds[current.parent]; //往父亲结点迭代
	}
	rrt_point[++NumPoint][0] = start_x;
	rrt_point[NumPoint][1] = start_y;
	//cout << "START:  " << start_x << "  " << start_y << endl;
	for (int i = NumPoint - 1; i > 1; --i)
	{
		double heading1 = compute_angle(rrt_point[i + 1][1], rrt_point[i + 1][0], rrt_point[i][1], rrt_point[i][0]);
		double heading2 = compute_angle(rrt_point[i][1], rrt_point[i][0], rrt_point[i - 1][1], rrt_point[i - 1][0]);
		D_heading[i] = fabs(heading1 - heading2);
		if (D_heading[i] > PI*1.5) D_heading[i] = fabs(D_heading[i] - 2 * PI);
		else if (D_heading[i] > PI) D_heading[i] = fabs(D_heading[i] - PI);
	}
	return Able;
}
Debug_Msgs draw_msgs;
char msg[16384];
char bufferf[1024];

double compute_velocity(double distance, double X, double Y, int id_num, int flag) {
	double VELOCITY;
	double V0 = 0;
	getBarrier();
	double My_X = barrier_x[ME];
	double My_Y = barrier_y[ME];

	if (id_num <= 2 && distance < final_JS_range) {
		double dx = final_JS_range - dis_lev;
		double dy = MAX_V - 80;
		double k = dy / dx;
		V0 = VELOCITY = min(MAX(MAX_V + (distance - final_JS_range) * k, 50), 300);

	}
	else if (distance < mid_JS_range)
	{
		double dx = mid_JS_range - dis_lev;
		double k;
		if (D_heading[id_num] <= PI / 6) k = ((MAX_V - 150) * 6 * D_heading[id_num]) / (dx * 3.1415926);
		else if (D_heading[id_num] <= PI / 3) k = ((MAX_V - 100) * 3 * D_heading[id_num]) / (dx * 3.1415926);
		else k = ((MAX_V - 50) * 2 * D_heading[id_num]) / (dx * 3.1415926);
		V0 = VELOCITY = MAX(MAX_V + (distance - mid_JS_range) * k, 50);

		
	}
	else
	{
		//double dx = 76.0;
		//double dy = 200.0;
		double k = 8;
		distance = sqrt(pow(rrt_point[id_num + 1][1] - 300 - X, 2) + pow(rrt_point[id_num + 1][0] - 225 - Y, 2));
		VELOCITY = min(MAX(30 + distance * k, V0), MAX_V);

	}
	return VELOCITY;
}
/*
double compute_velocity(double distance, double X, double Y, int id_num, int flag) {
	double VELOCITY;
	double V0 = 0;
	double V1 = MAX_V;
	getBarrier();
	double My_X = barrier_x[ME];
	double My_Y = barrier_y[ME];
	
	if ((id_num <= 1) && distance < final_JS_range) {
		
		double Vt = 0;
		double k = (pow(V1, 2) - pow(Vt, 2)) / 2.0 / (final_JS_range - dis_lev);
		V0 = VELOCITY = MAX(sqrt(pow(V1, 2) + 2 * k*(distance - final_JS_range)), 30);


	}
	else if (distance < mid_JS_range)
	{
		
		double Vt = 100;
		if (D_heading[id_num] >= PI / 3) Vt = 50;
		double k = (pow(V1, 2) - pow(Vt, 2)) / 2.0 / (mid_JS_range - dis_lev);
		V0 = VELOCITY = MAX(sqrt(pow(V1, 2) + 2 * k*(distance - mid_JS_range)), Vt);

	}
	else
	{
		
		double a_dis = 100;
		double k = (pow(MAX_V, 2) - pow(V0, 2)) / 2.0 / a_dis;
		distance = sqrt(pow(rrt_point[id_num + 1][1] - 300 - X, 2) + pow(rrt_point[id_num + 1][0] - 225 - Y, 2));
		V1 = VELOCITY = min(MAX(sqrt(pow(V0, 2) + 2 * k*distance), V0), MAX_V);

	}
	return VELOCITY;
}
*/
void SendDebug()
{
	Debug_Msg* draw_msg;
	for (int i = 1; i < NumPoint; ++i)
	{
		draw_msg = draw_msgs.add_msgs();
		draw_msg->set_type(Debug_Msg_Debug_Type::Debug_Msg_Debug_Type_LINE);
		draw_msg->set_color(Debug_Msg_Color::Debug_Msg_Color_RED);
		Debug_Line* line = draw_msg->mutable_line();
		Point* start = line->mutable_start();
		Point* end = line->mutable_end();
		//start = new Point();
		//end = new Point();
		//line = new Debug_Line();
		start->set_x(rrt_point[i][1] - 300);
		start->set_y(-rrt_point[i][0] + 225);
		end->set_x(rrt_point[i + 1][1] - 300);
		end->set_y(-rrt_point[i + 1][0] + 225);
		//cout << rrt_point[i][1]-300 << " " << rrt_point[i][0]-225 << endl;
		//cout << rrt_point[i+1][1] - 300 << " " << rrt_point[i+1][0] - 225 << endl;
		/*line->set_allocated_start(start);
		line->set_allocated_end(end);*/
		line->set_forward(1);
		line->set_back(1);
		//draw_msg->set_allocated_line(line);
	}
	draw_msgs.SerializeToArray(msg, 16384);
	draw_msgs.Clear();
}



void second_debug(Robots_Command&rcs) {
	const double cs = 0.5145, sn = 0.8575;
	double X[10], Y[10], V[10];
	static int flag[10] = { 1,1,1,1,1,1,1,1,1,1 };
	double center[10][2], destination[10][4], theta[10];
	double l = 35;


	for (int ii = 1; ii < 8; ii++)
	{
		X[ii] = frame.robots_blue(ii).x() / 10;
		Y[ii] = frame.robots_blue(ii).y() / 10;
		theta[ii] = -frame.robots_blue(ii).orientation();
		center[ii][0] = (-250.0 * (8.0 - ii) / 8.0) + (ii / 8.0) * 250.0;
		center[ii][1] = (150.0 * (8.0 - ii) / 8.0) - (ii / 8.0) * 150.0;
		//		cout << "     " << center[ii][0] << "       " << center[ii][1] << endl;
		destination[ii][0] = center[ii][0] + l * cs*min(ii, 8 - ii);
		destination[ii][1] = center[ii][1] + l * sn *min(ii, 8 - ii);
		destination[ii][2] = center[ii][0] - l * cs *min(ii, 8 - ii);
		destination[ii][3] = center[ii][1] - l * sn *min(ii, 8 - ii);

	}
	for (int ii = 1; ii < 8; ii++) {
		Robot_Command* rc;
		rc = rcs.add_command();
		rc->set_robot_id(ii);
		rc->set_velocity_r(0);
		rc->set_kick(false);
		rc->set_power(100);
		rc->set_dribbler_spin(1);
		if (flag[ii] == 1 && has_arrive(X[ii], -Y[ii], destination[ii][0], -destination[ii][1], 5) == 1) {
			flag[ii] = 0;
		}
		if (flag[ii] == 0 && has_arrive(X[ii], -Y[ii], destination[ii][2], -destination[ii][3], 5) == 1) {
			flag[ii] = 1;
		}
		if (flag[ii] == 1) {
			double angle = compute_heading(X[ii], -Y[ii], destination[ii][0], -destination[ii][1], theta[ii]);
			//cout << "X= " << destination[ii][0] << "Y : " << destination[ii][1]<<endl;
			//cout << "My_X=: " << X[ii] << " My_Y: " << Y[ii]<<endl;
			rc->set_velocity_x(200 * cos(angle));
			rc->set_velocity_y(200 * sin(angle));
		}
		else {
			double angle = compute_heading(X[ii], -Y[ii], destination[ii][2], -destination[ii][3], theta[ii]);
			//cout << "X= " << destination[ii][2] << "Y : " << destination[ii][3]<<endl;

			rc->set_velocity_x(200 * cos(angle));
			rc->set_velocity_y(200 * sin(angle));
		}


	}

}


void SendCmd(double v_x, double v_y)
{
	Robots_Command rcs;
	Robot_Command* rc;
	rc = rcs.add_command();
	int ret_val = 0;
	rc->set_robot_id(ME);
	rc->set_velocity_r(0);
	rc->set_velocity_x(v_x);
	rc->set_velocity_y(v_y);
	rc->set_kick(false);
	rc->set_power(100);
	rc->set_dribbler_spin(1);
	//	int sizef = rcs.command_size();
	int buffer_sizef = 1024;
	//second_debug(rcs);
	rcs.SerializeToArray(bufferf, buffer_sizef);
	//rcs.cl
	rcs.Clear();
}



double* Tangential_Velocity(double x, double y, double obstacle_x, double obstacle_y, double destination_x, double destination_y, double orientation, double VELOCITY) {
	double theta = atan((x - obstacle_x) / (-obstacle_y + y));
	const double l = 40;
	double V[2];
	double virtual_point[2][2] = { {x + l * cos(theta),-y + l * sin(theta)},{x - l * cos(theta),-y - l * sin(theta)} };
	bool flag;
	double k = (-obstacle_y + y) / (obstacle_x - x);
	double z0 = -destination_y - k * (destination_x - x) + y;
	double z1 = virtual_point[0][1] - k * (virtual_point[0][0] - x) + y;
	double z2 = virtual_point[1][1] - k * (virtual_point[1][0] - x) + y;
	if (z1 * z0 >= 0)
		flag = 1;
	else
		flag = 0;
	double heading;
	if (flag == 1)
	{
		heading = compute_heading(x, y, virtual_point[0][0], -virtual_point[0][1], orientation);
	}
	else {
		heading = compute_heading(x, y, virtual_point[1][0], -virtual_point[1][1], orientation);

	}
	double distance = sqrt(pow(x - obstacle_x, 2) + pow(y - obstacle_y, 2));
	double VEL;
	if (distance < 65) {
		VEL = 200 * VELOCITY / pow(MAX(0.2, fabs(distance - 18)), 2);
	}
	else VEL = 0;
	V[0] = min(VEL, 100) * cos(heading);
	V[1] = min(VEL, 100) * sin(heading);
	return V;
}



int main() {
	srand(time(NULL));
	//SendDebug();
	WSADATA wsa_data;
	SOCKET sock_serv = INVALID_SOCKET;
	SOCKADDR_IN addr_serv, addr_clt;
	char ip_buf[IP_BUF_SIZE];
	char msg_buf[MSG_BUF_SIZE];


	WSADATA wsaData;//初始化
	sockaddr_in RecvAddrf1;//服务器地址
	sockaddr_in RecvAddrf2;
	int Port1 = 50001;//服务器监听地址
	int Port2 = 20001;
	char SendBuff[1024 * 10];//发送数据的缓冲区
	int BufLenf = 1024 * 10;//缓冲区大小



	int ret_val = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (ret_val != 0) {
		cerr << "WSAStartup() function failed with error: " << WSAGetLastError() << "\n";
		return 1;
		system("pause");
	}
	SOCKET SendSocketf;
	SendSocketf = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//设置服务器地址
	RecvAddrf2.sin_family = AF_INET;
	RecvAddrf2.sin_port = htons(Port2);
	char *ipnow = getIP();
	cout << ipnow << endl;
	RecvAddrf2.sin_addr.s_addr = inet_addr(ipnow);
	RecvAddrf1.sin_family = AF_INET;
	RecvAddrf1.sin_port = htons(Port1);
	RecvAddrf1.sin_addr.s_addr = inet_addr(ipnow);
	//向服务器发送数据报
	printf("Sending a datagram to the receiver...\n");


	//cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;
	//
	//SecureZeroMemory(&addr_serv, SOCKADDR_IN_SIZE);
	addr_serv.sin_family = AF_INET;
	addr_serv.sin_port = htons(DEFAULT_PORT);
	addr_serv.sin_addr.S_un.S_addr = ADDR_ANY;
	//
	sock_serv = socket(addr_serv.sin_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock_serv == INVALID_SOCKET) {
		cerr << "socket() function failed with error: " << WSAGetLastError() << "\n";
		WSACleanup();
		system("pause");
		return 1;
	}
	//
	ret_val = bind(sock_serv, (SOCKADDR*)& addr_serv, SOCKADDR_IN_SIZE);
	if (ret_val != 0) {
		cerr << "bind() function failed with error: " << WSAGetLastError() << "\n";
		system("pause");
		return 1;
	}
	cout << "A UDP server has started successfully..." << endl;
	//
	/*for (double distance = 65; distance > 0; distance -= 0.2)
	{
		cout << distance << " :  ";
		cout << - 330 * 300 / pow(MAX(0.2, fabs(distance- 18)), 2)<<endl;
	}*/
	int ME1 = ME;
	while (true) {
		SecureZeroMemory(msg_buf, MSG_BUF_SIZE);
		ret_val = recvfrom(sock_serv, msg_buf, MSG_BUF_SIZE, 0, (SOCKADDR*)& addr_clt, &SOCKADDR_IN_SIZE);
		if (ret_val > 0)
		{
			inet_ntop(addr_clt.sin_family, &addr_clt, ip_buf, IP_BUF_SIZE);
			cout << "message from client " << ip_buf << ": " << msg_buf << endl;

			frame.ParseFromArray(msg_buf, MSG_BUF_SIZE);// 获取required数据
			for (int i = 0; i <= 31; ++i) ROBO[i][0] = ROBO[i][1] = -1;
			IniSizeBlue = frame.robots_blue_size();
			for (int ii = 0; ii < frame.robots_blue_size(); ++ii)
			{
				int i = frame.robots_blue(ii).robot_id();
				cout << "RBid=   " << i << endl;
				if (i == ME) {
					ME1 = ii; cout << "ME1 = " << ME1 << endl;
				}
				Vision_DetectionRobot robots_blue = frame.robots_blue(ii);
				ROBO[i][1] = robots_blue.x() / 10.0 + 300;
				ROBO[i][0] = 450 - (robots_blue.y() / 10.0 + 225);
				cout << ROBO[i][0] << " " << ROBO[i][1] << endl;
				//cout << /*robots_blue.raw_vel_x() << robots_blue.raw_vel_y() << */robots_blue.orientation() << endl;
			}
			for (int ii = 0; ii < frame.robots_yellow_size(); ++ii)
			{
				int i = frame.robots_yellow(ii).robot_id();
				//cout << "RBid=   " << i << endl;
				Vision_DetectionRobot robots_yellow = frame.robots_yellow(ii);
				ROBO[i + 16][1] = robots_yellow.x() / 10.0 + 300;
				ROBO[i + 16][0] = 450 - (robots_yellow.y() / 10.0 + 225);
				//cout << ROBO[i][0] << " " << ROBO[i][1] << endl;
			}
			double ROBO00 = 375, ROBO01 = 550, des_x = 75, des_y = 50;
			int flagRRT = RRTstar(ROBO00, ROBO01, des_x, des_y, 2);
			cout << flagRRT << " is RRT working status" << endl;
			if (flagRRT == 0)
			{
				flagRRT = RRTstar(ROBO00, ROBO01, des_x, des_y, 0);
				cout << flagRRT << " is RRT working status" << endl;
			}
			SendDebug();
			cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;

			//for(double dis=46;dis>9;dis-=0.2)
			//cout<<dis<<" dis :  "<< MAX(-1500, -5 * 400 * 100 / pow(MAX(0.2, fabs(dis - 25)), 2))<<endl;

			int id_num = NumPoint - 1, flag = 0, flag_replan = 0;
			double VELOCITY = 200;
			//bool is_accerate = 0;
			int cnt = 0;
			while (1) {
				//VELOCITY = 300;
				SecureZeroMemory(msg_buf, MSG_BUF_SIZE);
				ret_val = recvfrom(sock_serv, msg_buf, MSG_BUF_SIZE, 0, (SOCKADDR*)& addr_clt, &SOCKADDR_IN_SIZE);
				frame.ParseFromArray(msg_buf, MSG_BUF_SIZE);// 获取required数据
				//cout << frame.robots_blue_size() << "  " << ME1 + 2 << endl;
				if (frame.robots_blue_size() < (ME1 + 1)) continue;
				Vision_DetectionRobot my_robot = frame.robots_blue(ME1);
				//double X = my_robot.x() / 10;
				//double Y = -my_robot.y() / 10;
				double theta = -my_robot.orientation();
				//double now_v_x = my_robot.vel_x();
				//double now_v_y = my_robot.vel_y();
				getBarrier();
				double X = barrier_x[ME], Y = barrier_y[ME];
				//cout << "SIZE: " << frame.robots_yellow_size() << endl;

				double sum_v_x = 0;
				double sum_v_y = 0;

				if (id_num > 1 && Measure_Dis(rrt_point[id_num][1] - 300, rrt_point[id_num][0] - 225, X, Y) > Measure_Dis(rrt_point[id_num - 1][1] - 300, rrt_point[id_num - 1][0] - 225, X, Y))
					--id_num;
				//cout << rrt_point[id_num][1] - 300 << " " << rrt_point[id_num][0] - 225 << " " << X << " " << Y << endl;
				//cout << "ORt  " << theta << endl;
				if (id_num == 1) dis_lev = 2; else dis_lev = 30.0;
				if (has_arrive(rrt_point[id_num][1] - 300, rrt_point[id_num][0] - 225, X, Y, dis_lev))
				{
					if (id_num == 1 && Count >= 15) {
						Count = 0;
						id_num--;
					}
					else if (id_num == 1 && Count < 15) {
						Count++;
					}
					if (id_num == 1 || id_num == 0)id_num++;

					id_num--;
					if (id_num == 0)
					{
						getROBO();
						flag ^= 1;	//1--num, 终点返回起点
						++Score;
						cout << "Score:  " << Score << endl;
						if (flag)
						{
							flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], ROBO00, ROBO01, 2.5);
							if (flagRRT == 0)
							{
								flagRRT = RRTstar(ROBO00, ROBO01, des_x, des_y, 0);
								cout << flagRRT << " is RRT working status" << endl;
							}
							
						}
						else
						{
							flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], des_x, des_y, 2.5);
							if (flagRRT == 0)
							{
								flagRRT = RRTstar(ROBO00, ROBO01, des_x, des_y, 0);
								cout << flagRRT << " is RRT working status" << endl;
							}
						}
						id_num = NumPoint - 1;
						printf("%d is reverse RRT working status\n", flagRRT);

						SendDebug();
						cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;
					}
				}
				/*if (flag_replan)
				{
					if (flag == 1 && id_num == 1)
					{
						flag_replan = 0;
						flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], ROBO00, ROBO01, 0);
						cout<<"id_num: "<<id_num;
					}
					//if (flag) flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], ROBO00, ROBO01, 0);
					//else flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], des_x, des_y, 0);
					printf("%d is RRT working status\n", flagRRT);
					//id_num = NumPoint - 1;


				}*/
				while (1)
				{
					int Flag = 0;
					if (sqrt(pow(rrt_point[id_num][1] - 300 - X, 2) + pow(rrt_point[id_num][0] - 225 - Y, 2)) < 60) {
						for (int i = 0; i <= 31; i++)
							if ((barrier_x[i] != -1 || barrier_y[i] != -1) && i != ME)
							{
								if (sqrt(pow(rrt_point[id_num][1] - 300 - barrier_x[i], 2) + pow(rrt_point[id_num][0] - 225 - barrier_y[i], 2)) < 30)
									Flag = 1;
							}
					}
					//if ((id_num == 1 || id_num == NumPoint) && Flag == 1)id_num = id_num - flag * 2 + 1;
					if (id_num == 1) Flag = 0;
					if (Flag == 1) id_num = id_num - 1;
					if (Flag == 0) break;
				}

				//double now_velocity = sqrt(pow(now_v_x, 2)+ pow(now_v_y, 2));



				double destination_distance = sqrt(pow(rrt_point[id_num][1] - 300 - X, 2) + pow(rrt_point[id_num][0] - 225 - Y, 2));
				//double distance1= sqrt(pow(rrt_point[id_num][1] - 300 - X, 2) + pow(rrt_point[id_num][0] - 225 - Y, 2));

				VELOCITY = compute_velocity(destination_distance, X, Y, id_num, flag);
				double* Vel;
				for (int i = 0; i <= 31; i++)
				{
					if ((barrier_x[i] != -1 || barrier_y[i] != -1) && i != ME)
					{
						barrier_angle[i] = compute_heading(X, Y, barrier_x[i], barrier_y[i], theta);
						distance[i] = sqrt(pow(X - barrier_x[i], 2) + pow(Y - barrier_y[i], 2));
						if (distance[i] < 55) {
							//barrier_V[i] = MAX( -400,-50 * VELOCITY / pow(MAX(0.2, fabs(distance[i] - 25)), 2) - 50 * VELOCITY / pow(MAX(0.2, fabs(distance[i] - 22)), 2) - 80 * VELOCITY / pow(MAX(fabs(distance[i] - 22), 0.2), 2) - 100 * VELOCITY / pow(MAX(0.2, fabs(distance[i] - 18)), 2) );
							barrier_V[i] = -400 * VELOCITY / pow(MAX(0.2, fabs(distance[i] - 18)), 2);
						}
						else barrier_V[i] = 0;
						Vel = Tangential_Velocity(X, Y, barrier_x[i], barrier_y[i], rrt_point[id_num][1] - 300, rrt_point[id_num][0] - 225, theta, VELOCITY);
						sum_v_x += barrier_V[i] * cos(barrier_angle[i]) + Vel[0];
						sum_v_y += barrier_V[i] * sin(barrier_angle[i]) + Vel[1];


						//sum_v_x += barrier_V[i] * cos(barrier_angle[i]);
						//sum_v_y += barrier_V[i] * sin(barrier_angle[i]);
					}
				}
				sum_v_x = MAX(-600, sum_v_x); sum_v_y = MAX(-600, sum_v_y);
				sum_v_x = min(600, sum_v_x); sum_v_y = min(600, sum_v_y);
				//board_angle[0] = -PI / 2 - theta;
				board_angle[0] = compute_heading(X, Y, X, 225, theta);
				board_dis[0] = sqrt(pow(X - X, 2) + pow(Y - 225, 2));
				board_angle[1] = compute_heading(X, Y, X, -225, theta);
				//board_angle[1] = PI / 2 - theta;
				board_dis[1] = sqrt(pow(X - X, 2) + pow(Y + 225, 2));
				board_angle[2] = compute_heading(X, Y, 300, Y, theta);
				board_dis[2] = sqrt(pow(X - 300, 2) + pow(Y - Y, 2));
				board_angle[3] = compute_heading(X, Y, -300, Y, theta);
				board_dis[3] = sqrt(pow(X + 300, 2) + pow(Y - Y, 2));
				for (int i = 0; i <= 3; ++i) if (board_dis[i] < 40) { sum_v_x = 0; sum_v_y = 0; }
				for (int i = 0; i < 2; ++i)
					if (board_dis[i] < 60)
					{
						board_V[i] = MAX(-1500, -5 * 550 * MAX(30,VELOCITY) / pow(MAX(0.2, fabs(board_dis[i] - 25)), 2));
						if (board_dis[i] < 30) board_V[i] = -1000;
						if (board_dis[i] < 15) board_V[i] = -1500;
						if (Measure_Dis(rrt_point[1][1] - 300, rrt_point[1][0] - 225, X, Y) < 25) board_V[i] = 0;
						sum_v_x += board_V[i] * cos(board_angle[i]);
						sum_v_y += board_V[i] * sin(board_angle[i]);
					}
				for (int i = 2; i <= 3; i++)
				{
					double Keep = 48;
					if (fabs(Y) < 145) Keep = 56;
					if (board_dis[i] < Keep) 
					{
						//board_V[i] = -50 * VELOCITY / pow(MAX(0.2, fabs(board_dis[i] - 25)), 2) - 50 * VELOCITY / pow(MAX(0.2, fabs(board_dis[i] - 22)), 2) - 80 * VELOCITY / pow(MAX(fabs(board_dis[i] - 22), 0.2), 2) - 100 * VELOCITY / pow(MAX(0.2, fabs(board_dis[i] - 18)), 2);
						//board_V[i] = MAX(-1500,-5 * 400 * VELOCITY / pow(MAX(0.2, fabs(board_dis[i] - 25)), 2));
						board_V[i] = MAX(-1500, -5 * 500 * MAX(30,VELOCITY) / pow(MAX(0.2, fabs(board_dis[i] - 25)), 2));
						if (board_dis[i] < 30) board_V[i] = -1000;
						if (board_dis[i] < 15) board_V[i] = -1500;
						if (Measure_Dis(rrt_point[1][1] - 300, rrt_point[1][0] - 225, X, Y) < 25) board_V[i] = 0;
					}
					else
						board_V[i] = 0;
					sum_v_x += board_V[i] * cos(board_angle[i]);
					sum_v_y += board_V[i] * sin(board_angle[i]);
				}



				double heading = compute_heading(X, Y, rrt_point[id_num][1] - 300, rrt_point[id_num][0] - 225, theta);
				double v_x = VELOCITY * cos(heading) + sum_v_x;
				double v_y = VELOCITY * sin(heading) + sum_v_y;
				/*if (v_x <= 50 && v_y <= 50) {
					v_x = v_x + rand() % 300 - 150;
					v_y = v_y + rand() % 300 - 150;
				}*/
				SendCmd(v_x, v_y);
				//cout << v_x << " " << v_y << " " << heading << endl;
				sendto(SendSocketf, bufferf, BufLenf, 0, (SOCKADDR*)& RecvAddrf1, sizeof(RecvAddrf1));
				//Sleep(3);
				++cnt;
				if (sqrt(v_x * v_x + v_y * v_y) < 60.0 && cnt > 40 && id_num >= 2 && Measure_Dis(rrt_point[1][1] - 300, rrt_point[1][0] - 225, X, Y) > 80 && Measure_Dis(rrt_point[NumPoint][1] - 300, rrt_point[NumPoint][0] - 225, X, Y) > 80)
				{
					cnt = 0;
					flagRRT = 0;
					cout << "Dir " << flag << endl;
					getROBO();
					flag_replan = 1;
					if (flag)
					{
						flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], ROBO00, ROBO01,1);
						id_num = NumPoint;
					}
					else
					{
						flagRRT = RRTstar(ROBO[ME][0], ROBO[ME][1], des_x, des_y, 1);
						id_num = NumPoint;
					}
					printf("%d is RRT working status\n", flagRRT);

					SendDebug();
					cout << sendto(SendSocketf, msg, BufLenf, 0, (SOCKADDR*)& RecvAddrf2, sizeof(RecvAddrf2)) << endl;
				}
			}

			Sleep(5000000);
		}
	}
	ret_val = shutdown(sock_serv, SD_BOTH);
	closesocket(sock_serv);
	WSACleanup();
	cout << "server shutdown..." << endl;
	system("pause");
	return 0;
}
