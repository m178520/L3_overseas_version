#include "nav.h"
#include "gps.h"
#include "usart.h"
#include "pid.h"
#include "math.h"
#include "deal_data.h"
#include "motor.h"

static char Vehicle_To_Distance_Angle_flag = 0; //用于将起点与终点线在每次只计算一次
char wait_run_point[WAIT_RUN_POINT_NUM][2][20];

waypoints_run_status_t waypoints_run_status = {0};


PID_TypeDef PID_angle_control;

/*分割目前已获取的目标航点*/
void waypoints_Parse(char *string,char * str)
{
	char* q[20] = {0};
	char* temp = strtok(string, str);
	int k = 0;
	for(int i=0; temp != NULL; i++)
	{
//		printf("%s\r\n", temp);
		char *lon_lat = strtok(temp, " ");
		for(int j=0; lon_lat != NULL ; j++)
		{
//				printf("%s\r\n", lon_lat);
				strcpy(wait_run_point[i + (waypoints_run_status.Parse_index/10)*10][j],lon_lat);
				q[k++] = lon_lat;
				if (j >= 2)  break;
				lon_lat = strtok(NULL, " ");
		}
		waypoints_run_status.Parse_index++;
		if(waypoints_run_status.Parse_index >= 50) waypoints_run_status.Parse_index = 0;
		waypoints_run_status.Parse_num  ++;
		if (i >= 10)  break;
		temp = strtok(temp+strlen(q[k-1]) + strlen(q[k-2])+2, str); 
	}
}


/*求车辆点至目标航线垂直距离与在线的哪一侧*/
pointToline_distance_t pointToline_distance(double Vehicle_lat,double Vehicle_lon,double start_lat, double start_lon, double stop_lat, double stop_lon)
{
	pointToline_distance_t pointToline_info = {0};
	static WGS84_axis_t Endpoint_XY = {0};
	static Line_straight_param_t start_stop_line = {0};
	if(Vehicle_To_Distance_Angle_flag == 0) //本次航线初次进入计算
	{
		/*求以起点为原点，终点的坐标*/
		Endpoint_XY = GPStoXY(start_lat,start_lon,stop_lat,stop_lon);
		
		/*求出函数系数*/
		start_stop_line = line_fun(0,0,Endpoint_XY.x,Endpoint_XY.y);
		
		Vehicle_To_Distance_Angle_flag ++;
	}
	/*求以起点为原点，车辆的坐标*/
	WGS84_axis_t Vehicle_XY  = GPStoXY(Endpoint_XY.origin_lat,Endpoint_XY.origin_lon,Vehicle_lat,Vehicle_lon);
	
	#if CONTROL_WAY == 1
	pointToline_info.origin_start_stop_line_param = start_stop_line;
	pointToline_info.origin_Vehicle_XY            = Vehicle_XY;
	pointToline_info.Endpoint_XY                  = Endpoint_XY;
	
	#endif
	
	pointToline_info.VehicleToStart  = Vehicle_XY.length;
	pointToline_info.StartToTerminal = Endpoint_XY.length;
	
	/*计算点至线的距离*/              
	double denominator = sqrt(1 + start_stop_line.k * start_stop_line.k);                
	double numerator = fabs(start_stop_line.k * Vehicle_XY.x - Vehicle_XY.y + start_stop_line.b);              
	pointToline_info.pointToline = numerator / denominator;

	/*判断方向*/
	double pose_direction = (Vehicle_XY.y - Endpoint_XY.y) / (Vehicle_XY.x - Endpoint_XY.x);
	double direction_err = pose_direction - start_stop_line.k;
	
	if (direction_err < 0)      pointToline_info.direct = VEHICLE_DIRECT_RIGHT;
	else                        pointToline_info.direct = VEHICLE_DIRECT_LEFT;

	return pointToline_info;
}


/*轨迹跟踪控制*/
/*只是横向的控制*/
tracking_control_t tracking_control_Arith(PID_TypeDef *PID_InitStruct,pointToline_distance_t info)
{
	tracking_control_t tracking_control = {0};
	#if CONTROL_WAY == 0 //直接拐 在线的左边就向右拐 距离作为拐弯的大小值
		/*偏离航信过大，就打满方向*/
		if(info.pointToline > POINTTOLINE_THRES  )
		{
			tracking_control.value = 1;
		}
		/*偏离航线不大，使用点至线的距离求解个方向量*/
		else
		{
			/*将线与点之间的距离输入pid求取转向角控制量*/
			double PID_Angle = PID_Update(PID_InitStruct,info.pointToline,0);
			/*进行归一化*/
//			tracking_control.value = PID_Angle/PID_ANGLE_THRES;
			tracking_control.value = PID_Angle;
		}
		tracking_control.direct = info.direct;
	#else //画一个三角形，不断跟踪三角形的角
	double x,y;
	double target_degree;
	Line_straight_param_t target_line = {0};
		/*将车辆点至直线的垂足坐标求出*/
		/*将过车辆点的垂线求出*/
		Line_straight_param_t vertical_line_fun_param = vertical_line_fun(info.origin_Vehicle_XY.x,info.origin_Vehicle_XY.y,info.origin_start_stop_line_param.k,info.origin_start_stop_line_param.b);
		/*求出垂足*/	
		Line_inter_point_t    Line_inter_point        = Line_inter_point_math(info.origin_start_stop_line_param.k,info.origin_start_stop_line_param.b,vertical_line_fun_param.k,vertical_line_fun_param.b);
		/*将垂足至三角形边长为一定值的坐标求出*/
		if(info.Endpoint_XY.x >=0)
		{
			y = Line_inter_point.y + FIXED_LENGHT * sin(atan(info.origin_start_stop_line_param.k));
			x = Line_inter_point.x + FIXED_LENGHT * cos(atan(info.origin_start_stop_line_param.k));
		}
		else
		{
			y = Line_inter_point.y - FIXED_LENGHT * sin(atan(info.origin_start_stop_line_param.k));
			x = Line_inter_point.x - FIXED_LENGHT * cos(atan(info.origin_start_stop_line_param.k));
		}
		/*求出直线的角度*/
		target_line = line_fun(info.origin_Vehicle_XY.x,info.origin_Vehicle_XY.y,x,y);
		
		double degree = atan(target_line.k) * 180.0 / PI;
		
		if(y-info.origin_Vehicle_XY.y >= 0 && x - info.origin_Vehicle_XY.x < 0) //如果位于第二象限
		{
				target_degree = 180.0 + degree;
		}
		else if(y-info.origin_Vehicle_XY.y < 0 && x - info.origin_Vehicle_XY.x < 0) //如果位于第三象限
		{
				target_degree =  degree - 180.0 ;
		}
		else //位于一四象限
		{
			target_degree  = degree;
		}
		
		/*将x坐标轴为0度变换为将y周正方向为0度*/
		target_degree = -1.0 * target_degree +90;
		
		/* */
		double gnss_Angle = strtod(gnss.CourseAngle,NULL);
		if(gnss_Angle > 180)
		{
			gnss_Angle = gnss_Angle - 360.0;
		}
		
		double err = gnss_Angle - target_degree;
		double fabs_err = fabs(err);
		
		tracking_control.value = fabs_err ;  //归一化
		
		/*确定方向*/
		if(err > 0)          tracking_control.direct = VEHICLE_DIRECT_LEFT;
		else if(err < 0 )    tracking_control.direct = VEHICLE_DIRECT_RIGHT;
		else                 tracking_control.direct = 0;
		
		if(fabs_err > 180)  
		{
			tracking_control.direct =  -tracking_control.direct;
			tracking_control.value  =  360 - fabs_err; /*0-180*/
		}
		
	#endif
	return tracking_control;
}


/*终点判定函数*/
char  Terminal_decision(double distance)
{

	if(distance <= TERMINAL_RANGE)  return 0; //说明到达终点
	else                            return 1; //未到达终点
}

NAV_output_t NAV_Control()
{
	NAV_output_t NAV_output;
//	double process = 0;
	double Angle,Speed;
	pointToline_distance_t pointToline_info = {0};
	tracking_control_t     tracking_control = {0};
	if(waypoints_run_status.current_toindex != 0) //不是前往初始点
	{
		/*求车辆点至目标航线垂直距离与方向*/
		pointToline_info	= pointToline_distance(strtod(gnss.Lat,NULL),strtod(gnss.Lon,NULL),strtod(wait_run_point[waypoints_run_status.current_fromindex][0],NULL),strtod(wait_run_point[waypoints_run_status.current_fromindex][1],NULL),strtod(wait_run_point[waypoints_run_status.current_toindex][0],NULL),strtod(wait_run_point[waypoints_run_status.current_toindex][1],NULL));
	}
	else //前往初始点
	{
		/*求车辆点至目标航线垂直距离与方向*/
		pointToline_info	= pointToline_distance(strtod(gnss.Lat,NULL),strtod(gnss.Lon,NULL),strtod(gnss.Lat,NULL),strtod(gnss.Lon,NULL),strtod(wait_run_point[waypoints_run_status.current_toindex][0],NULL),strtod(wait_run_point[waypoints_run_status.current_toindex][1],NULL));
	}
	/*执行终点判定*/
	if(Terminal_decision(GPStoXY(strtod(gnss.Lat,NULL),strtod(gnss.Lon,NULL),strtod(wait_run_point[waypoints_run_status.current_toindex][0],NULL),strtod(wait_run_point[waypoints_run_status.current_toindex][1],NULL)).length))/*未到达终点*/
	{
		/*轨迹跟踪计算*/
		tracking_control = tracking_control_Arith(&PID_angle_control,pointToline_info);
		
		/*格式转换*/
		if(tracking_control.direct == VEHICLE_DIRECT_LEFT) Angle =  tracking_control.value/2;    //方向值0~90
		else                                               Angle = -tracking_control.value/2;
		
		/*驱动控制*/
		//Speed目前手动给值
		NAV_output.RSpeed =  60 + Angle;
		NAV_output.LSpeed =  60 - Angle;
//		/*进度计算*/
//		process = pointToline_info.VehicleToStart/ pointToline_info.StartToTerminal;
	}
	else /*到达终点*/
	{
//		/*进度计算*/
//		process = 1;
		
		if(waypoints_run_status.current_toindex > 0)
		{
			waypoints_run_status.current_toindex++;
			waypoints_run_status.current_fromindex++;
		}
		else  waypoints_run_status.current_toindex++;
		printf("方向量：%f,方向：%d\r\n",tracking_control.value,tracking_control.direct);
		Vehicle_To_Distance_Angle_flag = 0;
	}
	return NAV_output;
}


