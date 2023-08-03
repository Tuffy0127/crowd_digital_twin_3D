#include<math.h>
#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include<vector>
#include<iostream>
#include<list>
#include<queue>

using namespace std;

// ��������
#define MAX_AGENT_NUM 1000 // ���agent��������Ϊʹ�õ���vector����ʵ����û�õ�
#define MAX_OBLINE_NUM 1000 // ����ϰ�������

// ��ģ����
#define MAX_V 1.02 // agent ����ٶ�
#define sense_range 5 // ��֪��Χ��m
#define density 320 // ����/�뾶��m/density = r 320
#define tao 0.5 // Ŀǰ����ת�Ƶ�Ŀ�귽��ļ��ٶȲ���

#define total_level 3


// sfm����
#define k1 120000 // body force parameter
#define k2 240000 // tangential force parameter ����������s
#define A 2000 // N// A * exp[dis/B] only social force
#define B 0.08 // m

//������
int col_num[total_level] = {};
int row_num[total_level] = {};//init in init_map

// ����ͼ�Ŵ��
double  map_factor = 10;

// ��������
vector<vector<vector<int>>> map_matrix; // ����ͼ
vector<vector<vector<int>>> density_map; // ��Ⱥ�ܶ�ͼ,��A*����ѡȡ���ٵ�·��



struct QUEUE;

// �ṹ��
struct cordinate {
	double x;
	double y;
	int level;
	cordinate() = default;
	cordinate(double a, double b,int l) 
	{
		x = a;
		y = b;
		level = l;
	}
}; // ����ṹ��
struct AGENT
{
	int id;

	double m; // mass
	double x; // pos
	double y;
	int level = 0;
	double vx; // v
	double vy;
	double gx; // level goal pos
	double gy;
	double fgx; // final goal pos
	double fgy;
	int goal_level = 0;
	double next_gx; // next goal pos
	double next_gy;
	double v0; // max velocity
	double tao_1 = 0.1;

	double dis; // ��㵽�յ����
	int jam_time = 0; // ��������ٶȹ�Сʱ���ӣ��ﵽ�趨����ʱ�ж�Ϊ��ס������Ѱ·
	int np = 0; // no path flag
	int color = 0;
	double arrive_time = 0;

	bool arrived = 0;
	double arrive_range = 0;

	// queue
	QUEUE* Q;
	bool go_queue = 0;
	int order = 0;
	bool in_queue = 0;
	double process_time = 0;
	double cant_process_time = 0;


	
	


	list<cordinate> path; // A* ·���洢
}; // agent �ṹ��
struct OBLINE
{
	double sx; // start point
	double sy;
	double ex; // end point
	double ey;
	double len; // length

}; // �ϰ���ṹ��



// ��������
double randval(double, double);
double agent_dis(AGENT, AGENT); // agents ֮�����
void agent_force(AGENT*, AGENT*, double*, double*, double); // agents ֮��������
double point_to_line_dis(double, double, double, double, double, double, double, double*, double*); // ����㵽�߾���
void obline_force(AGENT*, OBLINE*, double*, double*); // �����ϰ���������



// ����ʵ��
double randval(double a, double b)
{
	return a + (b - a) * rand() / (double)RAND_MAX;
}


double agent_dis(AGENT* a1, AGENT* a2)
{
	return sqrt((a1->x - a2->x) * (a1->x - a2->x) + (a1->y - a2->y) * (a1->y - a2->y));
}


void agent_force(AGENT* a1, AGENT* a2, double* fx, double* fy, double dis) // a1<-a2
{
	// ��֪�����жϷ��ں�������֮ǰ��
	if (dis == 0)
	{
		dis = 1e-10;//��ֹ����dis=0
	}
	double dx = (a1->x - a2->x) / dis; // a2��a1ʩ�ӵ����ĵ�λ����������x
	double dy = (a1->y - a2->y) / dis; // a2��a1ʩ�ӵ����ĵ�λ����������y

	//�Գ��ж�
	double cos1 = (a2->vx * dx + a2->vy * dy) / sqrt(dx * dx + dy * dy) / sqrt(a2->vx * a2->vx + a2->vy * a2->vy);  // a2�ٶȺ�a2��a1����������н�cos
	double sin = (a2->vx * dy - a2->vy * dx) / sqrt(dx * dx + dy * dy) / sqrt(a2->vx * a2->vx + a2->vy * a2->vy); // a2�ٶȺ�a2��a1����������н�sin
	double cos2 = (-a1->vx * dx - a1->vy * dy) / sqrt(dx * dx + dy * dy) / sqrt(a1->vx * a1->vx + a1->vy * a1->vy); // a1�ٶȵķ������a2��a1����������н�

	if (cos1 >= 0.866 && cos2 >= 0.866)//60`
	{
		if (sin >= 0)
		{
			dx = dx + dy;
			dy = dy - dx;

		}
		else
		{
			dx = dx - dy;
			dy = dy + dx;

		}
	}

	double rij = a1->m / density + a2->m / density; // ����agent�뾶��
	double delta_d = rij - dis; // �뾶�ͼ�ȥ���ĵ����
	double rif = A * exp(delta_d / B); // repulsive interaction force ����
	double bf = delta_d < 0 ? 0 : k1 * delta_d; // body force agents�Ӵ�ʱ���ڵ���
	// ������body force�ĺ���
	double res_f_x = (rif + bf) * dx;
	double res_f_y = (rif + bf) * dy;

	// ������ agents�Ӵ�ʱ����
	double tfx = 0;
	double tfy = 0;
	if (delta_d > 0) // ����뾶�нӴ�ʱ
	{
		double tix = -dy;
		double tiy = dx;
		double delta_v = (a2->vx - a1->vx) * tix + (a2->vy - a1->vy) * tiy; // �����ٶȲ�
		tfx = k2 * delta_d * delta_v * tix;
		tfy = k2 * delta_d * delta_v * tiy;
	}
	*fx += res_f_x + tfx;
	*fy += res_f_y + tfy;

	return;

}


double point_to_line_dis(double ax, double ay, double sx, double sy, double ex, double ey, double d, double* px, double* py)//pΪ����
{
	// �ú�����Ҫ���㴹�㣬�����ϰ����agent����������ļ���
	double dis = 0;
	double dot_pro = ((ax - sx) * (ex - sx) + (ay - sy) * (ey - sy)) / (d * d); // dot product of s_a and s_e divide by d2(s->start point of line;e->end point of line;a->agent)

	if (dot_pro <= 0) // ������start point���
	{
		dis = sqrt((ax - sx) * (ax - sx) + (ay - sy) * (ay - sy));
		*px = sx;
		*py = sy;

	}
	else if (dot_pro < 1) // �������߶���
	{
		*px = sx + (ex - sx) * dot_pro;
		*py = sy + (ey - sy) * dot_pro;
		dis = sqrt((ax - *px) * (ax - *px) + (ay - *py) * (ay - *py));

	}
	else if (dot_pro >= 1) // ������end point���
	{
		dis = sqrt((ax - ex) * (ax - ex) + (ay - ey) * (ay - ey));
		*px = ex;
		*py = ey;

	}
	return dis;

}


void obline_force(AGENT* a, OBLINE* l, double* fx, double* fy)
{
	double px, py;
	double dis = point_to_line_dis(a->x, a->y, l->sx, l->sy, l->ex, l->ey, l->len, &px, &py);
	if (dis <= 0)
	{
		dis = 1e-10;//��ֹ����dis=0
	}
	if (dis > sense_range) // ��֪�����ж��ں�����
	{
		return;
	}
	double riw = a->m / density - dis; // �뾶-����
	double rif = A * exp(riw / B);
	double bf = riw < 0 ? 0 : k1 * riw;
	//if (bf != 0)cout << endl;
	// ��������������ķ�������
	double nx = (a->x - px) / dis;
	double ny = (a->y - py) / dis;
	// ������body force����
	double res_f_x = (rif + bf) * nx;
	double res_f_y = (rif + bf) * ny;
	// ������
	double tfx = 0;
	double tfy = 0;
	if (riw > 0)
	{
		double tix = -ny;
		double tiy = nx;
		double delta_v = a->vx * tix + a->vy * tiy;//�����ٶȲ�
		tfx = k2 * riw * delta_v * tix;
		tfy = k2 * riw * delta_v * tiy;
	}

	*fx += res_f_x - tfx;
	*fy += res_f_y - tfy;


	return;

}


//---------------------------------------------����ΪSFM����------------------------------------------------

//---------------------------------------------����ΪA�ﲿ��------------------------------------------------

//��������
void A_star(AGENT*);
bool in_map(int, int, int);

// �ṹ��
struct node
{
	int x;
	int y;
	int g; // �ƶ�����
	int h; // ����ɱ�
	bool flag; // ������flag����ʾnode������close_list
	bool check;



	node* parent; // ���ڵ�,���յ�����ָ�����

	node() = default;

	node(int a, int b, double h = 0) {
		flag = 0;
		check = 0;
		x = a;
		y = b;
		h = h;
	}

	/*static bool compare(node& a, node& b)
	{
		return (a.g + a.h) > (b.g + b.h);
	}*/

};

struct dir
{
	int x;
	int y;
	dir(int a, int b)
	{
		x = a;
		y = b;
	}
};

// ����
vector<vector<vector<node>>> map_matrix_A;

// �ɵ�����
const int a_step = 5;
const int detect_step = 3;
vector<dir> direction = { {a_step,0}, {-a_step,0}, {0,a_step}, {0,-a_step} }; // ����
vector<dir> ob_direction = { {a_step,a_step},{a_step,-a_step},{-a_step,-a_step},{-a_step,a_step} }; // б��
vector<dir> wall_detect = { {detect_step,0}, {-detect_step,0}, {0,detect_step}, {0,-detect_step},{detect_step,detect_step},{detect_step,-detect_step},{-detect_step,-detect_step},{-detect_step,detect_step} };
const int path_len = 1;
const int max_time = 800; // ms


//����ʵ��
void A_star(AGENT* a)
{
	list<node*> open_list;

	//��ʼ��close_list �� check
	for (auto& y : map_matrix_A[a->level])
	{
		for (auto& x : y)
		{
			x.flag = 0;
			x.check = 0;
		}
	}

	// ��ʼ����� Ŀǰ�����ģ����ȷ Ӧ���ǲ�����ֱ�������ͼ����� �����ⲿ�ֿ���ע��
	/*if (!in_map(int(a->x * map_factor), int(a->y * map_factor)))
	{
		a->path.clear();
		a->gx = a->x;
		a->gy = a->y;
		a->next_gx = a->x;
		a->next_gy = a->y;
		cout << "***********************************************no path" << endl;
		a->np = 1;
		return;
	}*/
	open_list.push_back(&map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)]);
	map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)].x = int(a->x * map_factor);
	map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)].y = int(a->y * map_factor);
	map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)].flag = 1;
	map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)].h = (abs(int(a->gx * map_factor) - int(a->x * map_factor)) + abs(int(a->gy * map_factor) - int(a->y * map_factor))) * 10;
	map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)].g = 0;
	map_matrix_A[a->level][int(a->y * map_factor)][int(a->x * map_factor)].parent = nullptr;

	clock_t start, end; // �Ӹ�ʱ��ͳ��,�����ж���·�������
	start = clock();

	node* temp = open_list.front(); // Ŀǰ�����ĵ�


	while (!(abs(temp->x - a->gx * map_factor) <= a_step && abs(temp->y - a->gy * map_factor) <= a_step))
	{
		double f = INT_MAX;
		//��F��С�Ľڵ�
		for (auto* a : open_list)
		{
			if ((a->g + a->h) < f)
			{
				f = (a->g + a->h);
				temp = a;
			}
		}

		bool flag = 0;
		// �ж���Χ�Ƿ���ǽ
		/*for (auto d : direction)
		{
			if (in_map(temp->x + d.x, temp->y + d.y,a->level) && map_matrix[a->level][temp->y + d.y][temp->x + d.x] == 0)
			{
				flag = 1;
				break;
			}
		}
		for (auto d : ob_direction)
		{
			if (in_map(temp->x + d.x, temp->y + d.y,a->level) && map_matrix[a->level][temp->y + d.y][temp->x + d.x] == 0)
			{
				flag = 1;
				break;
			}
		}*/

		// ������������,��ʱ�ƶ�����Ϊ10
		for (auto d : direction)
		{

			bool wall = 0;
			if (in_map(temp->x + d.x, temp->y + d.y,a->level))
			{
				for (double i = 1; i <= a_step; ++i)
				{
					// cout << d.y * (i / a_step) <<" " << d.x * (i / a_step) << endl;
					if (map_matrix[a->level][temp->y + d.y * (i / a_step)][temp->x + d.x * (i / a_step)] == 0)
					{
						wall = 1;
						flag = 1;
						break;
					}
					
				}
				if (wall)
				{
					continue;
				}
				for (auto dd : wall_detect)
				{
					if (in_map(temp->x + d.x + dd.x, temp->y + d.y + dd.y, a->level))
					{
						if (map_matrix[a->level][temp->y + d.y + dd.y][temp->x + d.x + dd.x] == 0)
						{
							wall = 1;
						}
					}
				}
				if (wall)
				{
					continue;
				}
				int point_type = map_matrix[a->level][temp->y + d.y][temp->x + d.x];
				//if (density_map[temp->y + d.y][temp->x + d.x])point_type == 0;
				//cout << density_map[temp->y + d.y][temp->x + d.x] << endl;
				if (point_type != 0 && map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].check == 0)//�ڵ��ڵ�ͼ�ڣ��Ҳ����ϰ��ﲿ��,��δcheck��
				{
					if (map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].flag == 0)//��ʾ�õ��һ�ν���open_list
					{
						map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
						map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
						map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
						map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step * (point_type * 0.75 + density_map[a->level][temp->y + d.y][temp->x + d.x]);
						map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].parent = temp;
						map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].flag = 1;
					}
					else // �ٴ�ɸ��һ����d,ֻ����ͨ��(��������ĵ�temp����d�Ĵ���)��(d�ĸ��ڵ㵽d)��Сʱ,�Ż����
					{
						if (temp->g + 10 * a_step < map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].g)
						{
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].g = temp->g + 10 * a_step * (point_type * 0.75 + density_map[a->level][temp->y + d.y][temp->x + d.x]);
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].parent = temp;
						}
					}
					open_list.push_back(&map_matrix_A[a->level][temp->y + d.y][temp->x + d.x]);
				}
			}

		}

		// б����������,��ʱ�ƶ�����Ϊ14,ʵ����Ӧ����10*2^(1/2),С�����ֵ�ᵼ�½����ƫ��б���ƶ�
		if (!flag)
		{
			for (auto d : ob_direction)
			{
				if (in_map(temp->x + d.x, temp->y + d.y,a->level))
				{
					bool wall = 0;
					for (auto dd : wall_detect)
					{
						if (in_map(temp->x + d.x + dd.x, temp->y + d.y + dd.y, a->level))
						{
							if (map_matrix[a->level][temp->y + d.y + dd.y][temp->x + d.x + dd.x] == 0)
							{
								wall = 1;
							}
						}
					}
					if (wall)
					{
						continue;
					}
					int point_type = map_matrix[a->level][temp->y + d.y][temp->x + d.x];
					//if (density_map[temp->y + d.y][temp->x + d.x])point_type == 0;
					if (point_type != 0 && map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].check == 0)//�ڵ��ڵ�ͼ�ڣ��Ҳ����ϰ��ﲿ��,��δcheck�� && map_matrix_A[temp->y + d.y][temp->x + d.x].check == 0
					{
						if (map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].flag == 0) // ��ʾ�õ��һ�ν���open_list
						{
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].x = temp->x + d.x;
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].y = temp->y + d.y;
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].h = (abs(int(a->gx * map_factor) - (temp->x + d.x)) + abs(int(a->gy * map_factor) - (temp->y + d.y))) * 10;
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type * 0.75 + density_map[a->level][temp->y + d.y][temp->x + d.x]);
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].parent = temp;
							map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].flag = 1;
						}
						else // �ٴ�ɸ��һ����d,ֻ����ͨ��(��������ĵ�temp����d�Ĵ���)��(d�ĸ��ڵ㵽d)��Сʱ,�Ż����
						{
							if (temp->g + 14 * a_step < map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].g)
							{
								map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].g = temp->g + 14 * a_step * (point_type * 0.75 + density_map[a->level][temp->y + d.y][temp->x + d.x]);
								map_matrix_A[a->level][temp->y + d.y][temp->x + d.x].parent = temp;
							}
						}
						open_list.push_back(&map_matrix_A[a->level][temp->y + d.y][temp->x + d.x]);
					}
				}
			}
		}

		open_list.remove(temp);
		temp->check = 1; // �Ѿ����������

		// ��·���ж�,���󲿷���n s֮�ڿ����ѵ�,������Ϊ������·��
		end = clock();
		if (end - start > max_time)
		{
			a->path.clear();
			a->gx = a->x;
			a->gy = a->y;
			a->next_gx = a->x;
			a->next_gy = a->y;
			a->np = 1;
			return;
		}

	}
	// push ·���� path,�������ѡ��·���ܶ�
	int counter = 0;
	do
	{
		counter++;
		if (counter == path_len) // ÿn��ȡһ����
		{
			a->path.push_front(cordinate(temp->x, temp->y,a->level));
			counter = 0;
		}
		else if (map_matrix[a->level][temp->y][temp->x] == 2) // �ſ���Ҫ�ڵ�ҲҪpush
		{
			a->path.push_front(cordinate(temp->x, temp->y,a->level));
		}
		if (temp->parent == nullptr)break;
		temp = temp->parent;

	} while (temp != nullptr);
	if (!a->path.empty())
	{
		a->next_gx = a->path.front().x / map_factor;
		a->next_gy = a->path.front().y / map_factor;
	}
	else
	{
		a->next_gx = a->gx;
		a->next_gy = a->gy;
	}

}


bool in_map(int x, int y,int level)
{
	return (x >= 0 && x < col_num[level] - 1) && (y >= 0 && y < row_num[level] - 1);
}


//---------------------------------------------����ΪA�ﲿ��------------------------------------------------

//---------------------------------------------����Ϊ3D����-------------------------------------------------

// �ṹ��
struct stair
{
	int id;
	double up_x;
	double up_y;
	double down_x;
	double down_y;
	stair(int i,double ux, double uy, double dx, double dy)
	{
		id = i;
		up_x = ux;
		up_y = uy;
		down_x = dx;
		down_y = dy;
	}
};
stair s1(0,82,7.6,82,11);


// ��������
void update_g(AGENT*);


// ����ʵ��

// ����¥��Ŀ����ж�
void update_g(AGENT* a)
{	// Ŀǰ�����
	
	if (a->level == a->goal_level)
	{   // �Ѿ���ͬ���˾�����
		a->gx = a->fgx;
		a->gy = a->fgy;
		a->path.clear();
		A_star(a);
	}
	else if(a->level < a->goal_level)
	{
		// ����gx����һ�����¥��,Ŀǰֻд¥��,ֻдһ��¥��
		//cout << "here" << endl;
		a->gx = s1.up_x;
		a->gy = s1.up_y;
		a->path.clear();
		A_star(a);
		
	}
	else if (a->level > a->goal_level)
	{
		// ����gx����һ�����¥��
		a->gx = s1.down_x;
		a->gy = s1.down_y;
		a->path.clear();
		A_star(a);
		
	}

}

// ��¥��
void use_stair(AGENT* a)
{


}

// �ߵ���
void use_lift(AGENT* a)
{

}

//---------------------------------------------����Ϊ3D����-------------------------------------------------

//---------------------------------------------����Ϊ�ŶӲ���-------------------------------------------------


// �յ�
struct cordinate goal[6] = { {80,57.5,1} , { 54,18,0 }, { 13.5,55.5,0 }, { 50,41,1 }, { 80,18.4,2 }, {50,34.7,2} };


struct QUEUE
{
	int id;
	int a_num = 0;
	double x;
	double y;
	int level;
	//bool symptom = 1;
	list<AGENT*> out_list;
	// list<AGENT*> in_list;
	vector<cordinate> path;

	int point_num = 0;
	QUEUE() = default;
	QUEUE(int id1,double x1, double y1,int level1)
	{
		id = id1;
		x = x1;
		y = y1;
		level = level1;
		a_num = 0;
		path.push_back(cordinate(x, y , level));
	}
	
	void queue_back();

};

void QUEUE::queue_back()
{
	if (a_num < 10)
	{
		path.push_back(cordinate(x, y - (a_num / map_factor) * 10, level));
	}
	else
	{
		path.push_back(cordinate(x, y - (10 / map_factor) * 10 - (a_num - 10) / map_factor * 0.01, level));
	}

}

// ���д��� ��init_map��
// 53.4 71.5

vector<QUEUE*> q_list = {};


// ��������
void go_queue(AGENT*,QUEUE*);
void in_queue(AGENT*);
void out_queue(AGENT*);
void cant_process(AGENT*);

// ����
double registration_time = 10;


// ����ʵ��
// agentѡ��һ�����м���
void go_queue(AGENT* a,QUEUE* q)
{
	a->go_queue = true;
	a->order = 0;
	a->Q = q;
	a->fgx = a->Q->x;
	a->fgy = a->Q->y;
	a->goal_level = a->Q->level;
	q->out_list.push_back(a);
}

// ��go_agent״̬��agent�������и����ж�Ϊarriveʱ����ʼ�����Ŷӽ���
void in_queue(AGENT* a)
{
	a->arrived = false;
	a->go_queue = false;
	a->in_queue = true;
	a->Q->a_num += 1;
	a->order = a->Q->a_num;
	a->path.clear();
	if (a->order > a->Q->point_num)
	{
		a->Q->point_num = a->order;
		a->Q->queue_back();
	}

	a->next_gx = a->Q->path[a->order].x;
	a->next_gy = a->Q->path[a->order].y;

}

// �Һ���ϣ��뿪���У�ȥ�Լ���Ŀ��
void out_queue(AGENT* a)
{
	
	a->process_time = 0;
	a->Q->a_num -= 1;
	for (auto& a_q : a->Q->out_list)
	{
		if (a_q->order && a_q->in_queue)
		{
			a_q->order -= 1;
			a_q->next_gx = a->Q->path[a_q->order].x;
			a_q->next_gy = a->Q->path[a_q->order].y;
		}
	}
	a->in_queue = false;
	a->Q->out_list.remove(a);
	int rand = int(randval(0, 6));
	a->fgx = goal[rand].x;
	a->fgy = goal[rand].y;
	a->goal_level = goal[rand].level;
	update_g(a);
}

void cant_process(AGENT* a)
{
	
	a->cant_process_time = 0;
	a->process_time = 0;
	a->Q->a_num -= 1;
	for (auto& a_q : a->Q->out_list)
	{
		if (a_q->order && a_q->in_queue)
		{
			a_q->order -= 1;
			a_q->next_gx = a->Q->path[a_q->order].x;
			a_q->next_gy = a->Q->path[a_q->order].y;
		}
	}
	a->in_queue = false;
	in_queue(a);

}



//---------------------------------------------����Ϊ�ŶӲ���-------------------------------------------------

//---------------------------------------------����ΪRPD����-------------------------------------------------

struct doing
{
	double time;
};


struct RPD_model
{
	int symptom;
	int state;
	int time;

};




