#include"srp_3d.h"
#include<iostream>
#include<fstream>
#include<vector>
#include<sstream>
#include<string>
#include<stdio.h>
#include<omp.h>
#include<time.h>

using namespace std;

const int thread_num = 12; // OpenMP线程数

// 可调参数
int agent_num = 200;
int step_num = 20000;
double tick = 0.05;//timestep
int jam_time_threshole = 50;
double total_time = 0;

// 程序数据
int  obstical_line_num[total_level] = {};
clock_t total_start, total_end;
vector<AGENT> agent_list; // agent vector
//struct OBLINE obstical_lines[total_level][MAX_OBLINE_NUM]; // obstical line array
vector<vector<OBLINE>> obline_list;
vector<vector<double>> seq;
vector<AGENT> agent_back_list; // agent vector

// 输出文件
FILE* f = fopen("C:/Users/leesh/Desktop/srp/output_3d/output.txt", "w");
FILE* ff = fopen("C:/Users/leesh/Desktop/srp/output_3d/test.txt", "w");

// 终点
struct cordinate goal[6] = { {80,57.5,1} , { 54,18,0 }, { 13.5,55.5,0 }, { 50,41,1 }, { 80,18.4,2 }, {50,34.7,2} };





// 函数声明
void init_obline(string[],int);//障碍物读入
void init_map(string[], int);
void init_agent_seq(string);
void init_agent(int);
void init();
void output(AGENT*);//写出agent坐标
void step();
void update_density();
void test();
void push_new_agent();
bool cross(double sx1, double sy1, double ex1, double ey1, double sx2, double sy2, double ex2, double ey2);


// 函数实现
void init_obline(string obline_file[], int level)
{
	for (int k = 0; k < level; ++k)
	{
		fstream infile(obline_file[k], ios::in);
		string buf;
		int num = 0;
		getline(infile, buf);
		vector<OBLINE> temp1;
		while (!infile.eof()) 
		{
			OBLINE temp;
			stringstream ss(buf);
			ss >> temp.sy >> temp.sx >> temp.ey >> temp.ex;
			temp.sx /= map_factor;
			temp.sy /= map_factor;
			temp.ex /= map_factor;
			temp.ey /= map_factor;
			temp.len = sqrt((temp.sx - temp.ex) * (temp.sx - temp.ex) + (temp.sy - temp.ey) * (temp.sy - temp.ey));
			getline(infile, buf);
			temp1.push_back(temp);
			num++;
		}
		obline_list.push_back(temp1);
		obstical_line_num[k] = num;
		infile.close();
	}
	
}


void init_map(string map_file[], int level)
{
	for (int k = 0; k < level; ++k)
	{
		vector<vector<int>> temp1;
		fstream infile(map_file[k], ios::in);
		string buf = "";
		getline(infile, buf);
		while (!infile.eof())
		{
			vector<int> temp2;
			++row_num[k];
			stringstream ss(buf);
			int num;
			while (!ss.eof())
			{
				if (row_num[k]== 1)
				{
					++col_num[k];
				}
				ss >> num;
				temp2.push_back(num);
			}
			getline(infile, buf);
			temp1.push_back(temp2);
		}
		++col_num[k];
		map_matrix.push_back(temp1);
		infile.close();

		vector<vector<node>> temp3;
		for (int i = 0; i < row_num[k]; i++)
		{
			vector<node> temp4;
			for (int j = 0; j < col_num[k]; j++)
			{
				temp4.push_back(node(j, i));
			}
			temp3.push_back(temp4);
		}
		map_matrix_A.push_back(temp3);


		vector<vector<int>> temp5;
		for (int i = 0; i < row_num[k]; i++)
		{
			vector<int> temp6;
			for (int j = 0; j < col_num[k]; j++)
			{
				temp6.push_back(0);
			}
			temp5.push_back(temp6);
		}
		density_map.push_back(temp5);

	}
	

}


void init_agent_seq(string seq_file)
{
	agent_num = 0;
	fstream infile(seq_file, ios::in);
	string buf = "";
	getline(infile, buf);
	while (!infile.eof())
	{
		agent_num++;
		stringstream ss(buf);
		seq.push_back(vector<double>());
		double temp;
		while (!ss.eof())
		{
			ss >> temp;
			seq[seq.size() - 1].push_back(temp);
		}
		getline(infile, buf);
	}
	cout << "Initializing " << agent_num << " agent(s)..." << endl;;
	for (int i = 0; i < agent_num; i++)
	{
		AGENT a;
		a.id = int(seq[i][0]);
		a.x = seq[i][1];
		a.y = seq[i][2];
		a.level = int(seq[i][3]);
		a.m = 80;
		a.gx = seq[i][4];
		a.gy = seq[i][5];
		a.fgx = seq[i][4];
		a.fgy = seq[i][5];
		a.goal_level = int(seq[i][6]);
		a.vx = randval(-2, 2);
		a.vy = randval(-2, 2);
		a.v0 = MAX_V;
		a.next_gx = seq[i][4];
		a.next_gy = seq[i][5];
		a.dis = sqrt((a.x - a.gx) * (a.x - a.gx) + (a.y - a.gy) * (a.y - a.gy)) * (abs(a.goal_level - a.level) + 1);
		int rand = int(randval(0, 6));
		a.color = rand;
		a.arrive_time = seq[i][7];
		agent_back_list.push_back(a);
	}


}


void init_agent(int agent_num)
{
	cout << "Initializing " << agent_num << " agent(s)" << endl;;
	for (int i = 0; i < agent_num; i++)
	{
		int rand = int(randval(0, 6));
		//cout << rand << endl;
		AGENT a;
		a.id = i;
		//a.level = int(randval(0, total_level));
		a.level = 0;
		do
		{
			a.x = randval(0, col_num[a.level] - 1) / map_factor;
			a.y = randval(0, row_num[a.level] - 1) / map_factor;
			//a.x = 31.35;
			//a.y = 50.61;

		} while (map_matrix[a.level][int(a.y * map_factor)][int(a.x * map_factor)] == 0);

		a.m = 80;
		a.vx = randval(-2, 2);
		a.vy = randval(-2, 2);
		a.v0 = MAX_V;
		//a.gx = 32;
		//a.gy = 22.5;
		a.gx = goal[rand].x;
		a.gy = goal[rand].y;
		a.fgx = goal[rand].x;
		a.fgy = goal[rand].y;
		a.goal_level = goal[rand].level;
		a.color = rand;
		//cout << a.gx << " " << a.gy << endl;
		//a.next_gx = 32;
		//a.next_gy = 22.5;
		a.next_gx = goal[rand].x;
		a.next_gy = goal[rand].y;
		a.dis = sqrt((a.x - a.gx) * (a.x - a.gx) + (a.y - a.gy) * (a.y - a.gy)) * (abs(a.goal_level-a.level)+1);
		
		// 在这里push a到Q->out_list会出问题
		

		agent_list.push_back(a);
	}

}


void init()
{
	string mapFiles[] = { "./map/data1/0.txt","./map/data1/1.txt" ,"./map/data1/2.txt" };
	string obFiles[] = { "./map/data3/0.txt" ,"./map/data3/1.txt" ,"./map/data3/2.txt" };
	init_obline(obFiles,total_level);
	init_map(mapFiles,total_level);
	for (int k = 0; k < total_level; ++k)
	{
		cout <<"level: "<<k<< " row_num: " << row_num[k] << " col_bnum: " << col_num[k] << endl;
	}
	
	init_agent(agent_num);
	//init_agent_seq("./map/seq.txt");
	cout << "Init finish" << endl;
}


void output(AGENT* a)
{
	fprintf(f, "%d %g %g %d %d %d\n", a->id, a->x, a->y,a->level , a->np, a->color);
	fprintf(ff, "%d,%g,%g,%d,%g,%g\n", a->id, a->x, a->y,a->level, a->vx, a->vy);
}


void step()
{
	total_time += tick;
	push_new_agent();
	fprintf(f, "%lld\n", agent_list.size());
	fprintf(ff, "%lld\n", agent_list.size());

	//单个测试用
	//fprintf(f, "%lu\n", 2);

	for (int i = 0; i < agent_list.size(); ++i)
	{
		AGENT* a = &agent_list[i];
		
		if (a->arrived)
		{
			if (a->go_queue)
			{
				//cout << "arrived" << endl;
				a->arrived = false;
				a->go_queue = false;
				a->in_queue = true;
				a->Q->a_num += 1;
				//cout << "aaa "<<a->Q->a_num << endl;
				a->order = a->Q->a_num;
				//cout <<"bbb " << a->order << endl;
				a->path.clear();
				if (a->order > a->Q->point_num)
				{
					a->Q->point_num = a->order;
					a->Q->queue_back();
				}
				
				a->next_gx = a->Q->path[a->order].x;
				a->next_gy = a->Q->path[a->order].y;
				//cout << a->next_gx << " " << a->next_gy << endl;
				//a->Q->in_list.push_back(a);
			}
		}

		if (a->in_queue && a->order == 1 && sqrt((a->x - a->Q->x)* (a->x - a->Q->x)+(a->y - a->Q->y)* (a->x - a->Q->x))<1)
		{
			//cout << "order 1" << endl;
			a->process_time += tick;
			if (a->process_time >= 3)
			{
				a->process_time = 0;
				a->Q->a_num -= 1;
				for (auto& a_q : a->Q->out_list)
				{
					if (a_q->order  && a_q->in_queue)
					{
						a_q->order -= 1;
						a_q->next_gx = a->Q->path[a_q->order].x;
						a_q->next_gy = a->Q->path[a_q->order].y;
					}
				}
				a->in_queue = false;
				a->Q->out_list.remove(a);
				// cout << a->order << endl;
				int rand = int(randval(0, 6));
				a->fgx = goal[rand].x;
				a->fgy = goal[rand].y;
				a->goal_level = goal[rand].level;
				update_g(a);
			}
		}

		if (a->np == 1)
		{
			output(a);
			continue;
		}
		//first compute the desired direction;
		double goal_dis = sqrt((a->x - a->next_gx) * (a->x - a->next_gx) + (a->y - a->next_gy) * (a->y - a->next_gy));
		if (goal_dis == 0)goal_dis = 1e-10;
		a->tao_1 = 0.2 + (goal_dis / a->dis) * 0.3;
		double dx = a->v0 * (a->next_gx - a->x) / goal_dis;//期望方向向量的x
		double dy = a->v0 * (a->next_gy - a->y) / goal_dis;

		double ax = (dx - a->vx) / a->tao_1;//加速度
		double ay = (dy - a->vy) / a->tao_1;

		double total_force_x = 0;
		double total_force_y = 0;

		if ((!a->arrived) && a->vx <= 0.1 && a->vy <= 0.1 && a->np != 1 && !a->in_queue)
		{
			a->tao_1 *= ((jam_time_threshole - a->jam_time) / jam_time_threshole);
			a->jam_time += 1;
		}
		else
		{
			a->jam_time = 0;
		}


		int agent_counter = 0;

		for (int j = 0; j < agent_list.size(); ++j)
		{
			AGENT* b = &agent_list[j];
			if (b->id == a->id || a->level != b->level)continue;
			double dis = agent_dis(a, b);
			if (dis <= sense_range)//超出感知范围不计算
			{
				agent_counter++;
				agent_force(a, b, &total_force_x, &total_force_y, dis);
			}
		}
		//cout << agent_counter << endl;
		//12 25
		if (a->jam_time >= jam_time_threshole && agent_counter > 10 && agent_counter <= 20) // 还没被挤入人群的绕路走,已经在人群里的就别搜了老老实实挤吧
		{
			a->jam_time = 0;
			a->path.clear();
			A_star(a);

		}
		else if (a->jam_time >= jam_time_threshole + 100)
		{
			a->jam_time = 0;
			a->path.clear();
			A_star(a);
		}


		//int id = omp_get_thread_num();
		for (int j = 0; j < obstical_line_num[a->level]; j++)
		{
			obline_force(a, &obline_list[a->level][j], &total_force_x, &total_force_y);
		}

		ax += total_force_x / a->m;//加速度加上社会力的影响
		ay += total_force_y / a->m;

		a->vx += ax * tick;
		a->vy += ay * tick;

		double new_v = sqrt(a->vx * a->vx + a->vy * a->vy);//更新后的速度大小
		if (new_v > a->v0)//超过最大速度后调整
		{
			a->vx = a->vx * a->v0 / new_v;
			a->vy = a->vy * a->v0 / new_v;
		}
		new_v = sqrt(a->vx * a->vx + a->vy * a->vy);

		a->x += a->vx * tick;
		a->y += a->vy * tick;

		if (a->x < 0)
		{
			a->x = 1e-10;
		}
		if (a->y < 0)
		{
			a->y = 1e-10;
		}
		//cout << agent_counter << endl;

		a->arrive_range = agent_counter > 25 ? 3 : 1;
	
		
		if (a->path.size() > 1 && sqrt((a->x * map_factor - a->path.front().x) * (a->x * map_factor - a->path.front().x) + (a->y * map_factor - a->path.front().y) * (a->y * map_factor - a->path.front().y)) < a->arrive_range * a_step)
		{

			a->path.pop_front();
			a->next_gx = a->path.front().x / map_factor;
			a->next_gy = a->path.front().y / map_factor;
		}
		else if (a->path.size() == 1 && !a->go_queue && !a->in_queue && sqrt((a->x * map_factor - a->path.front().x) * (a->x * map_factor - a->path.front().x) + (a->y * map_factor - a->path.front().y) * (a->y * map_factor - a->path.front().y)) < a->arrive_range * a_step)
		{
			// 判断是否到达最终目标,是则标注已到达,否则执行上下楼函数
			if (a->goal_level == a->level)
			{
				a->arrived = true;

			}// 注意: 这里假设的是agent已经走到这一层的目标点了,默认情况下如果不是目标层,这一层的目标点一定是楼梯位
			else if (a->level < a->goal_level)
			{
				a->level += 1; // 还在楼下就上楼
				a->x = s1.down_x + int(randval(-2, 2));
				a->y = s1.down_y + int(randval(-2, 2));
				update_g(a);
			}
			else if (a->level > a->goal_level)
			{
				a->level -= 1; // 还在楼上就下楼
				a->x = s1.up_x + int(randval(-2, 2));
				a->y = s1.up_y + int(randval(-2, 2));
				update_g(a);
			}

		}

		if (a->go_queue && a->path.size() <= 10 && a->goal_level == a->level && sqrt((a->x * map_factor - a->path.front().x) * (a->x * map_factor - a->path.front().x) + (a->y * map_factor - a->path.front().y) * (a->y * map_factor - a->path.front().y)) < 100)
		{
				a->arrived = true;
		}

		
		


		output(a);

	}
}


void update_density()
{
	for (int k = 0; k < total_level; ++k)
	{
		for (int i = 0; i < row_num[k]; ++i)
		{

			for (int j = 0; j < col_num[k]; ++j)
			{
				density_map[k][i][j] = 0;
			}
		}

	}
	
	for (int i = 0; i < agent_list.size(); ++i)
	{
		if (agent_list[i].arrived)continue;
		if (agent_list[i].vx <= 0.1 && agent_list[i].vy <= 0.1)
		{

			for (int j = -3; j <= 3; ++j)
			{
				for (int k = -3; k <= 3; ++k)
				{

					if (in_map(int(agent_list[i].x * map_factor + j), int(agent_list[i].y * map_factor + k), agent_list[i].level))
					{
						density_map[agent_list[i].level][int(agent_list[i].y * map_factor + k)][int(agent_list[i].x * map_factor + j)] += 2 * (abs(j) + abs(k));//***
					}
				}
			}
		}


	}
}


void test()
{
	//init_obline("./map/obstacles3.txt");
	//init_map("./map/matrix3.txt");
	cout << "row_num: " << row_num << " col_bnum: " << col_num << endl;

	AGENT a;
	a.id = 1;
	a.m = 80;
	a.gx = 1;
	a.gy = 0;
	a.x = 0;
	a.y = 0;

	A_star(&a);



}


void push_new_agent()
{
	for (int i = 0; i < agent_back_list.size(); ++i)
	{
		if (total_time >= agent_back_list[i].arrive_time)
		{
			update_g(&agent_back_list[i]);
			agent_list.push_back(agent_back_list[i]);
			agent_back_list.erase(agent_back_list.begin() + i);

		}
	}
}



// 暂时没用
double mult(double x1, double y1, double x2, double y2, double x3, double y3)
{
	return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
}

bool cross(double sx1, double sy1, double ex1, double ey1, double sx2, double sy2, double ex2, double ey2)
{
	if (max(sx1, ex1) < min(sx2, ex2))return false;
	if (max(sy1, ey1) < min(sy2, ey2))return false;
	if (max(sx2, ex2) < min(sx1, ex1))return false;
	if (max(sy2, ey2) < min(sy1, ey1))return false;
	if (mult(sx2, sy2, ex1, ey1, sx1, sx2) * mult(ex1, ey1, ex2, ey2, sx1, sy1) < 0)return false;
	if (mult(sx1, sy1, ex2, ey2, sx2, sy2) * mult(ex2, ey2, ex1, ey1, sx2, sy2) < 0)return false;
	return true;
}



// main
int main()
{


	total_start = clock();

	// omp_set_num_threads(thread_num);

	init();


	// 初始状态输出
	fprintf(f, "%lld\n", agent_list.size());
	fprintf(ff, "%d,%g\n", step_num, map_factor);
	fprintf(ff, "%lld\n", agent_list.size());
	for (int i = 0; i < agent_list.size(); ++i)
	{
		output(&agent_list[i]);
	}

	// 初始A★
	int counter = 0;
	cout << "Initializing path..." << endl;
	for (auto& a : agent_list)
	{
		a.go_queue = true;
		if (a.go_queue)
		{
			
			
			int rand_q = int(randval(0, 2));
			a.Q = q_list[rand_q];
			
			a.order = 0;
			// cout << a.order << endl;
			a.Q->out_list.push_back(&a);
			go_queue(&a);
		}
		update_g(&a);
		counter++;
		printf("A_star: %d / %d \r", counter, agent_num);
	}
	cout << endl;
	// 开始模拟
	clock_t start, end;
	cout << "Start steps..." << endl;
	start = clock();
	for (int i = 0; i < step_num; ++i)
	{
		step();

		if ((i + 1) % jam_time_threshole == 0)
		{
			update_density();
		}
		if ((i + 1) % 10 == 0)
		{

			end = clock();
			printf("Step: %d / %d ;Remaining time: %d s             \r", i + 1, step_num, (end - start) * (step_num - i - 1) / 10 / 1000);
			start = clock();
		}

	}
	cout << endl;

	total_end = clock();
	cout << "Done " << step_num << " step(s). Total time :" << (total_end - total_start) / 1000 / 60 << " min(s)" << endl;

}
