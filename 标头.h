#pragma once
if (a->arrived == true)
{
	// 在去队列的状态中到达,表示到了排队范围内,下一步加入队列
	if (a->go_queue)
	{
		a->go_queue = false;
		if (a->Q->agent_num == 0)
		{
			a->Q->agent_num += 1;
			a->Q->list.push(a);
			a->in_queue = true;
			cout << "first in queue" << endl;
		}
		else
		{
			a->Q->agent_num += 1;
			a->front_agent = a->Q->list.back();
			a->Q->list.push(a);
			a->in_queue = true;
			cout << "in-queue" << endl;

		}
	}
	// 在队列中的状态中到达,表示可以往前挪挪了
	else if (a->in_queue)
	{
		// 如果是队伍第一个,就可以开始处理了
		if (a->Q->list.front() == a && a->Q->symptom)
		{
			a->processing = true;
			a->in_queue = false;
			a->Q->list.pop();
			a->Q->symptom = false;
			--a->Q->agent_num;

		}
	}
	a->arrived = false;
}

if (a->go_queue)
{
	if (a->Q->agent_num == 0)
	{
		if (a->fgx == a->Q->x && a->fgy == a->Q->y)
		{

		}
		else
		{
			a->fgx = a->Q->x;
			a->fgy = a->Q->y;
			a->goal_level = a->Q->level;
			update_g(a);
		}
	}
	else
	{
		if (a->fgx == a->front_agent->x && a->fgy == a->front_agent->y)
		{
		}
		else
		{
			a->fgx = a->front_agent->x;
			a->fgy = a->front_agent->y;
		}
	}
}

if (a->in_queue)
{
	if (a->Q->list.front() == a)
	{
		a->next_gx = a->Q->x;
		a->next_gy = a->Q->y;
	}
	else
	{
		a->next_gx = a->front_agent->x;
		a->next_gy = a->front_agent->y;
	}
}
else if (a->processing)
{
	a->process_time += tick;
	if (a->process_time > 30)
	{
		a->processing = false;
		int rand = int(randval(0, 6));
		a->fgx = goal[rand].x;
		a->fgy = goal[rand].y;
		a->goal_level = goal[rand].level;
		a->Q->symptom = true;
		update_g(a);
	}
	else
	{
		output(a);
		continue;
	}
}














else if (a->in_queue && (sqrt((a->x - a->next_gx) * (a->x - a->next_gx) + (a->y - a->next_gy) * (a->y * map_factor - a->next_gy)) < 1))
{
	a->arrived = true;
}