#include <cmath>
#include <iostream>
#include "Astar.h"
using std::cout;
void Astar::InitAstar(vMap &_maze, vMap & _enemy)
{
	maze=_maze;
	EnemyMap=_enemy;
}

int Astar::calcG(Point *temp_start,Point *point)
{
	int extraG=kCost1;
	int parentG=point->parent==nullptr?0:point->parent->G; //����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿ�
	return parentG+extraG;
}

int Astar::calcH(Point *point,Point *end)
{
	return (std::abs((end->x-point->x))+std::abs(end->y-point->y))*kCost1;
}

int Astar::calcD(Point *point)
{
    int D[4]={10,10,10,10};
    int block[4]={0,0,0,0};
    int x1,x2,y1,y2,tx,ty;
    x1=point->x; x2=(int)maze.size()-x1-1;
    y1=point->y; y2=(int)maze[0].size()-y1-1;
    int limit[4]={x2,y2,x1,y1};
     for(int i=0;i<4;i++){
       for(int l=1;l<=limit[i];l++){
            tx=x1+dx[i]*l;
            ty=y1+dy[i]*l;
            if(maze[tx][ty]==1) {
                block[i]++;continue;
            }
            if(EnemyMap[tx][ty]==1){
                D[i]=l;
                break;
            }
        }
    }
    int ans=D[0]+block[0]*2;
    for(int i=1;i<4;i++){
        int temp=D[i]+block[i]*2;
         if(temp<ans) ans=temp;
    }
    ans=100*exp(1-ans);
    return ans;
}

int Astar::calcF(Point *point)
{
	return point->G+point->H+point->D;
}

Point *Astar::getLeastFpoint()
{
	if(!openList.empty())
	{
		auto resPoint=openList.front();
		for(auto &point:openList)
			if(point->F<resPoint->F)
				resPoint=point;
		return resPoint;
	}
	return nullptr;
}

Point *Astar::findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
	openList.push_back(new Point(startPoint.x,startPoint.y)); //�������,��������һ���ڵ㣬�������
	while(!openList.empty())
	{
		auto curPoint=getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
		//1,�ҵ���ǰ��Χ�˸����п���ͨ���ĸ���
		auto surroundPoints=getSurroundPoints(curPoint,isIgnoreCorner);
		for(auto &target:surroundPoints)
		{
			//2,��ĳһ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ��Ϊ�丸�ڵ㣬����F G H D
			if(!isInList(openList,target))
			{
				target->parent=curPoint;
				target->G=calcG(curPoint,target);
				target->H=calcH(target,&endPoint);
				target->D=calcD(target);
				target->F=calcF(target);
				openList.push_back(target);
			}
			//3����ĳһ�����ӣ����ڿ����б��У�����Gֵ, �����ԭ���Ĵ�, ��ʲô������, �����������ĸ��ڵ�Ϊ��ǰ��,������G��F
			else
			{
				int tempG=calcG(curPoint,target);
				int tempD=calcD(target);
				if(tempG+tempD<target->G+target->D)
				{
					target->parent=curPoint;
					target->G=tempG;
					target->D=tempD;
					target->F=calcF(target);
				}
			}
			Point *resPoint=isInList(openList,&endPoint);
			if(resPoint)
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
		}
	}
	return nullptr;
}

std::list<Point *> Astar::GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner)
{
	Point *result=findPath(startPoint,endPoint,isIgnoreCorner);
	std::list<Point *> path;
	//����·�������û�ҵ�·�������ؿ�����
	while(result)
	{
		path.push_front(result);
		result=result->parent;
	}
    // �����ʱ�����б���ֹ�ظ�ִ��GetPath���½���쳣
    openList.clear();
	closeList.clear();
	return path;
}
Point *Astar::isInList(const std::list<Point *> &list,const Point *point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
	for(auto p:list)
		if(p->x==point->x&&p->y==point->y)
			return p;
	return nullptr;
}

bool Astar::isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const
{
	if(target->x<0||target->x>int(maze.size()-1)
		||target->y<0||target->y>int(maze[0].size()-1)
		||maze[target->x][target->y]==1
		||isInList(closeList,target)) //������뵱ǰ�ڵ��غϡ�������ͼ�����ϰ�������ڹر��б��У�����false
		return false;
	return true;
}

std::vector<Point *> Astar::getSurroundPoints(const Point *point,bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;
	for(int i=0;i<4;i++){
		int x=point->x+dx[i];
		int y=point->y+dy[i];
        if(isCanreach(point,new Point(x,y),isIgnoreCorner))
            surroundPoints.push_back(new Point(x,y));
	}
	return surroundPoints;
}
