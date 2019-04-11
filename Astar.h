#ifndef ASTAR_H_INCLUDED
#define ASTAR_H_INCLUDED
#define vMap std::vector<std::vector<int>>
#define Mypath std::list<Point *>
#include <vector>
#include <list>
const int kCost1=10; //ֱ��һ������
const int dx[4]={1,0,-1,0};
const int dy[4]={0,1,0,-1};
struct Point
{
	int x,y; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x������ţ�y��������
	int F,G,D,H; //F=G+H+D
	Point *parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���
	Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(nullptr)  //������ʼ��
	{
	}
};

class Astar
{
public:
	void InitAstar(vMap &_maze, vMap &_enemy);
	Mypath GetPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
private:
	Point *findPath(Point &startPoint,Point &endPoint,bool isIgnoreCorner);
	Mypath getSurroundPoints(const Point *point,bool isIgnoreCorner) const;
	bool isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const; //�ж�ĳ���Ƿ����������һ���ж�
	Point *isInList(const Mypath &list,const Point *point) const; //�жϿ���/�ر��б����Ƿ����ĳ��
	Point *getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�
	//����FGHֵ
	int calcG(Point *temp_start,Point *point);
	int calcH(Point *point,Point *end);
	int calcD(Point *point);
	int calcF(Point *point);
private:
	vMap maze;
	vMap EnemyMap;
	Mypath openList;  //�����б�
	Mypath closeList; //�ر��б�
};

#endif // Ѱ·_H_INCLUDED
