#ifndef ASTAR_H_INCLUDED
#define ASTAR_H_INCLUDED
#define vMap std::vector<std::vector<int>>
#define Mypath std::list<Point *>
#include <vector>
#include <list>
const int kCost1=10; //直移一格消耗
const int dx[4]={1,0,-1,0};
const int dy[4]={0,1,0,-1};
struct Point
{
	int x,y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	int F,G,D,H; //F=G+H+D
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(int _x,int _y):x(_x),y(_y),F(0),G(0),H(0),parent(nullptr)  //变量初始化
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
	bool isCanreach(const Point *point,const Point *target,bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
	Point *isInList(const Mypath &list,const Point *point) const; //判断开启/关闭列表中是否包含某点
	Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
	//计算FGH值
	int calcG(Point *temp_start,Point *point);
	int calcH(Point *point,Point *end);
	int calcD(Point *point);
	int calcF(Point *point);
private:
	vMap maze;
	vMap EnemyMap;
	Mypath openList;  //开启列表
	Mypath closeList; //关闭列表
};

#endif // 寻路_H_INCLUDED
