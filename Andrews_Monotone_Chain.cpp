//UVA 596
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <cstring>
using namespace std;

#define MAXN 100005
#define STRINGSIZE 200
#define INF 1e9

typedef struct T_Point_Info{
	int iX,iY;
}T_Point;

T_Point g_tInput[MAXN], g_tConvexHull[MAXN*2];
int g_iInputNum, g_iConvexHullNum;

// "vector OA cross vector OB > 0" means OB is counter-clockwise from OA
int Cross(const T_Point& tPointO, const T_Point& tPointA, const T_Point& tPointB)
{
	return (tPointA.iX - tPointO.iX) * (tPointB.iY - tPointO.iY) - (tPointA.iY - tPointO.iY) * (tPointB.iX - tPointO.iX);
}
// choose bigger x and smaller y if the two x are the same
bool CompareCoordinateForAndrewsMonotoneChain(const T_Point& tPointA, const T_Point& tPointB)
{
	return (tPointA.iX > tPointB.iX) || (tPointA.iX == tPointB.iX && tPointA.iY < tPointB.iY);
}
// compare the two point are the same or not
bool CompareSamePoint(const T_Point& tPointA, const T_Point& tPointB)
{
	return (tPointA.iX == tPointB.iX) && (tPointA.iY == tPointB.iY);
}
// include all collinear nodes
void OutputConvexHullPoint()
{
	for (int i = 0; i < g_iConvexHullNum; ++i)
	{
		printf(" (%d,%d)", g_tConvexHull[i].iX, g_tConvexHull[i].iY);
	}
	printf("\n");
}
// quick hull
void QuickHull()
{
	T_Point tMinX={(int)INF,(int)INF}, tMaxX={(int)-INF,(int)INF}, tMinY={(int)INF,(int)INF}, tMaxY={(int)-INF,(int)INF};
	int iInputNumAfterQuickHull = 0;
	//printf("%d %d %d %d %d %d %d %d\n", tMinX.iX, tMinX.iY, tMaxX.iX, tMaxX.iY, tMinY.iX, tMinY.iY, tMaxY.iX, tMaxY.iY);
	
	for (int i = 0; i < g_iInputNum; ++i)
	{
		if (g_tInput[i].iX < tMinX.iX || (g_tInput[i].iX == tMinX.iX && g_tInput[i].iY < tMinX.iY))
		{
			tMinX = g_tInput[i];
		}
		if (g_tInput[i].iX > tMaxX.iX || (g_tInput[i].iX == tMaxX.iX && g_tInput[i].iY < tMaxX.iY))
		{
			tMaxX = g_tInput[i];
		}
		if (g_tInput[i].iY > tMinY.iY || (g_tInput[i].iY == tMinY.iY && g_tInput[i].iX < tMinY.iX))
		{
			tMinY = g_tInput[i];
		}
		if (g_tInput[i].iY > tMaxY.iY || (g_tInput[i].iY == tMaxY.iY && g_tInput[i].iX < tMaxY.iX))
		{
			tMaxY = g_tInput[i];
		}
	}
	
	for (int i = 0; i < g_iInputNum; ++i)
	{
		if (((tMinX.iX != tMaxY.iX || tMinX.iY != tMaxY.iY) && Cross(tMinX, tMaxY, g_tInput[i]) >= 0) || 
			((tMaxY.iX != tMaxX.iX || tMaxY.iY != tMaxX.iY) && Cross(tMaxY, tMaxX, g_tInput[i]) >= 0) ||
			((tMaxX.iX != tMinY.iX || tMaxX.iY != tMinY.iY) && Cross(tMaxX, tMinY, g_tInput[i]) >= 0) ||
			((tMinY.iX != tMinX.iX || tMinY.iY != tMinX.iY) && Cross(tMinY, tMinX, g_tInput[i]) >= 0))
		{
			g_tInput[iInputNumAfterQuickHull++] = g_tInput[i];
			continue;
		}
	}
	
	g_iInputNum = iInputNumAfterQuickHull;
}
// main algorithm
void AndrewsMonotoneChain()
{
	QuickHull();

	sort(g_tInput, g_tInput+g_iInputNum, CompareCoordinateForAndrewsMonotoneChain);
// upper hull
	for (int i = 0; i < g_iInputNum; ++i)
	{
		// skip the same node
		if ((i != g_iInputNum-1) && CompareSamePoint(g_tInput[i], g_tInput[i+1]))
		{
			continue;
		}
		// If you want to eliminate the collinear point, you should do it when the return value of "Cross" is not positive (<=0)
		while (g_iConvexHullNum >= 2 && Cross(g_tConvexHull[g_iConvexHullNum-2], g_tConvexHull[g_iConvexHullNum-1], g_tInput[i]) < 0)
		{
/*			printf("node 1: (%d,%d)\n, node 2: (%d,%d)\n, node 3: (%d,%d)\n", g_tConvexHull[g_iConvexHullNum-2].iX, g_tConvexHull[g_iConvexHullNum-2].iY,
																				g_tConvexHull[g_iConvexHullNum-1].iX, g_tConvexHull[g_iConvexHullNum-1].iY,
																				g_tInput[i].iX, g_tInput[i].iY);*/
			--g_iConvexHullNum;
		}
		g_tConvexHull[g_iConvexHullNum++] = g_tInput[i];
		//printf("%d: (%d,%d)\n", i, g_tInput[i].iX, g_tInput[i].iY);
	}
// lower hull
	for (int i = g_iInputNum-2, k = g_iConvexHullNum+1; i >= 0; --i)
	{
		//skip the same node
		if (CompareSamePoint(g_tInput[i], g_tInput[i+1]))
		{
			continue;
		}
		// If you want to eliminate the collinear point, you should do it when the return value of "Cross" is not positive (<=0)
		while (g_iConvexHullNum >= k && Cross(g_tConvexHull[g_iConvexHullNum-2], g_tConvexHull[g_iConvexHullNum-1], g_tInput[i]) < 0)
		{
			--g_iConvexHullNum;
		}
		g_tConvexHull[g_iConvexHullNum++] = g_tInput[i];
	}
	//Eliminate the first point because it was added twice
	--g_iConvexHullNum;
}

void ReadPoint()
{
	int iPointNum;

	scanf("%d", &iPointNum);
	for (int i = g_iInputNum; i < g_iInputNum + iPointNum; ++i)
	{
		scanf("%d %d", &g_tInput[i].iX, &g_tInput[i].iY);
	}
	
	g_iInputNum += iPointNum;
}

void ReadSetName()
{
	char szSetName[STRINGSIZE];

	fgets(szSetName, STRINGSIZE, stdin);
	
	for (int i = 1; i < strlen(szSetName) - 1; ++i)
	{
		printf("%c", szSetName[i]);
	}
	printf(" convex hull:\n");
}

void Initial()
{
	memset(g_tInput, 0, sizeof(g_tInput));
	memset(g_tConvexHull, 0, sizeof(g_tConvexHull));
	g_iInputNum = 0;
	g_iConvexHullNum = 0;
}

int main()
{
	char szCommand[STRINGSIZE];
	bool bFirstSet = true;
	
	scanf("%s", szCommand);
	
	//Read "END" and then break
	while (true)
	{
		if (strcmp(szCommand, "S") == 0)
		{
			// Before starting to read the next set, calculate the first set
			if (bFirstSet == true)
			{
				bFirstSet = false;
			}
			else
			{
				AndrewsMonotoneChain();
				OutputConvexHullPoint();
			}
			
			ReadSetName();
			Initial();
		}
		else if (strcmp(szCommand, "P") == 0)
		{
			ReadPoint();
			//printf("g_iInputNum: %d\n", g_iInputNum);
		}
		else if (strcmp(szCommand, "END") == 0)
		{
			// Calculate the last set
			AndrewsMonotoneChain();
			OutputConvexHullPoint();
			break;
		}
		else 
		{
			printf("[%s]: %d, Impossible command!\n", __func__, __LINE__);
		}
		
		scanf("%s", szCommand);
	}
	
	return 0;
}