#ifndef  _DSTAR_H_
#define  _DSTAR_H_

#include <vector> 
#include <queue>
#include <algorithm>
#include "globalVariables.h"
#include "gridworld.h"
#include "DStarCell.h"

class GridWorld;
class DStar
{
	  

	public:

		class compareDStarCell
		{
		public:
			bool operator()(DStarCell* c1,DStarCell* c2) const
			{
				double* lk=c1->get_key();
				double* rk=c2->get_key();
				if (rk[0]<lk[0])
				{
					return true;
				} else if ( (rk[0]==lk[0]) && (rk[1]<lk[1]) )
				{
					return true;
				}else
					return false;
			}
		};
		DStar::DStar(){

		}

		DStar(int rows_, int cols_){
			rows = rows_;
			cols = cols_;

			//Allocate memory 
			maze.resize(rows);
			for(int i=0; i < rows; i++){
				maze[i].resize(cols);
			}

		}
	    void initialise(int startX, int startY, int goalX, int goalY);
		void setNeighbors(int i,int j);
	    void updateHValues();
	    void updateAllKeyValues();
        void updateVertex(DStarCell* cell);
		void updateVertexNeighbors(DStarCell* cell);
		void detectChangeAndUpdateVertex(DStarCell* cell);
		bool hasArrived();

		vector<DStarCell*>::iterator posVertex(DStarCell* cell);
		void insertU(DStarCell* cell);
		DStarCell* getTopU();
		void popU();
		
        void computeShortestPath();
		void replan();
		

		friend void copyMazeToDisplayMap(GridWorld &gWorld, DStar* dsp);
		friend void copyDisplayMapToMaze(GridWorld &gWorld, DStar* dsp);


	private:
	
    	vector<vector<DStarCell> > maze;
        vector<DStarCell*> U;
    	//priority_queue<DStarCell*,vector<DStarCell*>,compareDStarCell> U;
    	DStarCell* start;
    	DStarCell* goal;
		DStarCell* last;
    	int rows;
    	int cols;
		double km;
    
};










#endif