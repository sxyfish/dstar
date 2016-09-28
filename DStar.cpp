#include "stdafx.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include <algorithm>

#include "DStar.h"
#include "DStarCell.h"
#include "gridworld.h"


 
void  DStar::initialise(int startX, int startY, int goalX, int goalY){
	while (!U.empty())
	{
		U.clear();
	}

	km=0.0;

	for(int i=0; i < rows; i++){
		for(int j=0; j < cols; j++){
			DStarCell dc=maze[i][j];
			setNeighbors(i,j);
			dc.set_h(dc.calc_H(start));
			dc.calcKey(km);

		}
	}

	U.push_back(goal);
	//---------------------

	//for debugging only
	//~ for(int i=0; i < rows; i++){
	//~ for(int j=0; j < cols; j++){
	//~ //cout << maze[i][j].g << ", ";
	//~ cout << maze[i][j].rhs << ", ";

	//~ }
	//~ cout << endl;
	//~ }

}
void DStar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].set_h(start->calc_H(&maze[i][j]));
		}
	}
	
}


void DStar::updateAllKeyValues(){	
	for(int i=0; i < rows; i++){
		for(int j=0; j < cols; j++)
		   maze[i][j].calcKey(km);
	}
	
	start->calcKey(km);
	goal->calcKey(km);
	
}





void DStar::setNeighbors(int i,int j)
{
	
			DStarCell** cp=maze[i][j].get_move();
			for (unsigned int k = 0; k < DIRECTIONS; k++)
				 cp[k]=NULL;
			
			// Top
			if (i != 0)
			{
				// Top left
				if (j != 0)
					cp[0] = &maze[i - 1][j - 1];			
				// Top middle
				cp[1] = &maze[i - 1][j];
                // Top right
				if (j < cols - 1)
					cp[2] = &maze[i - 1][j + 1];	

			}
            // Middle right
			if (j < cols - 1)
				cp[3] = &maze[i][j + 1];
	
			// Bottom
			if (i < rows - 1)
			{
				// Bottom right
				if (j < cols - 1)
					cp[4] = &maze[i + 1][j + 1];
					
				// Bottom middle
				cp[5] = &maze[i + 1][j];
				
                // Bottom left
				if (j != 0)
					cp[6] = &maze[i + 1][j - 1];
				
			}

			// Middle left
			if (j != 0)
				cp[7] = &maze[i][j - 1];	
	
}


void DStar::updateVertex(DStarCell* cell)
{
	numberOfVertexAccesses++;
	if (cell != goal)
	{
		cell->updateRhs();
	}

	vector<DStarCell*>::iterator pos=posVertex(cell);
	if (pos!=U.end())
	{
		U.erase(pos);
	}
	if (cell->get_g()!=cell->get_rhs())
	{
		insertU(cell);
	}
	 
}
void DStar::updateVertexNeighbors(DStarCell* cell)
{
	DStarCell** cp=cell->get_move();
	for (int i=0;i<DIRECTIONS;i++)
		if (cp[i]!=NULL)
			updateVertex(cp[i]);
}

vector<DStarCell*>::iterator DStar::posVertex(DStarCell* cell)
{
	int p=0;
	for (int i=0;i<U.size();i++)
	{
		if (U[i]==cell)
		{
			p=i;
			break;
			
		}
	}
	return (U.begin()+p);
}
void DStar::insertU(DStarCell* cell)
{
	U.push_back(cell);
	push_heap(U.begin(),U.end(),compareDStarCell());
}

DStarCell* DStar::getTopU()
{
	return U.front();
	
}
void DStar::popU()
{
	pop_heap(U.begin(),U.end(),compareDStarCell());
	U.pop_back();
}
void DStar::detectChangeAndUpdateVertex(DStarCell* cell)
{
    DStarCell** cp=cell->get_move();
	for (int i=0;i<DIRECTIONS;i++)
	{
		if (cp[i]!=NULL)
		{
			if (cp[i]->get_type()==8)
			{
				cp[i]->set_cost(INF);
				updateVertexNeighbors(cp[i]);
				
			}
		}
	}
}
bool DStar::hasArrived()
{
	if (start==goal)
		return true;
	else
		return false;
	
}
void DStar::computeShortestPath()
{
	start->set_type('4');
	int s=U.size();
	if (s>maxQLength)
	{
		maxQLength=s;
	}
	while( getTopU()->keyLessThan(start) || start->get_g()!=start->get_rhs())
	{
		DStarCell* u=getTopU();
		double* kold=u->get_key();
		popU();
		numberOfExpandedStates++;
		if (u->keyLessThan(u))
		{
			insertU(u);
		}
		else if (u->get_g() > u->get_rhs())
		{
			u->set_g(u->get_rhs());
			updateVertexNeighbors(u);
		} 
		else
		{
			u->set_g(INF);
			updateVertex(u);
			updateVertexNeighbors(u);
		}
	}
}
void DStar::replan()
{
		
		start=start->getNext();
		updateHValues();
		km=km+start->calc_H(last);
		last=start;
		detectChangeAndUpdateVertex(start);
}



