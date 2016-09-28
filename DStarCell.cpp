#include "stdafx.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include "stdafx.h"
#include "DStarCell.h"


using namespace std;

void DStarCell::calcKey(double k)
{
	_key[1]=min(this->_g,this->_rhs);
	_key[0]=this->_h+_key[1]+k;
}
double DStarCell::calc_H(DStarCell* cell)
{
	int diffx=abs(this->_posx-cell->_posx);
	int diffy=abs(this->_posy-cell->_posy);
    
	double h=SQRT_2*min(diffx,diffy)+max(diffx,diffy);
	return h;

}
double DStarCell::calCost(DStarCell* cell)
{
	int dx=abs(this->_posx-cell->_posx);
	int dy=abs(this->_posy-cell->_posy);

	if (this->_cost == INF || cell->_cost == INF)
		return INF;

	double scale = 1.0;

	if ((dx + dy) > 1)
	{
		scale = SQRT_2;
	}

	return scale * ((this->_cost + cell->_cost) / 2);
}

void DStarCell::updateRhs()
{
	double min_rhs=INF;
	for (int i=0;i<DIRECTIONS;i++)
	{
		if (move[i]!=NULL)
		{
			double new_rhs=calCost(move[i])+move[i]->_g;
			if (new_rhs<min_rhs)
			{
				min_rhs=new_rhs;

			}
		}

	}
	this->_rhs=min_rhs;
}

DStarCell* DStarCell::getNext()
{
   DStarCell* n=NULL;
   double mg=INF;
   for (int i=0;i<DIRECTIONS;i++)
   {
	   DStarCell* m=move[i];
	   if (m != NULL)
	   {
          if(m->_g < mg)
		  {
			  mg=m->_g;
			  n=m;
		  }
	   }
   }
   return n;
}
bool DStarCell::keyLessThan(DStarCell* cell)
{
	if (this->_key[0]<cell->_key[0])
	{
		return true;
	} else if ((this->_key[0]==cell->_key[0]) && (this->_key[1]<cell->_key[1]))
	{
		return true;
	}else
		return false;
}
