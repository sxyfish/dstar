#ifndef _DSTARCELL_H_
#define _DSTARCELL_H_
#include <queue>
#include <vector>
#include "globalVariables.h"




class DStarCell
{
public:

    double get_g()
	{
		return _g;
	}
	double get_h()
	{
		return _h;
	}
	double get_rhs()
	{
		return _rhs;
	}
	double* get_key()
	{
		return _key;
	}
   
	char get_type()
	{
		return _type;
	}
	char get_status()
	{
		return _status;
	}
	double get_cost()
	{
		return _cost;
	}
	int get_posx()
	{
		return _posx;
	}
	int get_posy()
	{
		return _posy;
	}
	void set_g(double g)
	{
		_g=g;
	}
	void set_h(double h)
	{
		_h=h;
	}
	void set_rhs(double rhs)
	{
		_rhs=rhs;
	}
	/*void set_key(double key[])
	{
		_key=key;
	}*/
	
	void set_type(char t)
	{
		_type=t;
	}
	void set_status(char s)
	{
		_status=s;
	}
	void set_cost(double c)
	{
		_cost=c;
	}
	void set_posx(int x)
	{
		_posx=x;
	}
	void set_posy(int y)
	{
		_posy=y;
	}
	DStarCell** get_move()
	{
		return move;
	}
/*	void set_move()
	{

	}
*/


	bool keyLessThan(DStarCell* cell);
	void calcKey(double k);
	double calc_H(DStarCell* cell);
	double calCost(DStarCell* cell);
	void updateRhs();
	DStarCell* getNext();

private:

	DStarCell* move[DIRECTIONS];
	
	double _g;
	double _rhs;
	double _h;
	double _key[2];
    double _cost;
	int _posx;
	int _posy;

	char _type;
	char _status;

};

#endif