#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "DStar.h"

using namespace std;

int main(int argc, char const *argv[])
{

	int r=3;
	int c=4;
	vector<vector<int>> v;
	v.resize(r);
	for(int i=0;i<r;i++)
		v[i].resize(c);
	cout<<v.size()<<'\n';
    
	return 0;
}