



#ifndef TERRAINMAP
#define TERRAINMAP

struct x_y_coord
{
x_y_coord();
x_y_coord(int x, int y);
int _x;
int _y;
};



int getPos(int x, int y);

bool isInVector(std::vector<x_y_coord> & toCheckAgainst, x_y_coord toCheck);

void findGroupings(x_y_coord start, std::vector<grid_square> & terrain, std::vector<x_y_coord> & grouping);

void terrainMap(std::vector<grid_square> & terrain);





#endif