#include "carmenmap.hpp"

carmenMap::carmenMap():
    x_size(0),y_size(0),resolution(0.1),complete_map(NULL) 
{

}
carmenMap::carmenMap(const char* path):
    x_size(0),y_size(0),resolution(0.1),complete_map(NULL) 
{
    open(path);
}

carmenMap::~carmenMap()
{
    if(complete_map)
        delete complete_map;
    complete_map = NULL;
}
carmenMap::carmenMap(int x, int y,float res)
        :x_size(x),y_size(y),resolution(res) 
{
    complete_map = new float[sizeof(float) * x_size * y_size];
}
float* carmenMap::getData()
{
    return complete_map;
}
void carmenMap::resize(int x, int y)
{
    x_size = x;
    y_size = y;
    if(complete_map)
        delete complete_map;
    complete_map = new float[sizeof(float) * x_size * y_size];
}

int carmenMap::open(const char* path)
{
    FILE* fp = fopen(path, "rb");
    if(fp == NULL)
        return -1;
    char buffer[12];
    
    /*find the head of the map file*/
    do{
        if( fread(buffer, 10, 1, fp) < 1)
        {
            fclose(fp);
            return -2;
        }
        buffer[10] = '\0';
        fseek(fp,-9, SEEK_CUR);
    }while(strstr(buffer, "GRIDMAP   ") == NULL);
    fseek(fp,9, SEEK_CUR);  //in some times 9 liao0322.map
    
    /*read the map data*/
    if(1 != fread(&x_size, sizeof(int), 1, fp))
        throw "read map x_size error!";
    if(1 !=fread(&y_size, sizeof(int), 1, fp))
        throw "read map y_size error!";
    if(1 !=fread(&resolution, sizeof(float), 1, fp))
        throw "read map error!";
    std::cout<<x_size<<" X "<<y_size
        <<" @ "<<resolution<<std::endl;
    if(x_size <= 0 || y_size <= 0)
    {
        throw "read map size error!";
    }
    if(complete_map)
        delete complete_map;
    complete_map = NULL;
    complete_map = new float[sizeof(float) * x_size * y_size];
    if( 1 != fread(complete_map, sizeof(float) * x_size * y_size, 1, fp))
        throw "read map data error!";
    fclose(fp);
    return 0;
}
int carmenMap::width() const
{
    return x_size;
}
float carmenMap::scale() const
{
    return resolution;
}
int carmenMap::height() const
{
    return y_size;
}
float carmenMap::operator() (const int &x,const int &y)
{
    return *(complete_map + x * y_size + y);
}


