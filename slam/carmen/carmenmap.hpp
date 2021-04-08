/****************************************************************************
** code by volans.liao@gmail.com
** use for carmen map read
** also can read image files when the QT_LIBRARY is present
***************************************************************************/


#ifndef CARMENMAP_H
#define CARMENMAP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <string>

class carmenMap
{
public:
    carmenMap();
    carmenMap(const char* path);
    carmenMap(int x, int y, float res);
    void resize(int x,int y);
    float* getData();
    ~carmenMap();
    
    int open(const char* path);
    int width() const;
    int height() const;

    float scale() const;
    float operator() (const int &x,const int &y);

private:
    int x_size;
    int y_size;
    float resolution;
    float *complete_map;
};
typedef carmenMap* carmenMapPtr;
#endif
