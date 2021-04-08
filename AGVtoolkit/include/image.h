#ifndef IMAGE_H
#define IMAGE_H

#include <string>
class Image
{
public:
    Image(int x, int y);
    Image(const std::string &filename);
	Image(const Image& img);
    ~Image();
    void save(const std::string& filename);
    unsigned int& operator() (int x, int y);

    inline int width() const
    {
        return _width;
    }
    inline int height() const
    {
        return _height;
    }

protected:
    int _width, _height;
    int _comp;
    void *_data;
};
class Map:public Image
{
public:
	static const unsigned int UNKOWN = 0x00FFFFFF;
	static inline unsigned int red(unsigned int& input)
	{
		return input &0x000000FF;
	}
	static inline unsigned int green(unsigned int& input)
	{
		return (input>>8) &0x000000FF;
	}
	static inline unsigned int blue(unsigned int& input)
	{
		return (input>>16) &0x000000FF;
	}
    inline Map(const std::string &filename)
        : Image(filename)
    {
        _resolution = *( (float*)_data);
        _x = *( (float*)_data + 1);
        _x = *( (float*)_data + 2);
    }
    inline Map(int w, int h, double res)
        :_resolution(res), Image(w,h), _x(0), _y(0)
    {
        *( (float*)_data) = _resolution;
    }
    inline void set_origin(double x, double y)
    {
        _x = x;
        _y = y;
        *( (float*)_data + 1) = _x;
        *( (float*)_data + 2) = _y;
    }
    inline void set_resolution(double res)
    {
        _resolution = res;
    }
    inline double origin_x() const
    {
        return _x;
    }
    inline double origin_y() const
    {
        return _y;
    }

    inline double resolution() const
    {
        return _resolution;
    }
	inline void save(const std::string& filename, bool transparent = true)
	{
		unsigned int *p = (unsigned int*)_data;
		if(_data && !transparent)
		{
			for(int i = 0; i < _width * _height; i++)
				*(p++) |= 0xFF000000;
		}
		*( (float*)_data) = _resolution;
		*( (float*)_data + 1) = _x;
        *( (float*)_data + 2) = _y;
		Image::save(filename);
	}

private:
    double _resolution;
    double _x;
    double _y;
};

#endif // IMAGE_H
