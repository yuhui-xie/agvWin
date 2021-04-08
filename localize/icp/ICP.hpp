#ifndef LIB_ICP
#define LIB_ICP
#include <vector>
#ifdef WIN32
#ifndef EXPORT
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT
#endif


class IcpPointToPoint;
class Matrix;
namespace icp
{

struct ICP_Point
{
    double x;
    double y;
    inline ICP_Point(double _x , double _y)
        :x(_x), y(_y){}
};
struct ICP_Pose
{
    double x;
    double y;
    double theta;
    inline ICP_Pose (double x_ = 0, double y_ = 0, double theta_ = 0)
        : x (x_), y (y_), theta (theta_){}
    inline ICP_Pose (const ICP_Pose& p)
        : x (p.x), y (p.y), theta (p.theta){}
};
class EXPORT ICP
{
private:
    IcpPointToPoint * icp;
    Matrix* rotation;
    Matrix* translate;
    double outlier;
public:
    ICP ();
    ~ ICP ();
    void set_outlier (double r);
    void set_model (std::vector <ICP_Point> model);
    void initial_pose (double x, double y, double theta);
    ICP_Pose pose ();
    ICP_Pose match (std::vector <ICP_Point> d);
};

}
#endif
