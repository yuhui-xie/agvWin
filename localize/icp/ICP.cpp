#define EXPORT __declspec(dllexport)
#include <iostream>
#include "ICP.hpp"
#include "libICP/icpPointToPoint.h"
using namespace icp;
ICP::ICP ()
    : icp (NULL), outlier (0.2)
{
    rotation = new Matrix(2, 2);
    for (int32_t i=0; i<2; i++)
      rotation->val[i][i] = 1;
    translate = new Matrix(2, 1);
}
ICP::~ ICP ()
{
    if(icp)
        delete icp;
    if(rotation)
        delete rotation;
    if(translate)
        delete translate;
}
void ICP::set_outlier (double r)
{
    outlier = r;
}
void ICP::set_model (std::vector <ICP_Point> model)
{
    if(icp)
        delete icp;
    icp = new IcpPointToPoint(&model[0].x, model.size(), 2);

    return;
}
void ICP::initial_pose (double x, double y, double theta)
{

    translate->val[0][0] = x;
    translate->val[1][0] = y;
    double c = cos(theta);
    double s = sin(theta);
    rotation->val[0][0] = c;
    rotation->val[0][1] = -s;
    rotation->val[1][1] = c;
    rotation->val[1][0] = s;
}
ICP_Pose ICP::pose ()
{
    double rx = translate->val[0][0];

    double ry = translate->val[1][0];
    double rtheta = atan2(-rotation->val[0][1], rotation->val[0][0]);
    return ICP_Pose(rx,ry,rtheta);
}
ICP_Pose ICP::match (std::vector <ICP_Point> d)
{
    if(icp == NULL)
        std::cerr << "icp not initialized" << std::endl;
    icp->setSubsamplingStep(1);
    icp->setMinDeltaParam(1e-5);
    icp->setMaxIterations(60);
    icp->fit(&d[0].x, d.size(), *rotation, *translate, outlier);

    /*double rx = x * rotation.val[0][0] + y * rotation.val[0][1]
                + translate.val[0][0];

        double ry = x * rotation.val[1][0] + y * rotation.val[1][1]
                + translate.val[1][0];
        double rtheta = theta + atan2(-rotation.val[0][1], rotation.val[0][0]);*/

    double rx = translate->val[0][0];
    double ry = translate->val[1][0];
    double rtheta = atan2(-rotation->val[0][1], rotation->val[0][0]);

    return ICP_Pose(rx,ry,rtheta);
}
