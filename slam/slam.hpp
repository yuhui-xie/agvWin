#ifndef SLAM_HPP
#define SLAM_HPP
#include <time.h>
#include <stdmsg.hh>
#include <image.h>
#include <stack>
#include <queue>
#include <vector>
#include <node.hpp>
#include <thread.hpp>
#include <fstream>
namespace SLAM
{
class GProcessor;
class GRangeSensor ;

#ifdef WIN32
#ifndef EXPORT
#define EXPORT __declspec(dllimport)
#endif
#else
#define EXPORT
#endif
/*
 Parameters used by GMapping itself:

Laser Parameters:
- @b "~/maxRange" @b [double] maximum range of the laser scans. Rays beyond this range get discarded completely. (default: maximum laser range minus 1 cm, as received in the the first LaserScan message)
- @b "~/maxUrange" @b [double] maximum range of the laser scanner that is used for map building (default: same as maxRange)
- @b "~/sigma" @b [double] standard deviation for the scan matching process (cell)
- @b "~/kernelSize" @b [double] search window for the scan matching process
- @b "~/lstep" @b [double] initial search step for scan matching (linear)
- @b "~/astep" @b [double] initial search step for scan matching (angular)
- @b "~/iterations" @b [double] number of refinement steps in the scan matching. The final "precision" for the match is lstep*2^(-iterations) or astep*2^(-iterations), respectively.
- @b "~/lsigma" @b [double] standard deviation for the scan matching process (single laser beam)
- @b "~/ogain" @b [double] gain for smoothing the likelihood
- @b "~/lskip" @b [int] take only every (n+1)th laser ray for computing a match (0 = take all rays)

Motion Model Parameters (all standard deviations of a gaussian noise model)
- @b "~/srr" @b [double] linear noise component (x and y)
- @b "~/stt" @b [double] angular noise component (theta)
- @b "~/srt" @b [double] linear -> angular noise component
- @b "~/str" @b [double] angular -> linear noise component

Others:
- @b "~/linearUpdate" @b [double] the robot only processes new measurements if the robot has moved at least this many meters
- @b "~/angularUpdate" @b [double] the robot only processes new measurements if the robot has turned at least this many rads

- @b "~/resampleThreshold" @b [double] threshold at which the particles get resampled. Higher means more frequent resampling.
- @b "~/particles" @b [int] (fixed) number of particles. Each particle represents a possible trajectory that the robot has traveled

Likelihood sampling (used in scan matching)
- @b "~/llsamplerange" @b [double] linear range
- @b "~/lasamplerange" @b [double] linear step size
- @b "~/llsamplestep" @b [double] linear range
- @b "~/lasamplestep" @b [double] angular setp size

Initial map dimensions and resolution:
- @b "~/xmin" @b [double] minimum x position in the map [m]
- @b "~/ymin" @b [double] minimum y position in the map [m]
- @b "~/xmax" @b [double] maximum x position in the map [m]
- @b "~/ymax" @b [double] maximum y position in the map [m]
- @b "~/delta" @b [double] size of one pixel [m]
 */

class EXPORT Slam
{
public:
    Slam(middleware::ConfigFile& cfg);
    ~Slam();

    void initialize(const stdmsg::Laser_Scan& laser);
    void update(const stdmsg::Laser_Scan& laser);

    struct _Pose
	{
        inline _Pose(double _x=0, double _y=0, double _theta=0, int _index =0)
			:x(_x), y(_y), theta(_theta), index(_index)
		{}
		double x;
		double y;
		double theta;
		int index;
        std::string name;
	};
    void generate_landmark(const std::string& filename, double origin_x, double origin_y);
    void generate_map(const std::string& filename,const std::string& filename_nav);
    void maker(const std::string& name);
    stdmsg::LaserList scans(bool all = false);

private:

	std::ofstream log;

    bool first_scan_;
    GProcessor* gsp_;
    GRangeSensor *gsp_laser_;

    BMutex lock;
    std::vector<stdmsg::Laser_Scan> _scans;
	std::vector<int> landmark_seq;
    std::vector<std::string> landmark_name;
    std::string current_landmark;

    // Parameters used by GMapping
    double maxUrange_;
    double maxRange_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;

	double angle_min_;
	double angle_increment_;

	//specify the laser pose in the robot frame
	double robot_offset_x;
	double robot_offset_y;
	double robot_offset_t;
}; 
}

#endif
