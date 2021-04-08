#include "gridfastslam/gridslamprocessor.h"
#define EXPORT __declspec(dllexport)
#include "slam.hpp"
using namespace SLAM;
class SLAM::GProcessor: public GMapping::GridSlamProcessor  {};
class SLAM::GRangeSensor: public GMapping::RangeSensor {};




Slam::Slam(middleware::ConfigFile& cfg):
    first_scan_(true),
    gsp_laser_(NULL)
{
    gsp_ = (GProcessor*)new GMapping::GridSlamProcessor();
#undef __helper__
#define __helper__(x,y) \
    this->x##_ = cfg.value("slam", #x, y); \
    std::cout<< #x " set to "<<this->x##_<<std::endl;
    __helper__(sigma, 0.05);
    __helper__(kernelSize,1);
    __helper__(lstep,0.05);
    __helper__(astep,0.05);
    __helper__(iterations,5);
    __helper__(lsigma,0.075);
    __helper__(ogain,3.0);
    __helper__(lskip,0);

    __helper__(srr, 0.1);
    __helper__(srt, 0.2);
    __helper__(str, 0.1);
    __helper__(stt, 0.2);

    __helper__(linearUpdate, 0.5);
    __helper__(angularUpdate, 0.5);
    __helper__(temporalUpdate, -1.0);
    __helper__(resampleThreshold, 0.5);
    __helper__(particles, 30);

    __helper__(xmin, -25.0);
    __helper__(ymin, -25.0);
    __helper__(xmax, 25.0);
    __helper__(ymax, 25.0);
    __helper__(delta, 0.1);
    __helper__(occ_thresh, 0.25);
    __helper__(llsamplerange, 0.01);
    __helper__(llsamplestep, 0.01);
    __helper__(lasamplerange, 0.005);
    __helper__(lasamplestep, 0.005);
#undef __helper__
	log.open("slam_log_file.txt", std::ios::out | std::ios::app);

}
Slam::~Slam()
{
    if(gsp_laser_)
        delete gsp_laser_;
}


void Slam::initialize(const stdmsg::Laser_Scan& laser)
{
    int gsp_laser_beam_count_ = laser.ranges_size();//( laser.config().angle_max() - laser.config().angle_min() )/
    //laser.config().angle_increment() + 1.0;
    std::cout<<"laser_beam_count: "<<gsp_laser_beam_count_<<std::endl;
    double gsp_laser_angle_increment_ = laser.config().angle_increment();
	//crti:2016-03-24
    GMapping::OrientedPoint initialPose(laser.pose().position().x(),
                                        laser.pose().position().y(),
                                        laser.pose().orentation().yaw());
	/*GMapping::OrientedPoint initialPose(laser.robot().position().x(),
                                        laser.robot().position().y(),
                                        laser.robot().orentation().yaw());*/

    maxRange_ = laser.config().range_max() - 0.01;
    maxUrange_ = maxRange_;

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    GMapping::OrientedPoint gmap_pose(0, 0, 0);
    gsp_laser_ = (GRangeSensor*) new GMapping::RangeSensor("FLASER",gsp_laser_beam_count_,
                                           fabs(gsp_laser_angle_increment_),gmap_pose,0.0,maxRange_);
    //gsp_odom_ = new GMapping::OdometrySensor("ODOM");

    GMapping::SensorMap smap;
    smap.insert(std::make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);


    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                               kernelSize_, lstep_, astep_, iterations_,
                               lsigma_, ogain_, lskip_);

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    //  gsp.setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                 delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1,time(NULL));

    angle_min_ = laser.config().angle_min();
    angle_increment_ = laser.config().angle_increment();

    double dx = laser.robot().position().x() - laser.pose().position().x();
    double dy = laser.robot().position().y() - laser.pose().position().y();
    double dtheta = atan2(dy, dx) - laser.pose().orentation().yaw();
    double offset = sqrt(dx * dx + dy * dy);
    double dhead = laser.robot().orentation().yaw() - laser.pose().orentation().yaw();

    // the robot pose in the fixed frame of laser
    robot_offset_t = dhead;
    robot_offset_x = offset * cos(dtheta);
    robot_offset_y = offset * sin(dtheta);
    //fprintf(stdout, "robot offset is (%f %f %f)", robot_offset_x, robot_offset_y, robot_offset_t);
}

void Slam::update(const stdmsg::Laser_Scan& laser)
{
    if(first_scan_)
    {
        initialize(laser);
        first_scan_ = false;
        return;
    }

    std::cerr<<" "<<laser.seq();

    this->lock.lock();
    this->_scans.push_back(laser);

    std::vector<double> ranges_double;
    ranges_double.resize(laser.ranges_size());

    for(int i = 0; i < ranges_double.size(); ++i)
        ranges_double[i] = laser.ranges(i);
	////crti:2016-03-24
 //   GMapping::OrientedPoint laser_pose(laser.pose().position().x(),
 //                                      laser.pose().position().y(),
 //                                      laser.pose().orentation().yaw());
	//crti:2016-03-27,还没试,感觉不对
	GMapping::OrientedPoint laser_pose(laser.pose().position().x(),
                                       laser.pose().position().y(),
                                       laser.pose().orentation().yaw());
	GMapping::OrientedPoint robot_pose(laser.robot().position().x(),
                                       laser.robot().position().y(),
                                       laser.robot().orentation().yaw());


    GMapping::RangeReading reading(ranges_double.size(),
                                   &ranges_double[0],
            gsp_laser_, laser.seq()/*,laser.time()*/);
    //crti:2016-03-24
	//reading.setPose(laser_pose);
	reading.setPose(laser_pose);


    /** force update when a landmark found */
    if(this->current_landmark != "" )
    {
        std::cerr<< "find landmark:"
                 << this->current_landmark
                 <<", record it!"<<std::endl;

        this->landmark_name.push_back(this->current_landmark);
        this->landmark_seq.push_back( laser.seq() );
        this->current_landmark = "";
        this->gsp_->forceUpdate();
    }
    this->lock.unlock();

    bool processed = gsp_->processScan(reading);
}

/*
    void update_map()
    {
        const GMapping::GridSlamProcessor::Particle& best = gsp_->getParticles()[gsp_->getBestParticleIndex()];
        GMapping::ScanMatcherMap map = best.map;

        for(int x = 0; x < map.getMapSizeX(); x++)
        {
            for(int y = 0; y < map.getMapSizeY(); y++)
            {
                GMapping::IntPoint p(x,y);
                double occ = map.cell(p);
            }
        }

    }
*/


void Slam::generate_landmark(const std::string& filename, double origin_x, double origin_y)
{
    std::stack<_Pose> poses;
    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];
    int landmark_index = landmark_seq.size() - 1;
    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n && landmark_index>=0;
        n = n->parent)
    {
        //if(n->reading)
        //	fprintf(fp, " %d:%d", n->reading->getTime(),landmark_seq[landmark_index]);
        if(n->reading && n->reading->getTime() == landmark_seq[landmark_index] )
        {
            _Pose pos(
                        n->pose.x - xmin_,
                        n->pose.y - ymin_,
                        n->pose.theta,
                        landmark_seq[landmark_index]);
			
			this->log << pos.x << " " << pos.y << " " << pos.theta << std::endl;
			this->log << "robot_offset_x: " << robot_offset_x;
			this->log << "robot_offset_y: " << robot_offset_y;
			this->log << "robot_offset_t: " << robot_offset_t;
			//translate it to robot pose
            pos.x = pos.x + robot_offset_x * cos(pos.theta ) - robot_offset_y * sin(pos.theta);
            pos.y = pos.y + robot_offset_x * sin(pos.theta ) + robot_offset_y * cos(pos.theta);
            pos.theta = pos.theta + robot_offset_t;

			// addapt to new map size
			pos.x -= origin_x;
			pos.y -= origin_y;

            poses.push( pos );
            landmark_index --;
        }
    }

    FILE *fp = fopen(filename.c_str(), "w");
    while( !poses.empty() && fp)
    {
        _Pose& pos = poses.top();
        fprintf(fp, "%f %f %f\n", pos.x, pos.y, pos.theta);
        poses.pop();
    }
    if(fp)
        fclose(fp);
}

void Slam::generate_map(const std::string& filename,const std::string& filename_nav)
{
    this->lock.lock();
    GMapping::ScanMatcher matcher;
    double* laser_angles = new double[gsp_laser_->beams().size()];
    for(int i = 0; i < gsp_laser_->beams().size() ; i++)
        laser_angles[i] = angle_min_ + i * angle_increment_;

    matcher.setLaserParameters(gsp_laser_->beams().size(), laser_angles,
                               gsp_laser_->getPose());

    delete[] laser_angles;
    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];


    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);


    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n;
        n = n->parent)
    {
        if(!n->reading)
        {
            continue;
        }
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;

	int minX, maxX, minY, maxY;
	minX = smap.getMapSizeX();
	maxX = 0;
	minY = smap.getMapSizeY();
	maxY = 0;
    Map img(smap.getMapSizeX(), smap.getMapSizeY(), delta_);
    for(int x = 0; x < smap.getMapSizeX(); x++)
    {
        for(int y = 0; y < smap.getMapSizeY(); y++)
        {
            GMapping::IntPoint p(x,y);
            double occ = smap.cell(p);
            if(occ > 1)
                occ = 1;
            if(occ>=-0.001)
            {
                unsigned int v =(1.0 - occ)* 0xFF;
                img(x,y) = 0xFF000000 + (v<<16) + (v<<8) + v ;
				if(x<minX) minX=x;
				if(x>maxX) maxX=x;
				if(y<minY) minY=y;
				if(y>maxY) maxY=y;
            }
            else
                img(x,y) = Map::UNKOWN;//m.at<unsigned char> (x,y,0) =  255;
        }
    }

	//crti:2016-12-22, 动态保存地图，按照实际大小，把周围空的地方去除，保留borderMeter大小的边界，navmap为空
	int border, borderMeter;
	borderMeter = 3;
	border = (int)(borderMeter/delta_);
	minX = ((minX-border) < 0) ? 0 : (minX-border);
	maxX = ((maxX+border) > smap.getMapSizeX()) ? smap.getMapSizeX() : (maxX+border);
	minY = ((minY-border) < 0) ? 0 : (minY-border);
	maxY = ((maxY+border) > smap.getMapSizeY()) ? smap.getMapSizeY() : (maxY+border);
	std::cout<<"minX="<<minX<<" maxX="<<maxX<<" minY="<<minY<<" maxY="<<maxY<<std::endl;

	Map locImg(maxX-minX,maxY-minY,delta_);
	Map navImg(maxX-minX,maxY-minY,delta_);

	for(int x = 0; x < maxX-minX; ++x){
		for(int y = 0; y < maxY-minY; ++y){
			locImg(x,y) = img(x+minX, y+minY);
		}
	}
	

	std::cout<<"save map: "<< maxX-minX <<" * "<< maxY-minY <<std::endl;
    locImg.save(filename);
	navImg.save(filename_nav);
	//img.save(filename);
	//Map img_(smap.getMapSizeX(), smap.getMapSizeY(), delta_);
	//img_.save(filename_nav);
    //generate_landmark(filename + ".txt");
	generate_landmark("landmarks.txt", minX * delta_, minY * delta_);
    this->lock.unlock();
}

void Slam::maker(const std::string &name)
{
    this->lock.lock();
    this->current_landmark = name;
    this->lock.unlock();
}
stdmsg::LaserList Slam::scans(bool all)
{
    this->lock.lock();
    stdmsg::LaserList ret;
    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];

    int index = this->_scans.size();
    for(GMapping::GridSlamProcessor::TNode* n = best.node;
        n;
        n = n->parent)
    {
        if(n->reading  )
        {
            double x = n->pose.x;
            double y = n->pose.y;
            double t = n->pose.theta;
            //translate it to robot pose

            if(!all)
            {
                /** not all the scan is required */
                while (n->reading->getTime() != this->_scans[index--].seq() && index >= 0);
                if(index < 0)
                    return ret;
                /** the scan should not be altered, because of the raw information is useful for the interpolation */
                stdmsg::Laser_Scan scan = this->_scans[index];
                scan.mutable_pose()->mutable_position()->set_x(x);
                scan.mutable_pose()->mutable_position()->set_y(y);
                scan.mutable_pose()->mutable_orentation()->set_yaw(t);
                *ret.add_scans() = scan;
            }else
            {
                /** all the scan is required, but some of them is not in the slam method!
                  * so, the interpolation is perfomed */
                while (n->reading->getTime() != this->_scans[index].seq() && index >= 0)
                {
                    /** the scan should not be altered, because of the raw information is useful for the interpolation */
                    stdmsg::Laser_Scan scan = this->_scans[index];
                    scan.mutable_pose()->mutable_position()->set_x(x);
                    scan.mutable_pose()->mutable_position()->set_y(y);
                    scan.mutable_pose()->mutable_orentation()->set_yaw(t);
                    index --;
                    *ret.add_scans() = scan;
                }
            }
        }
    }
    this->lock.unlock();
    return ret;
}

class SlamNonBlock: public Slam
{
private:
    struct _Update_Thread:public BThread
    {
        BMutex _lock;// multithread read/write variable plan
        BCond cond;
        Slam* handle;
        std::queue<stdmsg::Laser_Scan> scans;
        unsigned int processed;
        volatile bool is_waitiing;
        _Update_Thread(Slam* p)
        {
            handle = p;
            this->is_waitiing = false;
            this->processed = 0;
        }
        void run()
        {
            while(true)
            {
                this->_lock.lock();
                size_t size = this->scans.size();
                this->_lock.unlock();
                if(size <= this->processed)
                {
                    this->is_waitiing = true;
                    this->cond.wait();
                    is_waitiing = false;
                }

                this->_lock.lock();
                stdmsg::Laser_Scan scan(this->scans.front());
                this->scans.pop();
                this->processed ++ ;
                this->_lock.unlock();

                handle->update(scan);
                _sleep(1);//crti:2016-05-28,降低cpu使用率
            }
        }
    } update_thread;
public:
    SlamNonBlock(middleware::ConfigFile& cfg):
        update_thread(this), Slam(cfg)
    {
        update_thread.start();
    }

    void update(const stdmsg::Laser_Scan& laser)
    {
        update_thread._lock.lock();
        update_thread.scans.push(laser);
        if (update_thread.is_waitiing)
            update_thread.cond.wakeup();
        std::cout<<"scan processed: "
                << update_thread.processed << " of "
                <<update_thread.scans.size() << std::endl;
        update_thread._lock.unlock();
    }
};

/*
int main(int argc, char **argv)
{ 
    if(argc == 1)
    {
        std::cout<< "usage:\n" "\t" "-cfg cfgfile. "
                 << "defalut read 'navigator.ini' file in current path"
                 << std::endl
                 << "\t" "-s. " <<"not open the hardware, but the rpc can be called"<<std::endl;
    }

    middleware::ConfigFile cfg;
    std::string cfgfile("navigator.ini");
    for (int i = 0; i < argc - 1; i++)
        if( strcmp(argv[i],"-cfg") == 0 )
            cfgfile = argv[i+1];
    cfg.read(cfgfile);

    Slam slam(cfg);

    middleware::Node nh("tcp://*:6002");
    nh.connect("tcp://localhost:6000");

    nh.subscrible("laser", &Slam::on_laser, &slam);
    try
    {
        while (1)
            nh.run();
    }catch(...)
    {
        std::cout<<"exit"<<std::endl;
    }
}
*/
