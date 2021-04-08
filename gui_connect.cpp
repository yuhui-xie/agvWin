#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <image.h>
#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>
#include <queue>
#include <list>

#include <windows.h>
#include <stdio.h>

using namespace std;

class GuiConnect
{
protected:
	middleware::Node *nh_slam;
    middleware::RPC rpc_slam;
	middleware::RPC rpc_slam2;
	middleware::Node *nh_gui;
    middleware::RPC rpc_gui;
	middleware::RPC rpc_gui2;
	middleware::ConfigFile cfg;
private:
	struct _Rpc_Slam_Thread2 :public BThread
	{
		GuiConnect* handle;
		_Rpc_Slam_Thread2(GuiConnect* p)
		{
			handle = p;
		}
		~_Rpc_Slam_Thread2()
		{
			kill();
		}
		void run()
		{
			try{
				while (true)
				{
					handle->rpc_slam2.run();
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "rpc_slam_thread communication error: " << e.what() << std::endl;
			}
		}
	} rpc_slam2_thread;

	struct _Rpc_Slam_Thread:public BThread
    {
        GuiConnect* handle;
        _Rpc_Slam_Thread(GuiConnect* p)
        {
            handle = p;
        }
        ~_Rpc_Slam_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
                    handle->rpc_slam.run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"rpc_slam_thread communication error: "<< e.what() <<std::endl;
            }
        }
    } rpc_slam_thread;

    struct _Node_Slam_Thread:public BThread
    {
        GuiConnect* handle;
        _Node_Slam_Thread(GuiConnect* p)
        {
            handle = p;
        }
        ~_Node_Slam_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
                    handle->nh_slam->run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"nh_slam_thread communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_slam_thread;

	struct _Rpc_Gui_Thread:public BThread
    {
        GuiConnect* handle;
        _Rpc_Gui_Thread(GuiConnect* p)
        {
            handle = p;
        }
        ~_Rpc_Gui_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
                    handle->rpc_gui.run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"rpc_gui_thread communication error: "<< e.what() <<std::endl;
            }
        }
    } rpc_gui_thread;

	struct _Rpc_Gui_Thread2 :public BThread
	{
		GuiConnect* handle;
		_Rpc_Gui_Thread2(GuiConnect* p)
		{
			handle = p;
		}
		~_Rpc_Gui_Thread2()
		{
			kill();
		}
		void run()
		{
			try{
				while (true)
				{
					handle->rpc_gui2.run();
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "rpc_gui_thread communication error: " << e.what() << std::endl;
			}
		}
	} rpc_gui2_thread;

    struct _Node_Gui_Thread:public BThread
    {
        GuiConnect* handle;
        _Node_Gui_Thread(GuiConnect* p)
        {
            handle = p;
        }
        ~_Node_Gui_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
                    handle->nh_gui->run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"nh_gui_thread communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_gui_thread;

	struct _Path_Thread:public BThread
    {
        GuiConnect* handle;
        _Path_Thread(GuiConnect* p)
        {
            handle = p;
        }
        ~_Path_Thread()
        {
            kill();
        }
        void run()
        {
            try{
				handle->run_path();
            }
            catch(const std::exception& e)
            {
                std::cerr<<"path_thread communication error: "<< e.what() <<std::endl;
            }
        }
    } path_thread;

	//struct _Demo_Thread:public BThread
 //   {
 //       GuiConnect* handle;
 //       _Demo_Thread(GuiConnect* p)
 //       {
 //           handle = p;
 //       }
 //       ~_Demo_Thread()
 //       {
 //           kill();
 //       }
 //       void run()
 //       {
 //           try{
 //               while(true)
 //               {
 //                   handle->demo();
 //               }
 //           }
 //           catch(const std::exception& e)
 //           {
 //               std::cerr<<"demo error: "<< e.what() <<std::endl;
 //           }
 //       }
 //   } demo_thread;
	//stdmsg::Pose demoPoints[8];

	struct GOAL
	{
		double x;
		double y;
		double theta;
	};
	struct PATH
	{
		int goalID;
		int type;//0,目标;1,路过;
	};

	vector<GOAL> goalList;
	vector<PATH> pathList;
	
	STARTUPINFO startInfo;
	STARTUPINFO startInfo2;
	STARTUPINFO startInfo3;
    PROCESS_INFORMATION mappingProcessInfo;
	PROCESS_INFORMATION localizerProcessInfo;
	PROCESS_INFORMATION navigatorProcessInfo;

	int nav_mode;//0,stop; 1,goal; 2,path;
	int goalReach;
	int distanceGoalReach;

public:
    GuiConnect(int argc, char** argv) :
	    rpc_slam_thread(this),
		rpc_slam2_thread(this),
		nh_slam_thread(this),
		rpc_gui_thread(this),
		rpc_gui2_thread(this),
		nh_gui_thread(this),
		path_thread(this),
		nav_mode(0),goalReach(0),distanceGoalReach(0)
		//demo_thread(this)
    {
		std::string cfgfile("navigator.ini");
		for (int i = 0; i < argc - 1; i++)
			if( strcmp(argv[i],"-cfg") == 0 )
				cfgfile = argv[i+1];
		cfg.read(cfgfile);

		//
		std::string tmp = cfg.value("gui_connect", "nh_slam_bind", "tcp://127.0.0.1:9020");
		std::cout<<"[gui_connect] nh_slam_bind = "<< tmp <<std::endl;
		nh_slam = new middleware::Node(tmp);
		
		tmp = std::string( cfg.value("gui_connect", "nh_slam_connect", "tcp://127.0.0.1:9001;tcp://127.0.0.1:9002") );
		std::cout<<"[gui_connect] nh_slam_connect = " ;
		while( tmp != "" )
		{
			int pos = tmp.find(";");
			std::cout<<tmp.substr(0, pos)<<"; ";
			nh_slam->connect(tmp.substr(0, pos));
			if (pos != std::string::npos)
				tmp = tmp.substr(pos + 1 );
			else
				tmp = "";
		}
		std::cout<<std::endl;

		tmp = std::string( cfg.value("gui_connect", "rpc_slam_connect", "tcp://127.0.0.1:9000") );
        rpc_slam.connect(tmp);
        std::cout<<"[gui_connect] rpc_slam_connect = "<<tmp<<";"<<std::endl;

		tmp = std::string(cfg.value("gui_connect", "rpc_slam_connect", "tcp://127.0.0.1:9000"));
		rpc_slam2.connect(tmp);
		std::cout << "[gui_connect] rpc_slam_connect = " << tmp << ";" << std::endl;

		//
		tmp = cfg.value("gui_connect", "nh_gui_bind", "tcp://127.0.0.1:9021");
		std::cout<<"[gui_connect] nh_gui_bind = "<< tmp <<std::endl;
		nh_gui = new middleware::Node(tmp);

		//tmp = std::string( cfg.value("gui_connect", "rpc_gui_connect", "tcp://127.0.0.1:9022") );
		//tmp = std::string(cfg.value("gui_connect", "rpc_gui_connect3", "tcp://127.0.0.1:9024"));
		//rpc_gui.bind(tmp);
        //rpc_gui.connect(tmp);
        //std::cout<<"[gui_connect] rpc_gui_connect = "<<tmp<<";"<<std::endl;

		//tmp = std::string(cfg.value("gui_connect", "rpc_gui_connect2", "tcp://127.0.0.1:9023"));
		//rpc_gui2.bind(tmp);
		//rpc_gui2.connect(tmp);
		//std::cout << "[gui_connect] rpc_gui2_connect = " << tmp << ";" << std::endl;
		
		//hxw.5.10
		//double a2,b2,c2;
		//a2 = double(cfg.value("auto_recharge", "a2", "1.1"));
		//b2 = double(cfg.value("auto_recharge", "b2", "2.2"));
		//c2 = double(cfg.value("auto_recharge", "c2", "3.3"));
		//stdmsg::Pose pos2;
		//pos2.mutable_position()->set_x(a2);
		//pos2.mutable_position()->set_y(b2);
		//pos2.mutable_orentation()->set_yaw(c2);

		//std::cout << "hxw24";
		//rpc_slam.call<stdmsg::Pose, stdmsg::Pose>("initial_pose", pos2);
		//Sleep(100);
		//rpc_slam.call<stdmsg::Pose, stdmsg::Pose>("initial_pose", pos2);
		//


        nh_slam->subscrible("laser", &GuiConnect::slam_scan_handler, this);
		nh_slam->subscrible("global_plan", &GuiConnect::slam_plan_handler, this);
        nh_slam->subscrible("battery", &GuiConnect::slam_battery_handler, this);

		//
		rpc_gui2.set("initial_pose2", &GuiConnect::initial_pose2, this);
		rpc_gui.set("initial_pose", &GuiConnect::initial_pose, this);		
        rpc_gui.set("set_goal", &GuiConnect::set_goal, this);
		rpc_gui.set("set_change", &GuiConnect::set_change, this);
		rpc_gui.set("set_goal_list", &GuiConnect::set_goal_list, this);
        rpc_gui.set("map", &GuiConnect::map, this);
		rpc_gui.set("mapnav", &GuiConnect::mapnav, this);
		rpc_gui.set("set_nav_map", &GuiConnect::set_nav_map, this);
		rpc_gui.set("set", &GuiConnect::set, this);
		rpc_gui.set("set_nav_config", &GuiConnect::set_nav_config, this);

		//HXW:3.14
		//rpc_gui.set("start_guiconnect", &GuiConnect::start_guiconnect, this);
		rpc_gui.set("start_navigation", &GuiConnect::start_navigation, this);
		rpc_gui.set("finish_navigation", &GuiConnect::finish_navigation, this);

		rpc_gui.set("start_explore", &GuiConnect::start_explore, this);
		rpc_gui.set("finish_explore", &GuiConnect::finish_explore, this);//stdmsg::String& name
		rpc_gui.set("start_localize", &GuiConnect::start_localize, this);
		rpc_gui.set("finish_localize", &GuiConnect::finish_localize, this);
		rpc_gui.set("cancel_explore", &GuiConnect::cancel_explore, this);

		rpc_gui.set("set_path", &GuiConnect::set_path, this);
		rpc_gui.set("set_initial", &GuiConnect::set_initial, this);
		rpc_gui.set("set_move_mode", &GuiConnect::set_move_mode, this);
		rpc_gui.set("set_vel", &GuiConnect::set_vel, this);
		rpc_gui.set("set_path2", &GuiConnect::set_path2, this);

		rpc_gui.set("send_goal_file", &GuiConnect::send_path_file, this);
		rpc_gui.set("send_path_file", &GuiConnect::send_path_file, this);

		//stdmsg::Pose pos2;
		//pos2.mutable_position()->set_x(0);
		//pos2.mutable_position()->set_y(0);
		//pos2.mutable_orentation()->set_yaw(0);
		this->nh_slam_thread.start();
		this->nh_gui_thread.start();
		this->rpc_slam_thread.start();
		this->rpc_slam2_thread.start();
		this->rpc_gui_thread.start();
		this->rpc_gui2_thread.start();

		//demoPoints[0] = setPose(0.0, 0.0, 0.0);
		//demoPoints[1] = setPose(0.0, 0.0, 0.0);
		//demoPoints[2] = setPose(0.0, 0.0, 0.0);
		//demoPoints[3] = setPose(0.0, 0.0, 0.0);
		//demoPoints[4] = setPose(0.0, 0.0, 0.0);
		//demoPoints[5] = setPose(0.0, 0.0, 0.0);
		//demoPoints[6] = setPose(0.0, 0.0, 0.0);
		//demoPoints[7] = setPose(0.0, 0.0, 0.0);
    }

	//stdmsg::Pose setPose(double x, double y, double t){
	//	stdmsg::Pose tmp;
	//	tmp.mutable_position()->set_x(x);
	//	tmp.mutable_position()->set_y(y);
	//	tmp.mutable_orentation()->set_yaw(t);
	//	return tmp;
	//}
 //   void demo(){

	//}

	void slam_scan_handler(const stdmsg::Laser_Scan & scan)
    {
		nh_gui->publish("laser", scan);		
        return;
    }
	void slam_plan_handler(const stdmsg::Global_Plan & plan)
    {
		this->goalReach = plan.goal_reached();
		//!3.19
		this->distanceGoalReach = plan.distance_goal_reached();
		nh_gui->publish("global_plan", plan);
        return;
    }
    void slam_battery_handler(const stdmsg::SomeFlag & battrey_info)
    {
        nh_gui->publish("battery", battrey_info);
		//std::cout << "nihao"<<endl;
        return;
    }

	stdmsg::Pose initial_pose(const stdmsg::Pose& pos)
    {
        stdmsg::Pose tmp;
		tmp = rpc_slam.call<stdmsg::Pose,stdmsg::Pose>("initial_pose",pos);
		std::cerr<<"initial_pose"<<std::endl;
        return tmp;
    }
	stdmsg::Pose initial_pose2(const stdmsg::Pose& pos)
	{
		stdmsg::Pose tmp;
		tmp = rpc_slam2.call<stdmsg::Pose, stdmsg::Pose>("initial_pose", pos);
		std::cerr << "initial_pose" << std::endl;
		return tmp;
	}
	stdmsg::Pose set_goal_list(const stdmsg::Pose& pos)
    {
        stdmsg::Pose tmp;
		tmp = rpc_slam.call<stdmsg::Pose,stdmsg::Pose>("set_goal",pos);
		std::cerr<<"loc set_goal"<<std::endl;
        return tmp;
    }
	//hxw.3.24 将初始位姿写入一个文件
	stdmsg::Pose set_change(const stdmsg::Pose& pos)
	{
		//this->nav_mode = 1;//goal
		//this->path_thread.kill();


		//stdmsg::Pose tmp;
		//tmp = rpc_slam.call<stdmsg::Pose, stdmsg::Pose>("set_goal", pos);
		//std::cerr << "loc set_goal" << std::endl;
		//return tmp;
		//cout<<"xytheta"<<pos.x<<pos.y<<pos.theta<<endl;
		std::string pathFileName = std::string(cfg.value("gui_connect", "path", "sendinitial.txt"));
		printf("recv path file: %s\n", pathFileName.c_str());
		stdmsg::Pose ret;
		FILE* fp = fopen(pathFileName.c_str(), "wb");
		if (fp == NULL)
		{
			fclose(fp);
			return ret;
		}
		int s = 156;

		//int len = sizeof(pos);
		int len = sizeof(stdmsg::Pose);
		//int tmp = 
			fwrite(&pos, len, 1, fp);
		//int tmp = fwrite(&pos, len, 1, fp);
		fclose(fp);

		//this->nav_mode = 2;//path
		//this->path_thread.start();

		//return ret;
		return ret;
	}
	//hxw 5.10


	stdmsg::Pose set(const stdmsg::Pose& pos)
	{
		fstream pathFile;
		//std::string pathFileName = std::string(cfg.value("gui_connect", "path", "initial2.txt"));
		pathFile.open("initial2.txt", ios::in);
		if (!pathFile.good()){
			std::cout << "[PATH] read PATH file failed.";
			//return ;
		}
		
		string line;
		GOAL xyz;
		while (getline(pathFile, line)) {

			int i = 0;
			string goalInfo2;
			istringstream lineStream2(line);
			while (getline(lineStream2, goalInfo2, ','))
			{
				switch (i)
				{
				case 0:
					xyz.x = atof(goalInfo2.c_str());
					break;
				case 1:
					xyz.y = atof(goalInfo2.c_str());
					break;
				case 2:
					xyz.theta = atof(goalInfo2.c_str());
					break;

				}
				++i;
			}
			std::cout << "hxw" << xyz.x << xyz.y << xyz.theta << endl;

		}
		stdmsg::Pose pos2;
		pos2.mutable_position()->set_x(xyz.x);
		pos2.mutable_position()->set_y(xyz.y);
		pos2.mutable_orentation()->set_yaw(xyz.theta);

		
		rpc_slam.call<stdmsg::Pose, stdmsg::Pose>("initial_pose", pos2);
		Sleep(100);
		rpc_slam.call<stdmsg::Pose, stdmsg::Pose>("initial_pose", pos2);
		return pos2;
		pathFile.close();
	}
	//9000
	stdmsg::Pose set_goal(const stdmsg::Pose& pos)
    {
		this->nav_mode = 1;//goal
		this->path_thread.kill();

        stdmsg::Pose tmp;
		tmp = rpc_slam.call<stdmsg::Pose,stdmsg::Pose>("set_goal",pos);
		std::cerr<<"loc set_goal"<<std::endl;
        return tmp;
    }
	stdmsg::NavConfig set_nav_config(const stdmsg::NavConfig& pos)
	{
		nh_slam->publish("set_nav_config", pos);
		std::cout << "receive 1" << std::endl;
		return pos;
	}
	stdmsg::Data map(const stdmsg::Data& input)
    {
        std::string mapname = std::string(cfg.value("map", "localize ", "map.png"));
		printf("recv map requeset: %s\n", mapname.c_str());
        stdmsg::Data ret;
        FILE* fp = fopen(mapname.c_str(), "rb");
        if(fp == NULL)
        {
            fclose(fp);
            return ret;
        }
        fseek(fp, 0L, SEEK_END);
        int len = ftell(fp);
        void* data = new char[len];
        fseek(fp,0L, SEEK_SET);
        fread(data, len, 1, fp);
        fclose(fp);
        ret.set_data((char*)data, len);
        printf("send the map to the client, length : %d\n", len);
        return ret;
    }
	stdmsg::Data mapnav(const stdmsg::Data& input)
    {
        std::string mapname = std::string(cfg.value("map", "planer", "mapnav.png"));
		printf("recv mapnav requeset: %s\n", mapname.c_str());
        stdmsg::Data ret;
        FILE* fp = fopen(mapname.c_str(), "rb");
        if(fp == NULL)
        {
            fclose(fp);
            return ret;
        }
        fseek(fp, 0L, SEEK_END);
        int len = ftell(fp);
        void* data = new char[len];
        fseek(fp,0L, SEEK_SET);
        fread(data, len, 1, fp);
        fclose(fp);
        ret.set_data((char*)data, len);
        printf("send the mapnav to the client, length : %d\n", len);
        return ret;
    }
	stdmsg::Data set_nav_map(const stdmsg::Data& input)
    {
		std::string mapname = std::string(cfg.value("map", "planer", "mapnav.png"));
		printf("recv navmap: %s\n", mapname.c_str());
		stdmsg::Data ret;
        FILE* fp = fopen(mapname.c_str(), "wb");
        if(fp == NULL)
        {
            fclose(fp);
            return ret;
        }
		int len = input.data().size();

		int tmp = fwrite(input.data().c_str(), len, 1, fp);
        fclose(fp);

        printf("recv navmap, length : %d\n", len);

		stdmsg::Pose pos;
		pos.mutable_position()->set_x(0.0);
		pos.mutable_position()->set_y(0.0);
		pos.mutable_orentation()->set_yaw(0.0);
		nh_slam->publish("reset_nav", pos);
        return ret;
    }

	//HXW:3.14
	/*
	stdmsg::String start_guiconnect(const stdmsg::String& str_)
	{
	stdmsg::String tmp;

	memset(&startInfo4, 0, sizeof(STARTUPINFO));
	startInfo4.cb = sizeof(STARTUPINFO);
	startInfo4.dwFlags = STARTF_USESHOWWINDOW;
	startInfo4.wShowWindow = SW_SHOW;
	CreateProcess(NULL, "C:/Users/DL/Desktop/tianwei/bin2/gui_connect.exe", NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &startInfo4, &guiconnectProcessInfo);

	return str_;
	}
	*/
	stdmsg::String start_localize(const stdmsg::String& str_)
	{
		stdmsg::String tmp;

		//开启定位程序
		memset(&startInfo2, 0, sizeof(STARTUPINFO));
		startInfo2.cb = sizeof(STARTUPINFO);
		startInfo2.dwFlags = STARTF_USESHOWWINDOW;
		startInfo2.wShowWindow = SW_SHOW;
		CreateProcess(NULL, "localizer.exe", NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &startInfo2, &localizerProcessInfo);
		//开启导航程序
		//Sleep(3000);
		//memset(&startInfo3, 0, sizeof(STARTUPINFO));
		//startInfo3.cb = sizeof(STARTUPINFO);
		//startInfo3.dwFlags = STARTF_USESHOWWINDOW;
		//startInfo3.wShowWindow = SW_SHOW;
		//CreateProcess(NULL, "navigator.exe", NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &startInfo3, &navigatorProcessInfo);

		return str_;
	}
	stdmsg::String start_navigation(const stdmsg::String& str_)
	{
		stdmsg::String tmp;
		
		//开启定位程序
		//memset(&startInfo2, 0, sizeof(STARTUPINFO));
		//startInfo2.cb = sizeof(STARTUPINFO);
		//startInfo2.dwFlags = STARTF_USESHOWWINDOW;
		//startInfo2.wShowWindow = SW_SHOW;
		//CreateProcess(NULL, "localizer.exe", NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &startInfo2, &localizerProcessInfo);
		//开启导航程序
		//Sleep(3000);
		memset(&startInfo3, 0, sizeof(STARTUPINFO));
		startInfo3.cb = sizeof(STARTUPINFO);
		startInfo3.dwFlags = STARTF_USESHOWWINDOW;
		startInfo3.wShowWindow = SW_SHOW;
		CreateProcess(NULL, "navigator.exe", NULL, NULL, FALSE, CREATE_NEW_CONSOLE, NULL, NULL, &startInfo3, &navigatorProcessInfo);
		
		return str_;
	}
	stdmsg::String finish_navigation(const stdmsg::String& str_)
	{
		stdmsg::String tmp;
		//先判断进程是否存在
		//TerminateProcess(localizerProcessInfo.hProcess, 0);
		//std::cout << "[finish_localizer] pid:" << localizerProcessInfo.hProcess << std::endl;
		TerminateProcess(navigatorProcessInfo.hProcess, 0);
		std::cout << "[finish_navigator] pid:" << navigatorProcessInfo.hProcess << std::endl;
		return str_;
	}
	stdmsg::String finish_localize(const stdmsg::String& str_)
	{
		stdmsg::String tmp;
		//先判断进程是否存在
		TerminateProcess(localizerProcessInfo.hProcess, 0);
		std::cout << "[finish_localizer] pid:" << localizerProcessInfo.hProcess << std::endl;
		//TerminateProcess(navigatorProcessInfo.hProcess, 0);
		//std::cout << "[finish_navigator] pid:" << navigatorProcessInfo.hProcess << std::endl;
		return str_;
	}

	stdmsg::String start_explore(const stdmsg::String& str_)
    {
		this->path_thread.kill();

        stdmsg::String tmp;
		
		memset(&startInfo, 0, sizeof(STARTUPINFO));
		startInfo.cb = sizeof(STARTUPINFO);
		startInfo.dwFlags = STARTF_USESHOWWINDOW;
		startInfo.wShowWindow = SW_SHOW;
		
		CreateProcess(NULL,"explore.exe",NULL,NULL,FALSE,CREATE_NEW_CONSOLE,NULL,NULL,&startInfo,&mappingProcessInfo);

        return str_;
    }
	stdmsg::String finish_explore(const stdmsg::String& str_)
    {
        stdmsg::String tmp;
		//tmp = rpc_slam.call<stdmsg::String,stdmsg::String>("maker",str_,1.5);
		tmp = rpc_slam.call<stdmsg::String, stdmsg::String>("maker", str_, 1, 60);
		//rpc_slam.call<stdmsg::String, stdmsg::String>("maker", str_);
		//Sleep(100);
		//rpc_slam.call<stdmsg::String, stdmsg::String>("maker", str_);
		//Sleep(100);
		//rpc_slam.call<stdmsg::String, stdmsg::String>("maker", str_);
		std::cerr<<"[finish_explorer] map saved."<<std::endl;

		stdmsg::Data tmpData;
		//this->map(tmpData);
		std::cerr<<"[finish_explorer] map send."<<std::endl;

		//先判断进程是否存在
		TerminateProcess(mappingProcessInfo.hProcess, 0);
		std::cout<<"[finish_explorer] pid:"<<mappingProcessInfo.hProcess<<std::endl;
        return str_;
    }
	stdmsg::String cancel_explore(const stdmsg::String& str_)
    {
        stdmsg::String tmp;
		//先判断进程是否存在
		TerminateProcess(mappingProcessInfo.hProcess, 0);
		std::cout<<"[cancel_explore] pid:"<<mappingProcessInfo.hProcess<<std::endl;
        return str_;
    }

	stdmsg::Data set_path(const stdmsg::Data& input)
	{
		
		std::string pathFileName = std::string(cfg.value("gui_connect", "path", "goalPath.txt"));
		//std::string pathFileName = std::string("goalPath.txt");
		printf("recv path file: %s\n", pathFileName.c_str());
		stdmsg::Data ret;
		FILE* fp = fopen(pathFileName.c_str(), "wb");
		if (fp == NULL)
		{
			fclose(fp);
			return ret;
		}
		int len = input.data().size();
		int tmp = fwrite(input.data().c_str(), len, 1, fp);
		fclose(fp);
		std::cout << "copy" << endl;;
		this->nav_mode = 2;//path
		this->path_thread.start();

		//return ret;

		return ret;
	}

	stdmsg::Data set_path2(const stdmsg::Data& input2)
	{

		std::string pathFileName2 = std::string("navigator.ini");
		printf("recv path file: %s\n", pathFileName2.c_str());
		stdmsg::Data ret2;
		FILE* fp2 = fopen(pathFileName2.c_str(), "wb");
		if (fp2 == NULL)
		{
			fclose(fp2);
			return ret2;
		}
		int len = input2.data().size();
		int tmp2 = fwrite(input2.data().c_str(), len, 1, fp2);
		fclose(fp2);
		std::cout << "copy2" << endl;;
		//this->nav_mode = 2;//path
		//this->path_thread.start();

		//return ret;

		return ret2;
	}

	//HXW.3.24
	stdmsg::Data set_initial(const stdmsg::Data& input)
	{

		std::string pathFileName = std::string(cfg.value("gui_connect", "path", "sendinitial.txt"));
		printf("recv path file: %s\n", pathFileName.c_str());
		stdmsg::Data ret;
		FILE* fp = fopen(pathFileName.c_str(), "wb");
		if (fp == NULL)
		{
			fclose(fp);
			return ret;
		}
		int len = input.data().size();
		int tmp = fwrite(input.data().c_str(), len, 1, fp);
		fclose(fp);

		//this->nav_mode = 2;//path
		//this->path_thread.start();

		//return ret;

		return ret;
	}

	stdmsg::MoveMode set_move_mode(const stdmsg::MoveMode& mod)
	{
		stdmsg::MoveMode tmp;
		tmp = rpc_slam.call<stdmsg::MoveMode,stdmsg::MoveMode>("set_movemode",mod);
		printf("move_mode set to %d (0=stop 1=goal 2=vel 3=follow)\n",mod.move_mode());
		return tmp;
	}

	stdmsg::Velocity set_vel(const stdmsg::Velocity& vel)
	{
		stdmsg::Velocity tmp;
		nh_slam->publish("set_cmd_vel",vel);
		printf("set speed velocity=(%.2f,%.2f)\n",vel.v(),vel.w());
		return tmp;
	}

	stdmsg::Data send_path_file(const stdmsg::Data& input)
	{
		std::string pathFileName = std::string(cfg.value("gui_connect", "path", "goalPath.txt"));
		printf("recv path requeset: %s\n", pathFileName.c_str());
        stdmsg::Data ret;
        FILE* fp = fopen(pathFileName.c_str(), "rb");
        if(fp == NULL)
        {
            fclose(fp);
            return ret;
        }
        fseek(fp, 0L, SEEK_END);
        int len = ftell(fp);
        void* data = new char[len];
        fseek(fp,0L, SEEK_SET);
        fread(data, len, 1, fp);
        fclose(fp);
        ret.set_data((char*)data, len);
        printf("send the path file to the client, length : %d\n", len);
        return ret;
	}

	void run_path()
	{
		//
		/*
		fstream pathFile21;
		std::string pathFileName21 = std::string(cfg.value("gui_connect", "path", "sendinitial.txt"));
		pathFile21.open(pathFileName21, ios::in);
		if (!pathFile21.good()){
			cout << "[PATH] read PATH file(" << pathFileName21 << ") failed.";
			return;
		}
		string line21;
		GOAL tmpGoal21;
		string goalInfo21;
		istringstream lineStream21(line21);
		while (getline(lineStream21, goalInfo21))
		{
			tmpGoal21.x = atof(goalInfo21.c_str());
			tmpGoal21.y = atof(goalInfo21.c_str());
			tmpGoal21.theta = atof(goalInfo21.c_str());
		}
		cout << "hhh" << tmpGoal21.x << " " << tmpGoal21.y << " " << tmpGoal21.theta << endl;
		 */
		//
		fstream pathFile;
		std::string pathFileName = std::string(cfg.value("gui_connect", "path", "goalPath.txt"));
		pathFile.open(pathFileName, ios::in);
		if (!pathFile.good()){
			std::cout << "[PATH] read PATH file(" << pathFileName << ") failed.";
			return;
		}
		string line;
		bool isGoal = false;
		bool isPath = false;
		bool isorigin = false;
		vector<GOAL>().swap(goalList);
		vector<PATH>().swap(pathList);
		GOAL xyz;
		while (getline(pathFile, line)) {
			if (line=="[Origin]")
			{
				isorigin = true;
				isGoal = false;
				isPath = false;
			}
			

			//cout <<"#" << line << endl;
			else if (line == "[GOAL]") {
				isorigin = false;
				isGoal = true;
				isPath = false;
			}
			else if (line == "[PATH]") {
				isorigin = false;
				isGoal = false;
				isPath = true;
			}
			else if (isorigin)
			{
				int i = 0;
				string goalInfo2;
				istringstream lineStream2(line);
				while (getline(lineStream2, goalInfo2, ','))
				{
					switch (i)
					{
					case 0:
						xyz.x = atof(goalInfo2.c_str());
						break;
					case 1:
						xyz.y = atof(goalInfo2.c_str());
						break;
					case 2:
						xyz.theta = atof(goalInfo2.c_str());
						break;
					}
					++i;
				}
				std::cout << "hxw" << xyz.x << xyz.y << xyz.theta << endl;
				isorigin = false;
			}
			else if (isGoal) {
				GOAL tmpGoal;
				istringstream lineStream(line);
				string goalInfo;
				int i = 0;
				while (getline(lineStream, goalInfo, ',')) {
					switch (i)
					{
					case 0:
						cout << "No." << goalInfo;
						break;
					case 1:
						//cout << ", x:" << goalInfo;
						tmpGoal.x = atof(goalInfo.c_str());
						//tmpGoal.x = tmpGoal.x * cos(xyz.theta)-tmp;
						cout << ", x:" << tmpGoal.x;
						break;
					case 2:
						//cout << ", y:" << goalInfo;
						tmpGoal.y = atof(goalInfo.c_str());
						cout << ", y:" << tmpGoal.y;
						break;
					case 3:
						//cout << ", t:" << goalInfo << endl;
						tmpGoal.theta = atof(goalInfo.c_str());
						cout << ", t:" << tmpGoal.theta << endl;
						break;
					default:
						cout << "[PATH] read goal error" << endl;
						break;
					}
					++i;
					
				}
				double a = tmpGoal.x;
				double b = tmpGoal.y;
				double c = tmpGoal.theta;
				double a2 = xyz.x;
				double b2 = xyz.y;
				double c2 = xyz.theta;
				//std::cout
				tmpGoal.x = a * cos(c2) - b * sin(c2) + a2;
				tmpGoal.y = (a) * sin(c2) + (b) * cos(c2) + b2;
				//tmpGoal.x = (tmpGoal.x) * cos(xyz.theta) - (tmpGoal.y) * sin(xyz.theta)+xyz.x;
				//tmpGoal.y = (tmpGoal.x) * sin(xyz.theta) + (tmpGoal.y) * cos(xyz.theta)+xyz.y;
				//tmpGoal.theta = atan2(tmpGoal.y,tmpGoal.x);
				tmpGoal.theta = c + c2;
				//tmpGoal.x = tmpGoal.x * cos(tmpGoal21.theta) - tmpGoal.y * sin(tmpGoal21.theta);
				//tmpGoal.y = tmpGoal.x * sin(tmpGoal21.theta) + tmpGoal.y * cos(tmpGoal21.theta);
				//tmpGoal.theta = atan2(tmpGoal.x - tmpGoal21.x, tmpGoal.y - tmpGoal21.y)+tmpGoal21.theta;
				//cout << "hhh" << tmpGoal.x << " " << tmpGoal.y << " " << tmpGoal.theta << endl;
				//std::cout << "nih" << tmpGoal.theta << endl;
				//std::cout << "dushu" << cos(60) << endl;
				std::cout << "hhh2" << " " << tmpGoal.x << " "<<tmpGoal.y << " "<<tmpGoal.theta<< endl;
				this->goalList.push_back(tmpGoal);
			}
			else if (isPath) {
				PATH tmpPath;
				istringstream lineStream(line);
				string pathInfo;
				int i = 0;
				while (getline(lineStream, pathInfo, ',')) {
					switch (i)
					{
					case 0:
						tmpPath.goalID = atoi(pathInfo.c_str());
						cout << "goalID:" << tmpPath.goalID;
						break;
					case 1:
						//cout << ", t:" << goalInfo << endl;
						tmpPath.type = atoi(pathInfo.c_str());
						cout << ", type:" << tmpPath.type << endl;
						break;
					default:
						cout << "[PATH] read path error" << endl;
						break;
					}
					++i;
				}
				this->pathList.push_back(tmpPath);
				//cout << "add path" << atoi(line.c_str()) << endl;
			}
		}
		pathFile.close();
		//set movemode goal
		for(size_t i = 0; i < pathList.size(); ++i){
			if(this->nav_mode != 2) break;

			int currentGoalID = pathList[i].goalID;
			GOAL currentGoal = goalList[currentGoalID];
			stdmsg::Pose pos;
			pos.mutable_position()->set_x(currentGoal.x);
			pos.mutable_position()->set_y(currentGoal.y);
			pos.mutable_orentation()->set_yaw(currentGoal.theta);

		    rpc_slam.call<stdmsg::Pose,stdmsg::Pose>("set_goal",pos);
			Sleep(100);
			rpc_slam.call<stdmsg::Pose,stdmsg::Pose>("set_goal",pos);
			Sleep(100);
			rpc_slam.call<stdmsg::Pose, stdmsg::Pose>("set_goal", pos);
		    std::cerr<<"path set_goal "<<currentGoalID<<std::endl;
			Sleep(3000);
			if(pathList[i].type == 0){//0,目标
				while(this->goalReach != 1) {Sleep(10);}
			}
			else{//1,路过
				while(this->distanceGoalReach != 1) {Sleep(10);}
			}
		}

		vector<GOAL>().swap(goalList);
		vector<PATH>().swap(pathList);
	}

    ~GuiConnect()
    {
        std::cerr<<"node uninitialize!"<<std::endl;
        if(nh_slam) delete nh_slam;
		if(nh_gui) delete nh_gui;
    }
};

int main(int argc, char **argv)
{
    GuiConnect F(argc, argv);
	while(1){_sleep(360000);}
}
