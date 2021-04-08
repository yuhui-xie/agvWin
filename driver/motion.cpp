#define EXPORT __declspec(dllexport)
#include "motion.h"
#include <sstream>
#include <iostream>
using namespace driver;

#ifdef PYTHON_PLUGIN
#include "Python.h"

class python
{
public:
    static python *GetInstance()
    {
        static python instance;
        return &instance;
    }

    PyObject* run(const char* p)
    {
        return PyRun_String(p , Py_eval_input, dict, dict);
    }
    PyObject* call(const char* p, const char *fmt, ...)
    {
        va_list arguments;
        PyObject* object = PyObject_GetAttrString(mainModule, p);
        PyObject* argv = Py_BuildValue(fmt, arguments);
        if(!object)
        {
            Py_DECREF(object);
            Py_DECREF(argv);
            PyErr_Print();
            return NULL;
        }
        PyObject* ret = PyEval_CallObject( object, argv );
        Py_DECREF(object);
        Py_DECREF(argv);
        return ret;
    }
    PyObject* operator() (const char* p)
    {
        PyObject* object = PyObject_GetAttrString(mainModule, p);
        return object;
    }
    void import(const char*p)
    {
        PyObject* module = PyImport_ImportModule(p);
        if( !module )
        {
            PyErr_Print();
            std::cerr<<"module "<<p<<" not found"<<std::endl;
            throw "driver not found";
        }
        mainModule = module;
        dict =  PyModule_GetDict(mainModule);
        //PyObject_SetAttrString(mainModule, p, module);
    }
private:
    python()
    {
        Py_Initialize();
        if (!Py_IsInitialized())
        {
            PyErr_Print();
            throw "initialize python error!";
        }
//        mainModule = PyImport_AddModule("__main__" );
//        dict =  PyModule_GetDict(mainModule);
//        if (!mainModule || !dict)
//        {
//            throw "initialize python error!";
//        }

        PyObject* sys_path = PySys_GetObject("path");
        if(!sys_path)
        {
            PyErr_Print();
            throw "initialize python error!";
        }
        PyObject* path = PyString_FromString(".");
        PyList_Append(sys_path, path);
        Py_DECREF(path);
        path = PyString_FromString("./driver");
        PyList_Append(sys_path, path);
        Py_DECREF(path);
        //PySys_SetPath(".");
    }
    ~python()
    {
        Py_Finalize();
    }
    PyObject* mainModule;
    PyObject* dict;
};

odometry::odometry(const std::string &port, int baud)
    :_port ( port ), _baud ( baud )/*,
	rpc_thread(this)*/
	//, _serial(NULL)
{
    reserved_data = new double(0);
    reserved_len = sizeof(double);

	/*std::string cfgfile("navigator.ini");
	cfg.read(cfgfile);*/

	/*std::string tmp = std::string(cfg.value("gui_connect", "rpc_gui_connect", "tcp://127.0.0.1:9022"));
	gp.bind(tmp);
	gp.connect(tmp);

	this->rpc_thread.start();*/
}
odometry::~odometry()
{

}

//crti:2016-06-21, 差速update
//crti;2016-07-05, 添加控制投影仪的内容
//fyf: 2016-12-2, 添加从底层读取充电器对接状态
//fyf: 2016-12-24, 添加从底层读取电池电量
void odometry::update_diff(double cmd_v, double cmd_w, int projector, int lighttime, int lightnum)
{
    //handle python
    static python* py = NULL;
    static PyObject* p_update  = NULL;

    if( !py )
    {
        py = python::GetInstance();
        py->import("driver_diff");
        p_update = (*py) ("update");
    }
	//std::cout << "jjjjjjjjjjjjjjjjjjjjjjjj" << std::endl;
	//std::cout << "lighttime2= "<<lighttime << std::endl;
	//std::cout << "lightnum2= "<<lightnum << std::endl;
    //printf("%.3f %.3f\n", cmd_v, cmd_w);
    //PyObject * ret = py->call("update", "(dd)", cmd_v, cmd_w );
    PyObject* call_data = Py_BuildValue("(ddiii)", cmd_v, cmd_w, projector,lighttime,lightnum);
    PyObject* ret = PyEval_CallObject( p_update, call_data );

    Py_DECREF(call_data);
    if( !ret )
    {
        Py_DECREF(ret);
        PyErr_Print();
        return;
    } 


    _lock.lock();
    double *steer = (double*) reserved_data;
	//crti:2016-06-22,add emergency_stop_flag
	PyArg_ParseTuple(ret, "dddddddiiiiiiii", &_x, &_y, &_t, &_v, &_w, steer, &_docking_state, &_battery_level, &_is_charging, &_control_sta, &_goal_port, &_record_path, &_set_point, &_set_now_pos, &_build_map);
    Py_DECREF(ret);
	
	//std::cout << "[odometry] goal_port = " << _goal_port << std::endl;
	//std::cout << "[odometry] record_path = " << _record_path << std::endl;
	//std::cout << "lighttime "<<lighttime << std::endl;
    _lock.unlock();
}

//crti:2016-06-21, 全向update
void odometry::update_omni(double cmd_v, double cmd_vy, double cmd_w)
{
    //handle python
    static python* py = NULL;
    static PyObject* p_update  = NULL;

    if( !py )
    {
        py = python::GetInstance();
        py->import("driver_omni");
        p_update = (*py) ("update");
    }

    PyObject* call_data = Py_BuildValue("(ddd)", cmd_v, cmd_vy, cmd_w);
    PyObject* ret = PyEval_CallObject( p_update, call_data );

    Py_DECREF(call_data);
    if( !ret )
    {
        Py_DECREF(ret);
        PyErr_Print();
        return;
    }
    _lock.lock();
    double *steer = (double*) reserved_data;
    PyArg_ParseTuple(ret, "ddddddd", &_x, &_y, &_t, &_v, &_vy, &_w, steer);
    Py_DECREF(ret);
    _lock.unlock();
}

#endif