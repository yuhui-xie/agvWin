#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>

class receive_test
{
protected:
	middleware::Node *nh;
	middleware::RPC rpc;
	stdmsg::Position p,pp;
private:
	struct _Node_Thread:public BThread
    {
        receive_test* handle;
        _Node_Thread(receive_test* p)
        {
            handle = p;
        }
        ~_Node_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
					//std::cout<<"running"<<std::endl;
                    //handle->nh->run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"recv nh communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_thread;
	struct _Rpc_Thread:public BThread
    {
        receive_test* handle;
        _Rpc_Thread(receive_test* p)
        {
            handle = p;
        }
        ~_Rpc_Thread()
        {
            kill();
        }
        void run()
        {
            try{
                while(true)
                {
                    //handle->rpc.run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"recv rpc communication error: "<< e.what() <<std::endl;
            }
        }
    } rpc_thread;
public:
	receive_test(): nh_thread(this),rpc_thread(this)
	{
		pp.set_x(10);
		pp.set_y(20);
		pp.set_z(30);
		nh = new middleware::Node("tcp://127.0.0.1:8551");
		nh->connect("tcp://127.0.0.1:8550");	
		nh->subscrible("test_node",&receive_test::node_handler,this);

		CrThread<receive_test> nht;
		nht.start(&receive_test::nhrun);

		rpc.connect("tcp://127.0.0.1:8552");
	}
	int nhrun(){
		try{
                while(true)
                {
					nh->run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"recv nh communication error: "<< e.what() <<std::endl;
            }
			return 0;
	};

	~receive_test()
	{
		if(nh)
			delete nh;
		this->nh_thread.kill();
		this->rpc_thread.kill();
	}

	void node_handler(const stdmsg::Position &p)
	{
		std::cout<<"node: x="<<p.x()<<" y="<<p.y()<<" z="<<p.z()<<std::endl;
	}

	void rpc_call()
	{
		std::cout<<"rpc_call"<<std::endl;
		stdmsg::Position tmp_p;
		tmp_p = rpc.call<stdmsg::Position,stdmsg::Position>("test_rpc",pp);
		std::cout<<"rpc: x="<<tmp_p.x()<<" y="<<tmp_p.y()<<" z="<<tmp_p.z()<<std::endl;
	}

	void run_node_rpc()
	{
		this->nh_thread.start();
		this->rpc_thread.start();
	}


};



int main()
{
	receive_test rt;
	//rt.run_node_rpc();
	while(true)
	{
		system("pause");
		rt.rpc_call();
		
	}
	
	
}