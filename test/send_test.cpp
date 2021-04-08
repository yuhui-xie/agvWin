#include <node.hpp>
#include <stdmsg.hh>
#include <thread.hpp>

class send_test
{
protected:
	middleware::Node *nh;
	middleware::RPC rpc;
	stdmsg::Position p;
private:
	struct _Node_Thread:public BThread
    {
        send_test* handle;
        _Node_Thread(send_test* p)
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
					std::cout<<"running"<<std::endl;
                    handle->nh->run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"send nh communication error: "<< e.what() <<std::endl;
            }
        }
    } nh_thread;

	struct _Rpc_Thread:public BThread
    {
        send_test* handle;
        _Rpc_Thread(send_test* p)
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
                    handle->rpc.run();
                }
            }
            catch(const std::exception& e)
            {
                std::cerr<<"send rpc communication error: "<< e.what() <<std::endl;
            }
        }
    } rpc_thread;

public:
	send_test(): nh_thread(this),rpc_thread(this)
	{
		p.set_x(1.0);
		p.set_y(2.0);
		p.set_z(3.0);

		nh = new middleware::Node("tcp://127.0.0.1:8550");

		rpc.bind("tcp://127.0.0.1:8552");
		rpc.set("test_rpc",&send_test::reply_rpc,this);
		rpc.connect("tcp://127.0.0.1:8552");
	}
	~send_test()
	{
		if(nh)
			delete nh;
		this->nh_thread.kill();
		this->rpc_thread.kill();
	}

	void send_node()
	{
		nh->publish("test_node",p);
		std::cout<<"send_node"<<std::endl;
	}

	stdmsg::Position reply_rpc(const stdmsg::Position& input_no_use)
	{
		std::cout<<"replied_rpc_call"<<std::endl;
		return this->p;
	}

	void run_node_rpc()
	{
		this->nh_thread.start();
		this->rpc_thread.start();
	}

};

int main()
{
	class send_test st;
	st.run_node_rpc();
	while(true)
	{
		st.send_node();
		system("pause");
	}
	
}