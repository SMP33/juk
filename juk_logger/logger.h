#include <ros/ros.h>

#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <sstream>
#include <iostream>
#include <pthread.h>


template<typename T>
	class SimpleSub
	{
	public:
		SimpleSub(ros::NodeHandle* nh, std::string topic_name_, ros::Time start_time_)
			:
			topic_name(topic_name_),
			start_time(start_time_),
			upd(false)
		{
			sub = nh->subscribe(topic_name, 1, &SimpleSub::callback, this);
			last_upd_time = ros::Time::now();
		}
		
		std::string topic_name;
		T data;
		
		ros::Time last_upd_time;
		ros::Time start_time;
		bool upd;
		void clean_upd()
		{
			upd = false;
		}
		
		ros::Subscriber sub;
		
		std::stringstream full_str;
		
		std::string get_full_str()
		{
			return full_str.str()+"}";
		}
		void reset_str()
		{
			full_str.str("");
			full_str.clear();
			
			full_str.flags(std::ios::fixed);
			full_str.precision(10);
			full_str << "\"" << topic_name << "\":{" << std::endl;
			//std::cout << full_str.str() << std::endl;
			full_str << "\t\"upd\": \"" << (upd ? "YES" : "NO") << "\"," << std::endl; 
			//std::cout << full_str.str() << std::endl;
			full_str << "\t\"last_upd\":" << (last_upd_time - start_time) << "," << std::endl;
			//std::cout << full_str.str() << std::endl;
			full_str << std::endl;
			//std::cout << full_str.str() << std::endl;
			
		}
		template<typename V>
			void add_str(std::string txt, V value,bool vrg)
			{
				full_str << "\t\"" << txt << "\":" << value;
				if(vrg)
				full_str << ",";
				
				full_str << std::endl;
				
			}
		
		void callback(const typename T::ConstPtr& inp)
		{
			data = *inp;
			last_upd_time = ros::Time::now();
			upd = true;
		}
	};




class SimpleServer
{
public:
	SimpleServer(int  PORT);
	~SimpleServer();
	void* run();
	void set_response(std::string str);

private:
	int server_fd, client, valread;
	struct sockaddr_in address; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	uint PORT ;
	std::string response;
	pthread_t thr;
	pthread_mutex_t mutex;
	
	static void*run_helper(void* context)
	{
		((SimpleServer*)context)->run();
	}
};

SimpleServer::SimpleServer(int  PORT_):
	PORT(PORT_),
	response(" ")
{
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
	{ 
		perror("socket failed"); 
		exit(EXIT_FAILURE); 
	} 
	
	// Forcefully attaching socket to the port 8080 
	if(setsockopt(server_fd,
		SOL_SOCKET,
		SO_REUSEADDR | SO_REUSEPORT, 
		&opt,
		sizeof(opt))) 
	{ 
		perror("setsockopt"); 
		exit(EXIT_FAILURE); 
	} 
	address.sin_family = AF_INET; 
	address.sin_addr.s_addr = INADDR_ANY; 
	address.sin_port = htons(PORT); 
	
	// Forcefully attaching socket to the port 8080 
	if(bind(server_fd,
		(struct sockaddr *)&address, 
		sizeof(address)) < 0) 
	{ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 
	if (listen(server_fd, 3) < 0) 
	{ 
		perror("listen"); 
		exit(EXIT_FAILURE); 
	} 
	pthread_mutex_init(&mutex, NULL);
	pthread_create(&thr,NULL,&SimpleServer::run_helper,this);
}

void*
SimpleServer::run()
{
	while (1)
	{
		client = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
		char buffer[4096] = { 0 }; 
		read(client, buffer, 4096); 
		pthread_mutex_lock(&mutex);
		std::string str = response;
		pthread_mutex_unlock(&mutex);
		send(client, str.c_str(), str.length(), 0); 
		close(client);
	
	}
}

void
SimpleServer::set_response(std::string str)
{
	pthread_mutex_lock(&mutex);
	response = str;
	pthread_mutex_unlock(&mutex);
}

SimpleServer::~SimpleServer()
{
}