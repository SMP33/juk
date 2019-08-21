#include <iostream>
#include "NavigationNode.h"

using namespace std;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "JUK_NAVIGATION_NODE");
	
	NavigationNode::Parameters par;
	if (argc < 2)
		par.enable_emlid = false;
	else
		par.enable_emlid = (argv[1] != "0");
		
	par.enable_emlid = false;
	NavigationNode navigationNode(par);
	ros::spin();
	return 0;
}