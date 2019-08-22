#include <iostream>
#include "NavigationNode.h"

using namespace std;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "JUK_NAVIGATION_NODE");
	
	NavigationNode::Parameters par;
		
	par.enable_emlid = false;
	NavigationNode navigationNode(par);
	ros::spin();
	return 0;
}