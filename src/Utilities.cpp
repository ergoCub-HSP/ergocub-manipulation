#include <Utilities.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Convert a std::vector object to an Eigen::Isometry3d object                  //    
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d transform_from_vector(const std::vector<double> &input)
{
	if(input.size() != 6)
	{
		std::string errorMessage = "[ERROR] transform_from_vector(): 6 elements required, but your argument had " + std::to_string(input.size()) + " elements.\n";
		                         
		throw std::invalid_argument(errorMessage);
	}

	Eigen::Translation3d translation(input[0],input[1],input[2]);
	
	double angle = sqrt(input[3]*input[3] + input[4]*input[4] + input[5]*input[5]);
	
	Eigen::AngleAxisd angleAxis;
	
	if(angle == 0) angleAxis = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX());
	else           angleAxis = Eigen::AngleAxisd(angle, Eigen::Vector3d(input[3]/angle, input[4]/angle, input[5]/angle));
	
	return Eigen::Isometry3d(translation*angleAxis);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Convert a list of floating point numbers to an Eigen::Vector object              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd vector_from_bottle(const yarp::os::Bottle *bottle)
{
	Eigen::VectorXd vector(bottle->size());                                                     // Value to be returned
	
	for(int i = 0; i < bottle->size(); i++)
	{
		yarp::os::Value value = bottle->get(i);                                            // Get the ith element
		
		if(value.isFloat64()) vector(i) = value.asFloat64();                                // If it is a double, add to the vector
		else throw std::invalid_argument("[ERROR] vector_from_bottle(): The list contains a non-floating point element."); // Failure
	}
	
	return vector;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Convert a list of strings to a std::vector<std:string>>                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> string_from_bottle(const yarp::os::Bottle *bottle)
{
	std::vector<std::string> list;
	
	for(int i = 0; i < bottle->size(); i++)
	{
		yarp::os::Value value = bottle->get(i);                                             // Get the ith element
		
		if(value.isString()) list.push_back(value.asString());
		else throw std::invalid_argument("[ERROR] string_from_bottle(): The list contains a non-string element.");
	}
	
	return list;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Put joint trajectories from the config file in to a std::map                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool load_joint_configurations(const yarp::os::Bottle *bottle,
                               std::map<std::string,JointTrajectory> &map)
{
	std::vector<std::string> configName = string_from_bottle(bottle->find("names").asList());
	
	// Search through all the listed configuration names
	for(int i = 0; i < configName.size(); i++)
	{
		yarp::os::Bottle* vial = bottle->find(configName[i]).asList();
		if(vial == nullptr)
		{
			std::cerr << "[ERROR] Could not find the joint configuration(s) named "
			          << configName[i] << " in the JOINT_CONFIGURATIONS group of the config file.\n";  
			          
			return false;
		}
		
		// Get the points for this name
		yarp::os::Bottle* points = vial->find("points").asList();
		if(points == nullptr)
		{
			std::cerr << "[ERROR] The joint configuration " << configName[i] << " does not appear to have "
			          << "any points listed.\n";
			
			return false;
		}
		
		// Get the time for this name
		yarp::os::Bottle* times = vial->find("times").asList();
		if(times == nullptr)
		{
			std::cerr << "[ERROR] The joint configuration " << configName[i] << " does not appear to have "
			          << "any times listed.\n";
			
			return false;
		}
		
		// Make sure the number of elements match
		if(points->size() != times->size())
		{
			std::cerr << "[ERROR] In the " << configName[i] << " joint configuration, the points list had "
			          << points->size() << " elements, and the times list had " << times->size()
			          << " elements.\n";
			
			return false;
		}

		// Put the waypoints and times together in a single data structure
		JointTrajectory temp;
		for(int j = 0; j < points->size(); j++)
		{	
			temp.waypoints.push_back(vector_from_bottle(points->get(j).asList()));
			temp.times.push_back(times->get(j).asFloat64());
		}
		
		map.emplace(configName[i],temp);                                                    // Add to map so we can search by name later
	}
	
	return true;
}
