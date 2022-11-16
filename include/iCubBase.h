    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                       Base class for bimanual control of the iCub                              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUBBASE_H_
#define ICUBBASE_H_

#include <CartesianTrajectory.h>                                                                    // Custom class
#include <Eigen/Dense>                                                                              // Tensors and matrix decomposition
#include <iDynTree/Core/EigenHelpers.h>                                                             // Converts iDynTree tensors to Eigen
#include <iDynTree/Core/CubicSpline.h>                                                              // For joint trajectories
#include <iDynTree/KinDynComputations.h>                                                            // Class for inverse dynamics calculations
#include <iDynTree/Model/Model.h>                                                                   // Class that holds basic dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <JointInterface.h>                                                                         // Communicates with motors
#include <QPSolver.h>                                                                               // Custom class
#include <yarp/os/PeriodicThread.h>                                                                 // Keeps timing of the control loop
#include <yarp/sig/Vector.h>

class iCubBase : public yarp::os::PeriodicThread,
                 public JointInterface,
                 public QPSolver
{
	public:
		
		iCubBase(const std::string &fileName,
		         const std::vector<std::string> &jointList,
		         const std::vector<std::string> &portList);
		         
		bool move_to_position(const yarp::sig::Vector &position,                            // Move joints to given position in a given time
		                      const double            &time);

		bool move_to_positions(const std::vector<yarp::sig::Vector> &positions,             // Move joints through multiple positions
				       const std::vector<double>            &times);
				       
		bool set_joint_gains(const double &proportional, const double &derivative);         // As it says on the label
				      
		void halt();
	
	protected:
	
		double dt = 0.01;                                                                   // Default control frequency
		
		double startTime;                                                                   // Used for timing the control loop
		
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
		
		enum ControlMode {joint, cartesian, grasp} controlMode;                             // Defines the different control modes
		
		// Joint control properties
		double kq = 10.0;                                                                   // Feedback on joint position error
		double kd =  5.0;                                                                   // Feedback on joint velocity error
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // Generates reference trajectories for joints
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Individual trajectories for left, right hand
		Eigen::Matrix<double,6,6> K;                                                        // Feedback on pose error
		Eigen::Matrix<double,6,6> D;                                                        // Feedback on velocity error
		
		// Kinematics & dynamics
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		iDynTree::Transform          torsoPose;                                             // Needed for inverse dynamics; not used yet
		iDynTree::Twist              torsoTwist;                                            // Needed for inverse dynamics; not used yet
		iDynTree::Vector3            gravity;                                               // Needed for inverse dynamics; not used yet
			                       	
		// Internal functions
		bool update_state();
		
		void get_speed_limits(double &minSpeed, double &maxSpeed, const int &i);            // For joint limit avoidance
		
		// Functions related to the PeriodicThread class
		bool threadInit();
		void threadRelease();
//              void run(); // This is declared in the child class since it can be unique
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         CONSTRUCTOR                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCubBase::iCubBase(const std::string &fileName,
                   const std::vector<std::string> &jointList,
                   const std::vector<std::string> &portList) :
                   yarp::os::PeriodicThread(0.01),                                                  // Create thread to run at 100Hz
                   JointInterface(jointList, portList),                                             // Communicates with joint motors
                   torsoPose(iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0), iDynTree::Position(0,0,0.64))),
                   torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0),iDynTree::GeomVector3(0,0,0)))
{
	// Set the gravity vector
	this->gravity(0) =  0.00;
	this->gravity(1) =  0.00;
	this->gravity(2) = -9.81;

	// Set the Cartesian control gains	
	this->K <<    1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		      0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		      0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		      0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
		      0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
		      0.0,   0.0,   0.0,   0.0,   0.0,   0.1;
	
	this->K *= 100.0;   
	
	this->D <<    1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		      0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		      0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		      0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
		      0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
		      0.0,   0.0,   0.0,   0.0,   0.0,   0.1;
	
	this->D *= 10.0;   

	// Load a model
	iDynTree::ModelLoader loader;                                                               // Temporary object

	if(not loader.loadReducedModelFromFile(fileName, jointList, "urdf"))
	{
		std::cerr << "[ERROR] [ICUB] Constructor: "
		          << "Could not load model from the given path " << fileName << std::endl;
	}
	else
	{
		// Get the model and add some additional frames for the hands
		iDynTree::Model temp = loader.model();
		temp.addAdditionalFrameToLink("l_hand", "left",
					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0.0),
								  iDynTree::Position(0,0,-0.06	)));
		temp.addAdditionalFrameToLink("r_hand", "right",
					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0.0),
					      			  iDynTree::Position(0,0,-0.06)));
							    
		if(not this->computer.loadRobotModel(temp))
		{
			std::cerr << "[ERROR] [ICUB] Constructor: "
				  << "Could not generate iDynTree::KinDynComputations class from given model: "
				  << loader.model().toString() << std::endl;
		}
		else
		{
			this->n = this->computer.model().getNrOfDOFs();                             // Get number of joints from underlying model

			// Resize vectors based on model information
			this->jointTrajectory.resize(this->n);                                      // Trajectory for joint motion control
			this->q.resize(this->n);                                                    // Vector of measured joint positions
			this->qdot.resize(this->n);                                                 // Vector of measured joint velocities

			std::cout << "[INFO] [ICUB] Successfully created iDynTree model from "
			          << fileName << "." << std::endl;

			update_state();                                                             // Get the current joint state
			
			if(not activate_control())
			{
				std::cerr << "[ERROR] [ICUB] Constructor: "
                                          << "Could not activate joint control." << std::endl;
                        }
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::update_state()
{
	if(JointInterface::read_encoders())
	{
		// Get the values from the JointInterface class
		std::vector<double> temp_position = get_joint_positions();
		std::vector<double> temp_velocity = get_joint_velocities();
		
		//////////////////// There is probably a smarter way to do this... /////////////////
		for(int i = 0; i < this->n; i++)
		{
			this->q[i]    = temp_position[i];
			this->qdot[i] = temp_velocity[i];
		}
		
		// Put them in to the iDynTree class to solve all the physics
		if(this->computer.setRobotState(this->torsoPose,
		                                iDynTree::VectorDynSize(temp_position),
		                                this->torsoTwist,
		                                iDynTree::VectorDynSize(temp_velocity),
		                                this->gravity))
		{
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [ICUB] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [ICUB] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the joints to a desired configuration                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_position(const yarp::sig::Vector &position,
                                const double &time)
{
	if(position.size() != this->n)
	{
		std::cerr << "[ERROR] [ICUB] move_to_position(): "
			  << "Position vector had " << position.size() << " elements, "
			  << "but this model has " << this->n << " joints." << std::endl;
			  
		return false;
	}
	else
	{
		std::vector<yarp::sig::Vector> target; target.push_back(position);                  // Insert in to std::vector to pass onward
		
		std::vector<double> times; times.push_back(time);                                   // Time in which to reach the target position
		
		return move_to_positions(target,times);                                             // Call "main" function
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_positions(const std::vector<yarp::sig::Vector> &positions,
                                 const std::vector<double> &times)
{
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [ICUB] move_to_positions(): "
		          << "Position array had " << positions.size() << " waypoints, "
		          << "but the time array had " << times.size() << " elements!" << std::endl;

		return false;
	}
	else
	{
		if(isRunning()) stop();                                                             // Stop any control thread that might be running
		
		this->controlMode = joint;                                                          // Switch to joint control mode
		
		int m = positions.size() + 1;                                                       // We need to add 1 extra waypoint for the start
		
		iDynTree::VectorDynSize waypoint(m);                                                // All the waypoints for a single joint
		iDynTree::VectorDynSize t(m);                                                       // Times to reach the waypoints
		
		for(int i = 0; i < this->n; i++)                                                    // For the ith joint...
		{
			for(int j = 0; j < m; j++)                                                  // ... and jth waypoint
			{
				if(j == 0)
				{
					waypoint[j] = this->q[i];                                   // Current position is start point
					t[j] = 0.0;                                                 // Start immediately
				}
				else
				{
					double target = positions[j-1][i];                                  // Get the jth target for the ith joint
					
					     if(target < this->pLim[i][0]) target = this->pLim[i][0] + 0.01;// Just above the lower limit
					else if(target > this->pLim[i][1]) target = this->pLim[i][1] - 0.01;// Just below the upper limit
					
					waypoint[j] = target;                                       // Assign the target for the jth waypoint
					
					t[j] = times[j-1];                                          // Add on subsequent time data
				}
			}
			
			if(not this->jointTrajectory[i].setData(t,waypoint))
			{
				std::cerr << "[ERROR] [ICUB] move_to_positions(): "
				          << "There was a problem setting new joint trajectory data." << std::endl;
			
				return false;
			}
		}
		
		start();                                                                            // Start the control thread
		return true;                                                                        // Success
	}
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the gains for control in the joint space                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional <= 0 or derivative <= 0)
	{
		std::cerr << "[ERROR] [ICUB] set_joint_gains(): "
                          << "Gains cannot be negative! "
                          << "You input " << proportional << " for the proportional gain, "
                          << "and " << derivative << " for the derivative gain." << std::endl;
                
                return false;
        }
        else
        {
        	this->kq = proportional;
        	this->kd = derivative;
        	return true;
        }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Stop the robot immediately                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::halt()
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	
	// There's probably a better way to do this:
	yarp::sig::Vector temp(this->n);
	for(int i = 0; i < this->n; i++) temp[i] = this->q[i];
	
	move_to_position(temp,2.0);                                                                 // Maintain current joint position
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::threadInit()
{
	this->startTime = yarp::os::Time::now();
	return true;
	// jump immediately to run();
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::threadRelease()
{
	std::vector<double> command;
	for(int i = 0; i < this->n; i++) command.push_back(0.0);                                    // Set all as zero
	send_velocity_commands(command);                                                            // Pass on to JointInterface
	
//      This is for when running in torque mode:
//	this->computer.generalizedGravityForces(this->generalForces);
//      send_torque_commands(this->generalForces.jointTorques();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get the instantenous speed limits for joint limit avoidance                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::get_speed_limits(double &minSpeed, double &maxSpeed, const int &i)
{
	double maxAcc = 50;
	
	// Compute lower limit
	minSpeed = std::max( (this->pLim[i][0] - this->q[i])/this->dt,
	           std::max( -this->vLim[i],
	                     -sqrt(2*maxAcc*(this->q[i] - this->pLim[i][0]))));
	              
	// Compute upper limit
	maxSpeed = std::min( (this->pLim[i][1] - this->q[i])/this->dt,
	           std::min(  this->vLim[i],
	                      sqrt(2*maxAcc*(this->pLim[i][1] - this->q[i]))));               
}

#endif
