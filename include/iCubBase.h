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
#include <Payload.h>
#include <QPSolver.h>                                                                               // Custom class
#include <yarp/os/PeriodicThread.h>                                                                 // Keeps timing of the control loop
#include <yarp/sig/Vector.h>

class iCubBase : public yarp::os::PeriodicThread,                                                   // Regulates the control loop
                 public JointInterface,                                                             // Communicates with motor controllers
                 public QPSolver                                                                    // Used to solve joint control
{
	public:
		
		iCubBase(const std::string              &pathToURDF,
		         const std::vector<std::string> &jointNames,
		         const std::vector<std::string> &portNames,
		         const Eigen::Isometry3d        &_torsoPose,
		         const std::string              &robotName);
		
		// Joint Control Functions

		void halt();		                                                            // Stops the robot immediately
		
		bool move_to_position(const Eigen::VectorXd &position,
		                      const double &time);

		bool move_to_positions(const std::vector<Eigen::VectorXd> &positions,               // Move joints through multiple positions
				       const std::vector<double> &times);
				       		
		// Cartesian Control Functions
		
		bool is_grasping() const { return this->isGrasping; }
		
		bool is_finished() const { return this->isFinished; }
		
		bool move_to_pose(const Eigen::Isometry3d &leftPose,
		                  const Eigen::Isometry3d &rightPose,
		                  const double &time);
		
		bool move_to_poses(const std::vector<Eigen::Isometry3d> &leftPoses,
		                   const std::vector<Eigen::Isometry3d> &rightPoses,
		                   const std::vector<double> &times);
		                   
		bool translate(const Eigen::Vector3d &left,                                         // Translate both hands by the given amount
		               const Eigen::Vector3d &right,
		               const double &time);
		
		bool move_object(const Eigen::Isometry3d &pose,
		                 const double &time);
		                 
		bool move_object(const std::vector<Eigen::Isometry3d> &poses,
		                 const std::vector<double> &times);
				       
		bool grasp_object(const Payload &_payload);

		bool release_object();
		               
		// Information
				       
		bool print_hand_pose(const std::string &whichHand);                                 // As it says on the label
		
		bool set_cartesian_gains(const double &stiffness, const double &damping);
				       
		bool set_joint_gains(const double &proportional, const double &derivative);         // As it says on the label
		               
		Eigen::Isometry3d left_hand_pose()  const { return this->leftPose;  }
		
		Eigen::Isometry3d right_hand_pose() const { return this->rightPose; }
				      
	
	protected:
		std::string name;                                                                   // As an internal reference
		
		Payload payload;                                                                    
	
		bool isGrasping = false;
		
		bool isFinished = false;
	
		double dt     = 0.01;                                                               // Default control frequency
		double hertz  = 100;                                                                // Control frequency = 1/dt
		double maxAcc = 10;                                                                 // Limits the acceleration
		double startTime;                                                                   // Used for timing the control loop
		double endTime;                                                                     // Used for checking when actions are complete
		
		Eigen::VectorXd q, qdot;                                                            // Joint positions and velocities
		
		enum ControlSpace {joint, cartesian} controlSpace;
		
		// Joint control properties
		double kp = 1e-3;                                                                   // Feedback on joint position error
		double kd =  0.0;                                                                   // Feedback on joint velocity error
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                 // Generates reference trajectories for joints
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Individual trajectories for left, right hand
		CartesianTrajectory payloadTrajectory;
		Eigen::Matrix<double,6,6> K;                                                        // Feedback on pose error
		Eigen::Matrix<double,6,6> D;                                                        // Feedback on velocity error
		Eigen::Matrix<double,6,6> gainTemplate;                                             // Structure for the Cartesian gains
		Eigen::MatrixXd J;                                                                  // Jacobian for both hands
		Eigen::MatrixXd M;                                                                  // Inertia matrix
		Eigen::PartialPivLU<Eigen::MatrixXd> Mdecomp;                                                        // LU decomposition: M = L*U
		Eigen::Isometry3d leftPose, rightPose;                                              // Pose of the left and right hands
		
		// Grasping
		Eigen::Matrix<double,6,12> G, C;                                                    // Grasp and constraint matrices
		
		// Kinematics & dynamics
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		iDynTree::Transform          torsoPose;                                             // Needed for inverse dynamics; not used yet
			                       	
		// Internal functions
				
		bool update_state();                                                                // Get new joint state, update kinematics
		
		Eigen::Matrix<double,6,1> pose_error(const Eigen::Isometry3d &desired,
		                                     const Eigen::Isometry3d &actual);              // Get the error between 2 poses for feedback control
		                                     
		Eigen::Isometry3d iDynTree_to_Eigen(const iDynTree::Transform &T);                  // Convert iDynTree::Transform to Eigen::Isometry3d
		
		iDynTree::Transform Eigen_to_iDynTree(const Eigen::Isometry3d &T);
		
		// Virtual functions to be overridden in derived class                            
		
		virtual void compute_joint_limits(double &lower, double &upper, const int &i) = 0;    
		virtual Eigen::VectorXd track_joint_trajectory(const double &time) = 0;                 // Solve feedforward + feedback control         
		virtual Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time) = 0;
		
		// Functions related to the PeriodicThread class
//		bool threadInit();                                                                  // Defined in PositionControl.h
//		void threadRelease();                                                               // Defined in PositionContro.h
//              void run();                                                                         // Defined in iCub2.h

};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         CONSTRUCTOR                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCubBase::iCubBase(const std::string              &pathToURDF,
                   const std::vector<std::string> &jointNames,
                   const std::vector<std::string> &portNames,
                   const Eigen::Isometry3d        &_torsoPose,
                   const std::string              &robotName) :
                   yarp::os::PeriodicThread(0.01),                                                  // Create thread to run at 100Hz
                   JointInterface(jointNames, portNames),                                           // Communicates with joint motors
                   torsoPose(Eigen_to_iDynTree(_torsoPose)),
                   name(robotName)
{
	// Set the Cartesian control gains	
	this->gainTemplate << 1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		              0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		              0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		              0.0,   0.0,   0.0,   0.1,   0.0,   0.0,
		              0.0,   0.0,   0.0,   0.0,   0.1,   0.0,
		              0.0,   0.0,   0.0,   0.0,   0.0,   0.1;
	
	this->K = 10*this->gainTemplate;                                                            // Set the spring forces
	this->D =  5*this->gainTemplate;                                                            // Set the damping forces
	
	this->J.resize(12,this->n);
	this->M.resize(this->n,this->n);

	// G = [    I    0     I    0 ]
	//     [ S(left) I S(right) I ]
	this->G.block(0,0,3,3).setIdentity();
	this->G.block(0,3,3,3).setZero();
	this->G.block(0,6,3,3).setIdentity();
	this->G.block(0,9,3,3).setZero();
	this->G.block(3,3,3,3).setIdentity();
	this->G.block(3,9,3,3).setIdentity();
	
	// C = [  I  -S(left) -I  S(right) ]
	//     [  0      I     0     -I    ]
	C.block(0,0,3,3).setIdentity();
	C.block(0,6,3,3) = -C.block(0,0,3,3);
	C.block(3,0,3,3).setZero();
	C.block(3,3,3,3).setIdentity();
	C.block(3,6,3,3).setZero();
	C.block(3,9,3,3) = -C.block(0,0,3,3);

	// Load a model
	iDynTree::ModelLoader loader;                                                               // Temporary object

	if(not loader.loadReducedModelFromFile(pathToURDF, jointNames, "urdf"))
	{
		std::cerr << "[ERROR] [ICUB BASE] Constructor: "
		          << "Could not load model from the given path " << pathToURDF << "." << std::endl;
	}
	else
	{
		// Get the model and add some additional frames for the hands
		iDynTree::Model temp = loader.model();
		
		// Add custom hand frames based on the model of the robot
		if(this->name == "icub2")
		{
			temp.addAdditionalFrameToLink("l_hand", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,0.0),
			                                                  iDynTree::Position(0.05765, -0.00556, 0.01369)));
		
			temp.addAdditionalFrameToLink("r_hand", "right",
						      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,0.0,M_PI),
						                          iDynTree::Position(-0.05765, -0.00556, 0.01369)));
		}
		else if(this->name == "icub3")
		{
			std::cerr << "This hasn't been programmed yet!" << std::endl;
		}
		else if(this->name == "ergocub")
		{
			temp.addAdditionalFrameToLink("l_hand_palm", "left",
			                              iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
			                                                  iDynTree::Position(-0.00346, 0.00266, -0.0592)));
                	temp.addAdditionalFrameToLink("r_hand_palm", "right",
                				      iDynTree::Transform(iDynTree::Rotation::RPY(0.0,M_PI/2,0.0),
                				                          iDynTree::Position(-0.00387, -0.00280, -0.0597)));
		}
		else
		{
			std::cerr << "[ERROR] [ICUB BASE] Constructor: "
			          << "Expected icub2, icub3 or ergocub for the robot name, "
			          << "but your input was " << robotName << "." << std::endl;
		}
		
		// Now load the model in to the KinDynComputations class	    
		if(not this->computer.loadRobotModel(temp))
		{
			std::cerr << "[ERROR] [ICUB BASE] Constructor: "
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
			
			std::cout << "[INFO] [ICUB BASE] Successfully created iDynTree model from "
			          << pathToURDF << "." << std::endl;

			update_state();                                                             // Get the current joint state
			
			if(not activate_control())
			{
				std::cerr << "[ERROR] [ICUB BASE] Constructor: "
					  << "Could not activate joint control." << std::endl;
			}
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Move each hand to a desired pose                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_pose(const Eigen::Isometry3d &leftPose,
                            const Eigen::Isometry3d &rightPose,
                            const double &time)
{
	// Put them in to std::vector objects and pass onward
	std::vector<Eigen::Isometry3d> leftPoses(1,leftPose);
	std::vector<Eigen::Isometry3d> rightPoses(1,rightPose);
	std::vector<double> times(1,time);
	
	return move_to_poses(leftPoses,rightPoses,times);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move both hands through multiple poses                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_poses(const std::vector<Eigen::Isometry3d> &left,
                             const std::vector<Eigen::Isometry3d> &right,
                             const std::vector<double> &times)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlSpace = cartesian;                                                             // Switch to Cartesian control mode
	this->leftControl = true; this->rightControl = true;                                        // Activate both hands
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0.0);                                                    // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the left hand trajectory	
	std::vector<Eigen::Isometry3d> waypoint; waypoint.push_back(this->leftPose);
	waypoint.insert(waypoint.end(),left.begin(),left.end());                                    // Add additional waypoints to the end
	this->leftTrajectory = CartesianTrajectory(waypoint, t);                                    // Create the left-hand trajectory
	
	// Set up the right hand trajectory
	waypoint.clear();
	waypoint.push_back(this->rightPose);
	waypoint.insert(waypoint.end(),right.begin(),right.end());                                  // Add additional poses to the end
	this->rightTrajectory = CartesianTrajectory(waypoint,t);                                    // Create new trajectory for the right hand
	
	this->endTime = times.back();
	
	start(); // Go immediately to threadInit();

	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Move the box to a given pose                                      //          
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_object(const Eigen::Isometry3d &pose,
                           const double &time)
{
	if(time < 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_object(): "
		          << "Time of " << time << " cannot be negative!" << std::endl;
		          
		return false;
	}
	else
	{
		// Insert in to std::vector objects and pass on to spline generator
		std::vector<Eigen::Isometry3d> poses;
		poses.push_back(pose);
		
		std::vector<double> times;
		times.push_back(time);
		
		return move_object(poses,times);                                                    // Pass onward for spline generation
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Move the box through multiple poses                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_object(const std::vector<Eigen::Isometry3d> &poses,
                           const std::vector<double> &times)
{
	if( isRunning() ) stop();                                                                   // Stop any control threads that are running
	
	this->controlSpace = cartesian;                                                             // Ensure that we are running in Cartesian mode
	
	// Set up the times for the trajectory
	std::vector<double> t; t.push_back(0);                                                      // Start immediately
	t.insert(t.end(),times.begin(),times.end());                                                // Add on the rest of the times
	
	// Set up the waypoints for the object
	std::vector<Eigen::Isometry3d> waypoints; waypoints.push_back( this->payload.pose() );      // First waypoint is current pose
	waypoints.insert( waypoints.end(), poses.begin(), poses.end() );                            // Add on additional waypoints to the end
	this->payloadTrajectory = CartesianTrajectory(waypoints, t);                                // Create new trajectory to follow
	
	this->endTime = times.back();                                                               // Assign the end time
	
	start(); // go immediately to threadInit()
	
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Grasp an object with two hands                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::grasp_object(const Payload &_payload)
{
	if(this->isGrasping)
	{
		std::cout << "[INFO] [ICUB BASE] grasp_object(): "
		          << "You are already grasping an object!" << std::endl;
		
		return false;
	}
	else
	{
		this->isGrasping = true;                                                            // Set grasp constraint
//		this->payload = _payload;                                                           // Transfer payload information
		
//		update_state();                                                                     // This will update the payload pose
		
//		return move_object( this->payload.pose(), 5.0 );                                    // Run control on payload pose
		return true;
	}
}
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Release an object                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::release_object()
{
	if(this->isGrasping)
	{
		this->isGrasping = false;
		
		// Move the hands apart?
		
		return true;
	}
	else
	{
		std::cout << "[INFO] [ICUB BASE] release_object(): "
		          << "You are not currently grasping!" << std::endl;
		
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Move the joints to a desired configuration                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_position(const Eigen::VectorXd &position,
                                const double &time)
{
	if(position.size() != this->n)
	{
		std::cerr << "[ERROR] [ICUB BASE] move_to_position(): "
			  << "Position vector had " << position.size() << " elements, "
			  << "but this model has " << this->n << " joints." << std::endl;
			  
		return false;
	}
	else
	{
		std::vector<Eigen::VectorXd> target; target.push_back(position);                    // Insert in to std::vector to pass onward
		std::vector<double> times; times.push_back(time);                                   // Time in which to reach the target position
		return move_to_positions(target,times);                                             // Call "main" function
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::move_to_positions(const std::vector<Eigen::VectorXd> &positions,
                                 const std::vector<double> &times)
{
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [ICUB BASE] move_to_positions(): "
		          << "Position array had " << positions.size() << " waypoints, "
		          << "but the time array had " << times.size() << " elements!" << std::endl;

		return false;
	}
	else
	{
		if(isRunning()) stop();                                                             // Stop any control thread that might be running
		this->controlSpace = joint;                                                         // Switch to joint control mode
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
					
					     if(target < this->pLim[i][0]) target = this->pLim[i][0] + 0.001;// Just above the lower limit
					else if(target > this->pLim[i][1]) target = this->pLim[i][1] - 0.001;// Just below the upper limit
					
					waypoint[j] = target;                                       // Assign the target for the jth waypoint
					
					t[j] = times[j-1];                                          // Add on subsequent time data
				}
			}
			
			if(not this->jointTrajectory[i].setData(t,waypoint))
			{
				std::cerr << "[ERROR] [ICUB BASE] move_to_positions(): "
				          << "There was a problem setting new joint trajectory data." << std::endl;
			
				return false;
			}
		}
		
		this->endTime = times.back();                                                       // Assign the end time
		
		start();                                                                            // Start the control thread
		return true;                                                                        // Success
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Print the pose of a hand to the console                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::print_hand_pose(const std::string &which)
{
	if(which == "left" or which == "right")
	{
		std::cout << "Here is the " << which << " hand pose:" << std::endl;
		std::cout << this->computer.getWorldTransform(which).asHomogeneousTransform().toString() << std::endl;

		return true;
	}
	else
	{
		std::cout << "[ERROR] [ICUB BASE] print_hand_pose(): " 
		          << "Expected 'left' or 'right' as the argument, "
		          << "but the input was " << which << "." << std::endl;
		
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Set the gains for control in Cartesian space                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_cartesian_gains(const double &stiffness, const double &damping)
{
	if(stiffness <= 0 or damping <= 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_cartesian_gains(): "
		          << "Gains cannot be negative! "
		          << "You input " << stiffness << " for the stiffness gain, "
		          << "and " << damping << " for the damping gain." << std::endl;
		
		return false;
	}
	else
	{	
		this->K = stiffness*this->gainTemplate;
		this->D =   damping*this->gainTemplate;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the gains for control in the joint space                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::set_joint_gains(const double &proportional, const double &derivative)
{
	if(proportional <= 0 or derivative <= 0)
	{
		std::cerr << "[ERROR] [ICUB BASE] set_joint_gains(): "
                          << "Gains cannot be negative! "
                          << "You input " << proportional << " for the proportional gain, "
                          << "and " << derivative << " for the derivative gain." << std::endl;
                
                return false;
        }
        else
        {
        	this->kp = proportional;
        	this->kd = derivative;
        	return true;
        }
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Translate both hands by the given amount                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::translate(const Eigen::Vector3d &left,
                         const Eigen::Vector3d &right,
                         const double &time)
{
	if(this->isGrasping)
	{
		return move_object( this->payload.pose() * Eigen::Translation3d(left) , time );
	}
	else
	{
		Eigen::Isometry3d leftTarget  = this->leftPose  * Eigen::Translation3d(left);
		Eigen::Isometry3d rightTarget = this->rightPose * Eigen::Translation3d(right);
		
		return move_to_pose(leftTarget, rightTarget, time);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Stop the robot immediately                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCubBase::halt()
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	
	for(int i = 0; i < this->n; i++) send_joint_command(i,this->q[i]);                          // Hold current joint position
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the error between a desired and actual pose                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> iCubBase::pose_error(const Eigen::Isometry3d &desired,
                                               const Eigen::Isometry3d &actual)
{
	Eigen::Matrix<double,6,1> error;                                                            // Value to be computed
	
	error.head(3) = desired.translation() - actual.translation();                               // Position / translation error
	
	Eigen::Matrix<double,3,3> R = desired.rotation()*actual.rotation().inverse();               // Rotation error as SO(3)
	
	// "Unskew" the rotation error
	error(3) = R(2,1);
	error(4) = R(0,2);
	error(5) = R(1,0);
	
	return error;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Update the kinematics & dynamics of the robot                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool iCubBase::update_state()
{
	if(JointInterface::read_encoders())
	{	
		// Get the values from the JointInterface class (is there a smarter way???)
		std::vector<double> temp_position = joint_positions();
		std::vector<double> temp_velocity = joint_velocities();
		
		// Transfer joint state values for use in this class
		for(int i = 0; i < this->n; i++)
		{
			this->q[i]    = temp_position[i];
			this->qdot[i] = temp_velocity[i];
		}

		// Put them in to the iDynTree class to solve the kinematics and dynamics
		if(this->computer.setRobotState(this->torsoPose,                                    // As it says on the label
		                                iDynTree::VectorDynSize(temp_position),             // Joint positions
		                                iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)), // Torso twist
		                                iDynTree::VectorDynSize(temp_velocity),             // Joint velocities
		                                iDynTree::Vector3(std::vector<double> {0.0, 0.0, -9.81}))) // Direction of gravity
		{
		
			// Get the Jacobian for the hands
			Eigen::MatrixXd temp(6,6+this->n);                                          // Temporary storage
			
			this->computer.getFrameFreeFloatingJacobian("left",temp);                 // Compute left hand Jacobian
			this->J.block(0,0,6,this->n) = temp.block(0,6,6,this->n);                   // Assign to larger matrix
			
			this->computer.getFrameFreeFloatingJacobian("right",temp);                  // Compute right hand Jacobian
			this->J.block(6,0,6,this->n) = temp.block(0,6,6,this->n);                   // Assign to larger matrix
			
			// Compute inertia matrix
			temp.resize(6+this->n,6+this->n);
			this->computer.getFreeFloatingMassMatrix(temp);                             // Compute full inertia matrix
			this->M = temp.block(6,6,this->n,this->n);                                  // Remove floating base
			
			this->Mdecomp = M.partialPivLu();                                           // Decompose M = L*U
			
			// Update hand poses
			this->leftPose  = iDynTree_to_Eigen(this->computer.getWorldTransform("left"));
			this->rightPose = iDynTree_to_Eigen(this->computer.getWorldTransform("right"));
			
			// Update the grasp and constraint matrices
			if(this->isGrasping)
			{
				// Assume the payload is rigidly attached to the left hand
				this->payload.update_state(this->leftPose,
				                           iDynTree::toEigen(this->computer.getFrameVel("left"))); 

				// G = [    I    0     I    0 ]
				//     [ S(left) I S(right) I ]
				
				// C = [  I  -S(left)  -I  S(right) ]
				//     [  0      I      0    -I     ]
				
				Eigen::Matrix<double,3,3> S;                                        // Skew symmetric matrix
				
				// Left hand component
				Eigen::Vector3d r = this->leftPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0 ;
				
				this->G.block(3,0,3,3) =  S;
				this->C.block(0,3,3,3) =  S;
				
				// Right hand component
				r = this->rightPose.translation() - this->payload.pose().translation();
				
				S <<    0 , -r(2),  r(1),
				      r(2),    0 , -r(0),
				     -r(1),  r(0),    0;
				     
				this->G.block(3,6,3,3) = S;
				this->C.block(0,9,3,3) =-S;
			}
			
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [ICUB BASE] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [ICUB BASE] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Convert iDynTree::Transform to Eigen::Isometry3d                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3d iCubBase::iDynTree_to_Eigen(const iDynTree::Transform &T)
{
	iDynTree::Position pos = T.getPosition();
	iDynTree::Vector4 quat = T.getRotation().asQuaternion();
	
	return Eigen::Translation3d(pos[0],pos[1],pos[2])*Eigen::Quaterniond(quat[0],quat[1],quat[2],quat[3]);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Convert Eigen::Isometry3d to iDynTree::Transform                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Transform iCubBase::Eigen_to_iDynTree(const Eigen::Isometry3d &T)
{
	Eigen::Matrix<double,3,3> R = T.rotation();
	Eigen::Vector3d           p = T.translation();
	
	return iDynTree::Transform(iDynTree::Rotation(R),
	                           iDynTree::Position(p(0),p(1),p(2)));
}

#endif
