    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                         Position control functions for the iCub/ergoCub                       //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_

#include <iCubBase.h>


class PositionControl : public iCubBase
{
	public:
		PositionControl(const std::string &pathToURDF,
			        const std::vector<std::string> &jointNames,
			        const std::vector<std::string> &portNames,
			        const Eigen::Isometry3d &_torsoPose) :
	        iCubBase(pathToURDF,
	                 jointNames,
	                 portNames,
	                 _torsoPose) { this->qHat = this->q; }
		
		// Inherited from the iCubBase class   
		void compute_joint_limits(double &lower, double &upper, const int &i);
			              
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
		// Inherited from the yarp::PeriodicThread class
		bool threadInit();
		void threadRelease();
		
	protected:
		Eigen::VectorXd qHat;                                                               // Estimated joint configuration
		
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::compute_joint_limits(double &lower, double &upper, const int &i)
{
	lower = this->pLim[i][0] - this->qHat[i];
	upper = this->pLim[i][1] - this->qHat[i];
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::threadInit()
{
//	this->qHat = this->q;
	this->startTime = yarp::os::Time::now();
	return true;
	// jump immediately to run();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	for(int i = 0; i < this->n; i++) send_joint_command(i,this->qHat[i]);                       // Maintain current joint position
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a discrete time step for Cartesian control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,12,1> PositionControl::track_cartesian_trajectory(const double &time)
{
	// Variables used in this scope
	Eigen::Matrix<double,12,1> dx; dx.setZero();                                                // Value to be returned
	Eigen::Isometry3d pose;                                                                     // Desired pose
	Eigen::Matrix<double,6,1> vel, acc;
	

		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,vel,acc,time);
			
			dx.head(6) = this->dt*vel;
		}
		
		if(this->leftControl)
		{
			this->rightTrajectory.get_state(pose,vel,acc,time);
			
			dx.tail(6) = this->dt*vel;
		}
	
	return dx;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Solve the step size to track the joint trajectory                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd PositionControl::track_joint_trajectory(const double &time)
{
	Eigen::VectorXd dq(this->n); dq.setZero();                                                  // Value to be returned
	
	for(int i = 0; i < this->n; i++)
	{
		dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->qHat[i];
	}
	
	return dq;
}

#endif
