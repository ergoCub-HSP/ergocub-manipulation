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
		PositionControl(const std::string              &pathToURDF,
			        const std::vector<std::string> &jointNames,
			        const std::vector<std::string> &portNames,
			        const Eigen::Isometry3d        &_torsoPose,
			        const std::string              &robotName) :
	        iCubBase(pathToURDF, jointNames, portNames, _torsoPose, robotName) {}
		
		// Inherited from the iCubBase class   
		void compute_joint_limits(double &lower, double &upper, const int &i);
			              
		Eigen::Matrix<double,12,1> track_cartesian_trajectory(const double &time);
		
		Eigen::VectorXd track_joint_trajectory(const double &time);
		
		// Inherited from the yarp::PeriodicThread class
		bool threadInit();
		void threadRelease();
		
	protected:
		Eigen::VectorXd qRef;                                                               // Reference joint position to send to motors
		
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Compute instantenous position limits                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::compute_joint_limits(double &lower, double &upper, const int &i)
{
	lower = this->pLim[i][0] - this->qRef[i];
	upper = this->pLim[i][1] - this->qRef[i];
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Initialise the control thread                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PositionControl::threadInit()
{
	this->isFinished = false;                                                                   
	this->qRef = this->q;                                                                       // Use current joint configuration as reference point
	this->startTime = yarp::os::Time::now();
	return true;
	// jump immediately to run();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Executed after a control thread is stopped                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
void PositionControl::threadRelease()
{
	for(int i = 0; i < this->n; i++) send_joint_command(i,this->q[i]);                          // Maintain current joint position
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
	double gain = 0.005;
	
	if(this->isGrasping)
	{
		this->payloadTrajectory.get_state(pose,vel,acc,time);

		dx = this->G.transpose()*( this->dt*vel + gain*pose_error(pose,this->payload.pose()) );
	}
	else
	{
		if(this->leftControl)
		{
			this->leftTrajectory.get_state(pose,vel,acc,time);

			dx.head(6) = this->dt*vel + gain*pose_error(pose,this->leftPose);
		}

		if(this->rightControl)
		{
			this->rightTrajectory.get_state(pose,vel,acc,time);

			dx.tail(6) = this->dt*vel + gain*pose_error(pose,this->rightPose);
		}
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
		dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->q[i];                  
		
//		dq[i] = this->jointTrajectory[i].evaluatePoint(time) - this->qHat[i];
	}
	
	return dq;
}

#endif
