  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Custom class for 2-handed control of iCub 2                          //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ICUB2_H_
#define ICUB2_H_

#include <Eigen/Geometry>                                                                           // Eigen::Isometry3d and the like
#include <PositionControl.h>                                                                        // Custom class: most functions defined here
		        
class iCub2 : public PositionControl
{
	public:
		iCub2(const std::string &pathToURDF,
		      const std::vector<std::string> &jointNames,
		      const std::vector<std::string> &portNames);
	
	private:
		// Shoulder constraints
		Eigen::MatrixXd A;
		Eigen::Matrix<double,10,1> b;
		
		// General constraint matrices for QP solver
		Eigen::MatrixXd B;
		Eigen::VectorXd z;
		
		Eigen::VectorXd setPoint;                                                           // Desired joint configuration
		
		void run();                                                                         // Main control loop
			
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Constructor for the iCub 2                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
iCub2::iCub2(const std::string &pathToURDF,
             const std::vector<std::string> &jointNames,
             const std::vector<std::string> &portNames) :
             PositionControl(pathToURDF,
                             jointNames,
                             portNames,
                             Eigen::Isometry3d(Eigen::Translation3d(0.0,0.0,0.63)*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())),
                             "icub2")
{
	// Lower the gains since we're running in velocity mode
	set_joint_gains(5.0, 0.01);                                                                 // We don't actually care about the derivative
	set_cartesian_gains(10.0, 0.01);
	
	// Set the constraints for the iCub2 shoulder tendons.
	// A *single* arm is constrained by
	//      A*q + b > 0,
	// but we have two arms so we need to double up the constraint matrix.
	
	double c = 1.71;
	this->A = Eigen::MatrixXd::Zero(10,this->n);
	this->A.block(0,3,5,3) <<  c, -c,  0,
	                           c, -c, -c,
	                           0,  1,  1,
	                          -c,  c,  c,
	                           0, -1, -1;
	                           
	this->A.block(5,10,5,3) = this->A.block(0,3,5,3);                                           // Same constraint for right arm as left arm
	
	this->b.head(5) << 347.00*(M_PI/180),
	                   366.57*(M_PI/180),
	                    66.60*(M_PI/180),
	                   112.42*(M_PI/180),
	                   213.30*(M_PI/180);
	                   
	this->b.tail(5) = this->b.head(5);                                                          // Same constraint for the right arm
	
	// In discrete time we have:
	// A*(q + dq) >= b ---> A*dq > -(A*q + b)
	
	// The constraint matrix is unique to iCub2.
	// NOTE: First column here pertains to Lagrange multipliers when running in Cartesian mode
	// B = [ 0   -I ]
	//     [ 0    I ]
	//     [ 0    A ]
	this->B.resize(10+2*this->n,12+this->n);                                                    // 2*n for joint limits, 10 for shoulder limits
	this->B.block(        0, 0,10+2*this->n,12)      = Eigen::MatrixXd::Zero(10+2*this->n,12);
	this->B.block(        0,12,     this->n,this->n) =-Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(  this->n,12,     this->n,this->n) = Eigen::MatrixXd::Identity(this->n,this->n);
	this->B.block(2*this->n,12,          10,this->n) = this->A;
	
	this->z.resize(10+2*this->n);
	
	// Set the desired configuration for the arms when running in Cartesian mode
	this->setPoint.resize(this->n);
	this->setPoint.head(3).setZero();                                                           // Torso joints -> stay upright if possible
	this->setPoint(3) = -0.5;
	this->setPoint(4) =  0.5;
	this->setPoint(5) =  0.5;
	this->setPoint(6) =  0.9;                                                                   // Elbow
	this->setPoint(7) = -0.5;
	this->setPoint(8) = -0.1;
	this->setPoint(9) =  0.0;
	this->setPoint.tail(7) = this->setPoint.block(3,0,7,1);
	
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     MAIN CONTROL LOOP                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void iCub2::run()
{
	update_state();                                                                             // Update kinematics, dynamics, constraints
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Time since start of control loop
	
	if(this->controlSpace == joint)
	{
		Eigen::VectorXd qd(this->n);
		Eigen::VectorXd q0(this->n);
		
		for(int i = 0; i < this->n; i++)
		{
			qd(i) = this->jointTrajectory[i].evaluatePoint(elapsedTime);                // Desired state for the given time
			
			double lower = this->pLim[i][0];
			double upper = this->pLim[i][1];

			this->z(i)         = -upper;
			this->z(i+this->n) =  lower;
			
			q0(i) = 0.5*(lower + upper);
		}
		
		this->z.tail(10) = -this->b;
		
		try // to solve the QP problem
		{
			qRef = solve(Eigen::MatrixXd::Identity(this->n,this->n),                    // H
				    -qd,                                                            // f
				     this->B.block(0,12,10+2*this->n,this->n),                      // Remove component for Lagrange multipliers in Cartesian mode
				     this->z,                             
				     q0);
		}
		catch(const char* error_message)
		{
			std::cerr << error_message << std::endl;
		}
	}
	else
	{
		Eigen::VectorXd dx = track_cartesian_trajectory(elapsedTime);                       // Feedforward + feedback control
		Eigen::VectorXd redundantTask = 0.01*(this->setPoint - this->q);                    // As it says on the label
		Eigen::VectorXd q0 = Eigen::VectorXd::Zero(this->n);
		
		// Solve for instantaneous joint limits
		for(int i = 0; i < this->n; i++)
		{
			double lower, upper;
			compute_joint_limits(lower,upper,i);
			this->z(i) = -upper;
			this->z(i+this->n) = lower;
			q0(i) = 0.5*(lower + upper);
		}
		this->z.tail(10) = -(this->A*this->qRef + this->b);
		
		// H = [ 0  J ]
		//     [ J' M ]
		Eigen::MatrixXd H(12+this->n,12+this->n);
		H.block( 0, 0,     12,     12).setZero();
		H.block( 0,12,     12,this->n) = this->J;
		H.block(12, 0,this->n,     12) = this->J.transpose();
		H.block(12,12,this->n,this->n) = this->M;
		
		// f = [      -dx       ]
		//   = [ -M*redundantTask ]
		Eigen::VectorXd f(12+this->n);
		f.head(12)      = -dx;                                                              // Primary task
		f.tail(this->n) = -M*redundantTask;
		
		// Compute the start point for the QP solver
		Eigen::VectorXd startPoint(12+this->n);
		startPoint.head(12) = (this->J*this->Mdecomp.inverse()*this->J.transpose()).partialPivLu().solve(this->J*redundantTask - dx); // These are the Lagrange multipliers
		startPoint.tail(this->n) = q0;
		
		Eigen::VectorXd dq(this->n);
		
		try
		{
			dq = (solve(H,f,this->B,z,startPoint)).tail(this->n);                       // We can ignore the Lagrange multipliers
		}
		catch(const char* error_message)
		{
			std::cerr << error_message << std::endl;                                    // Inform user
			dq.setZero();                                                               // Don't move
		}
		
		// Re-solve the problem subject to grasp contraints Jc*qdot = 0
		if(this->isGrasping)
		{	
			Eigen::MatrixXd Jc = this->C*this->J;                                       // Constraint Jacobian
			
			// Set up the new start point for the solver
			startPoint.resize(6+this->n);
			startPoint.head(6)  = (Jc*this->Mdecomp.inverse()*Jc.transpose())*Jc*dq;    // Lagrange multipliers
			startPoint.tail(this->n) = dq;                                                   // Previous solution
			
			// H = [ 0   Jc ]
			//     [ Jc' M  ]
			H.resize(6+this->n,6+this->n);
			H.block(0,0,6,6).setZero();
			H.block(0,6,6,this->n)       = Jc;
			H.block(6,0,this->n,6)       = Jc.transpose();
			H.block(6,6,this->n,this->n) = M;
			
			// f = [   0   ]
			//     [ -M*dq ]
			f.resize(6+this->n);
			f.head(6).setZero();
			f.tail(this->n) = -this->M*dq;
			
			try // to solve the constrained motion
			{
				dq = (solve(H,
				            f,
				            this->B.block(0,6,10+2*this->n,6+this->n),              // Reduce the constraint since we now have -6 Lagrange multipliers
				            z,
				            startPoint)
				     ).tail(this->n);                                               // We can throw away the Lagrange multipliers from the result
			}
			catch(const char* error_message)
			{
				std::cerr << error_message << std::endl;                            // Inform user
				dq.setZero();                                                       // Don't move
			}
		}
		
		this->qRef += dq;                                                                   // Update the reference position
	}

	for(int i = 0; i < this->n; i++) send_joint_command(i,qRef[i]);
}
#endif
