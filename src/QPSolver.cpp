#include <QPSolver.h>                                                                               // Declaration of functions

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a generic QP problem min 0.5*x'*H*x + x'*f                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd QPSolver::solve(const Eigen::MatrixXd &H,
                                const Eigen::VectorXd &f)
{
	
	if(H.rows() != H.cols())
	{
		std::string message = "[ERROR] [QP SOLVER] solve(): "
		                      "Expected a square matrix for the Hessian H but it was "
		                    + std::to_string(H.rows()) + "x" + std::to_string(H.cols()) + ".";                      
		               
		throw std::runtime_error(message);
	}
	else if(H.rows() != f.size())
	{	
		std::string message = "[ERROR] [QP SOLVER] solve(): "
		                      "Dimensions of arguments do not match. "
		                      "The Hessian H was " + std::to_string(H.rows()) + "x" + std::to_string(H.cols()) +
		                      " and the f vector was " + std::to_string(f.size()) + "x1.";
		                      
		throw std::runtime_error(message);
	}
	else 	return H.partialPivLu().solve(-f);                                                  // Too easy lol ᕙ(▀̿̿ĺ̯̿̿▀̿ ̿) ᕗ
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Solve a constrained QP problem: min 0.5*x'*H*x + x'*f subject to: B*x >= z           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd QPSolver::solve(const Eigen::MatrixXd &H,
                                const Eigen::VectorXd &f,
                                const Eigen::MatrixXd &B,
                                const Eigen::VectorXd &z,
                                const Eigen::VectorXd &x0)
{
	int dim = x0.size();                                                                        // Dimensions for the state vector
	int numConstraints = B.rows();                                                              // As it says
	
	// Check that the inputs are sound
	if(H.rows() != H.cols())
	{
		std::string message = "[ERROR] [QP SOLVER] solve(): Expected a square matrix for the Hessian but it was "
		                    + std::to_string(H.rows()) + "x" + std::to_string(H.cols()) + ".";
		         	
		throw std::runtime_error(message);
	}
	else if(f.size() != dim
	     or H.rows() != dim
	     or B.cols() != dim)
	{
		std::string message = "[ERROR] [QP SOLVER] solve(): Dimensions of arguments do not match. "
		                      "The Hessian was " + std::to_string(H.rows()) + "x" + std::to_string(H.cols()) + ", "
		                      "the f vector was " + std::to_string(f.size()) + "x1, "
		                      "the constraint matrix B had " + std::to_string(B.cols()) + " columns, and "
		                      "the start point x0 was " + std::to_string(x0.size()) + "x1.";

		throw std::runtime_error(message);
	}
	else if(B.rows() != z.size())
	{
		std::string message = "[ERROR] [QP SOLVER] solve(): Dimensions for constraints do not match. "
		                      "The constraint matrix B had " + std::to_string(B.rows()) + " rows, and "
		                      "the constaint vector z had " + std::to_string(z.size()) + " elements.";	
		
		throw std::runtime_error(message);
	}
	else
	{
		// Solve the following optimization problem with Guass-Newton method:
		//
		//    min f(x) = 0.5*x'*H*x + x'*f - u*sum(log(d_i))
		//
		// where d_i = b_i*x - c_i is the distance to the constraint
		//
		// Then the gradient and Hessian are:
		//
		//    g(x) = H*x + f - u*sum((1/d_i)*b_i')
		//
		//    I(x) = H + u*sum((1/(d_i^2))*b_i'*b_i)
		
		// Local variables
	
		Eigen::MatrixXd I;                                                                  // Hessian matrix
		Eigen::VectorXd g(dim);                                                             // Gradient vector
		Eigen::VectorXd dx = Eigen::VectorXd::Zero(dim);                                    // Newton step = -I^-1*g
		Eigen::VectorXd x = x0;                                                             // Assign initial state variable
		
		double alpha;                                                                        // Scalar for Newton step
		double beta  = this->beta0;                                                          // Shrinks barrier function
		double u     = this->u0;                                                             // Scalar for barrier function
	
		std::vector<double> d; d.resize(numConstraints);
		
		// Do some pre-processing
		std::vector<Eigen::VectorXd> bt(numConstraints);
		std::vector<Eigen::MatrixXd> btb(numConstraints);
		for(int j = 0; j < numConstraints; j++)
		{
			bt[j]  = B.row(j).transpose();                                              // Row vectors of B transposed
			btb[j] = bt[j]*bt[j].transpose();                                           // Outer product of row vectors
		}
		
		// Run the interior point method
		for(int i = 0; i < this->steps; i++)
		{
			// (Re)set values for new loop
			g.setZero();                                                                // Gradient vector
			I = H;                                                                      // Hessian for log-barrier function
			
			// Compute distance to each constraint
			for(int j = 0; j < numConstraints; j++)
			{
				d[j] = bt[j].dot(x) - z(j);                                         // Distance to jth constraint
				
				if(d[j] <= 0)
				{
					if(i == 0) throw std::runtime_error("[ERROR] [QP SOLVER] solve(): Start point x0 is outside the constraints!");
		
					d[j] = 1e-03;                                               // Set a small, non-zero value
					u *= 100;                                                   // Increase the barrier function
				}
						
				g += -(u/d[j])*bt[j];                                               // Add up gradient vector
				I +=  (u/(d[j]*d[j]))*btb[j];                                       // Add up Hessian
			}
			
			g += H*x + f;                                                               // Finish summation of gradient vector

			dx = I.partialPivLu().solve(-g);                                            // LU decomposition seems most stable
			
			// Ensure the next position is within the constraint
			alpha = this->alpha0;                                                       // Reset the scalar for the step size
			for(int j = 0; j < numConstraints; j++)
			{
				double dotProduct = bt[j].dot(dx);                                   // Makes things a little easier
				
				if( d[j] + alpha*dotProduct < 0 )                                   // If constraint violated on next step...
				{
					double temp = (1e-04 - d[j])/dotProduct;                     // Compute optimal scalar to avoid constraint violation
					
					if(temp < alpha) alpha = temp;                              // If smaller, override
				}
			}

			if(alpha*dx.norm() < this->tol) break;                                      // Change in position is insignificant; must be optimal
			
			// Update values for next loop
			x += alpha*dx;                                                              // Increment state
			u *= beta;                                                                  // Decrease barrier function
		}
			
		this->lastSolution = x;                                                             // Save this value for future use
		this->lastSolutionExists = true;                                                    // Flag that the interior point method has been run
		
		return x;
	}
}	

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Solve an unconstrained least squares problem: min 0.5(y-A*x)'*W*(y-A*x)              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd QPSolver::least_squares(const Eigen::VectorXd &y,
                                        const Eigen::MatrixXd &A,
                                        const Eigen::MatrixXd &W)
{
	if(A.rows() < A.cols())                                                                     // Redundant system, use other function
	{
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "The A matrix has more rows than columns ("
		                      + std::to_string(A.rows()) + "x" + std::to_string(A.cols()) + "). "
		                      "Did you mean to call the function for redundant least squares?";
		
		throw std::runtime_error(message);		                    		                   
	}
	if(W.rows() != W.cols())
	{
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Expected a square weighting matrix W but it was "
		                      + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ".";
		                      
		throw std::runtime_error(message);
	}
	else if(y.size() != W.rows() and A.rows() != y.size())
	{
		auto size = std::to_string(y.size());
		auto rows = std::to_string(A.rows());
		auto cols = std::to_string(W.cols());
		
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Dimensions of input arguments do not match. "
		                      "The y vector was " + std::to_string(y.size()) + "x1, "
		                      "the A matrix had " + std::to_string(A.rows()) + " rows, and "
		                      "the weighting matrix W was " + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ".";

		throw std::runtime_error(message);	
	}
	else	return (A.transpose()*W*A).partialPivLu().solve(A.transpose()*W*y);                 // x = (A'*W*A)^-1*A'*W*y
}
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //    Solve a constrained least squares problem 0.5*(y-A*x)'*W*(y-A*x) s.t. xMin <= x <= xMax    //
///////////////////////////////////////////////////////////////////////////////////////////////////                           
Eigen::VectorXd QPSolver::least_squares(const Eigen::VectorXd &y,
                                        const Eigen::MatrixXd &A,
                                        const Eigen::MatrixXd &W,
                                        const Eigen::VectorXd &xMin,
                                        const Eigen::VectorXd &xMax,
                                        const Eigen::VectorXd &x0)
{
	if(W.rows() != W.cols())
	{	
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Expected a square weighting matrix W but it was "
		                    + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ".";
		
		throw std::runtime_error(message);
	}
	else if(y.size() != W.rows() and A.rows() != y.size())
	{
		
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Dimensions of input arguments do not match. "
		                      "The y vector was " + std::to_string(y.size()) + "x1, "
		                      "the A matrix had " + std::to_string(A.rows()) + " rows, and "
		                      "the weighting matrix W was " + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ".";
	
		throw std::runtime_error(message);
	}
	else if(A.cols() != xMin.size() or xMin.size() != xMax.size() or xMax.size() != x0.size())
	{

		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Dimensions for the decision variable do not match. "
		                      "The A matrix had " + std::to_string(A.cols()) + " columns, "
		                      "the xMin vector had " + std::to_string(xMin.size()) + " elements, "
		                      "the xMax vector had " + std::to_string(xMax.size()) + " elements, and "
		                      "the start point x0 had " + std::to_string(x0.size()) + " elements.";
	
		throw std::runtime_error(message);
	}
	else
	{
		int n = x0.size();
		
		// Set up constraint matrices in standard form Bx >= c where:
		// B*x = [ -I ] >= [ -xMax ]
		//       [  I ]    [  xMin ]
		Eigen::MatrixXd B(2*n,n);
		B.block(n,0,n,n).setIdentity();
		B.block(0,0,n,n) = -B.block(n,0,n,n);

		Eigen::VectorXd z(2*n);
		z.head(n) = -xMax;
		z.tail(n) =  xMin;
		
		Eigen::MatrixXd AtW = A.transpose()*W;                                              // Makes calcs a little simpler

		return solve(AtW*A,-AtW*y, B, z, x0);                                               // Convert to standard form and solve
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //       Solve a least squares problem of the form min 0.5*(xd-x)'*W*(xd-x)  s.t. A*x = y        //
///////////////////////////////////////////////////////////////////////////////////////////////////                                  
Eigen::VectorXd QPSolver::redundant_least_squares(const Eigen::VectorXd &xd,
                                                  const Eigen::MatrixXd &W,
                                                  const Eigen::VectorXd &y,
                                                  const Eigen::MatrixXd &A)
{
	if(W.rows() != W.cols())
	{
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Expected the weighting matrix to be square but it was "
		                    + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ".";
		
		throw std::runtime_error(message);
	}
	else if(xd.size() != W.rows() or W.cols() != A.cols())
	{	
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Dimensions for the decision variable do not match. "
		                      "The desired vector xd had " + std::to_string(xd.size()) + " elements, "
		                      "the weighting matrix was " + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ", and "
		                      "the constraint matrix A had " + std::to_string(A.cols()) + " columns.";
		
		throw std::runtime_error(message);
        }
        else if(y.size() != A.rows())
        {    	
        	std::string message = "[ERROR] [QP SOLVER] least_squares(): "
        	                      "Dimensions for the equality constraint do not match. "
        	                      "The y vector had " + std::to_string(y.size()) + " elements, and "
        	                      "the constraint matrix had " + std::to_string(A.rows()) + " rows.";
        	                      
        	throw std::runtime_error(message);
        }
        else
        {   	
		// Lagrangian L = 0.5*x'*W*x - x'*W*xd + (A*x - y)'*lambda,
		// Solution exists for:
		//
		// [ dL/dlambda ]  =  [ 0   A ][ lambda ] - [   y  ] = [ 0 ]
		// [   dL/dx    ]     [ A'  W ][   x    ]   [ W*xd ]   [ 0 ]
		//
		// but we can speed up calcs and skip solving lambda if we are clever.
		
		int m = A.rows();
		int n = A.cols();
		
		Eigen::MatrixXd H(m+n,m+n);
		H.block(0,0,m,m).setZero();
		H.block(0,m,m,n) = A;
		H.block(m,0,n,m) = A.transpose();
		H.block(m,m,n,n) = W;
		
		Eigen::VectorXd f(m+n);
		f.head(m) = y;
		f.tail(n) = W*xd;
		
		// NOTE: We could be smarter here and skip solving the Lagrange multipliers using
		// QR decomposition...
		
		return (H.partialPivLu().solve(f)).tail(n);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //     Solve a problem of the form min 0.5*(xd-x)'*W*(xd-x)  s.t. A*x = y, xMin <= x <= xMax     //
///////////////////////////////////////////////////////////////////////////////////////////////////  
Eigen::VectorXd QPSolver::redundant_least_squares(const Eigen::VectorXd &xd,
                                                  const Eigen::MatrixXd &W,
                                                  const Eigen::VectorXd &y,
                                                  const Eigen::MatrixXd &A,
                                                  const Eigen::VectorXd &xMin,
                                                  const Eigen::VectorXd &xMax,
                                                  const Eigen::VectorXd &x0)
{
	unsigned int m = y.size();
	unsigned int n = x0.size();
	
	if(W.rows() != W.cols())
	{	
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Expected the weighting matrix to be square but it was "
		                    + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + ".";
		
		throw std::runtime_error(message);
	}
	else if(xd.size()   != n
	     or W.rows()    != n
	     or A.cols()    != n
	     or xMin.size() != n
	     or xMax.size() != n)
	{	
		std::string message = "[ERROR] [QP SOLVER] least_squares(): "
		                      "Dimensions for the decision variable do not match. "
		                      "The desired vector xd had " + std::to_string(xd.size()) + " elements, "
		                      "the weighting matrix W had " + std::to_string(W.rows()) + "x" + std::to_string(W.cols()) + " elements, "
		                      "the constraint matrix A had " + std::to_string(A.cols()) + " columns, "
		                      "the xMin vector had " + std::to_string(xMin.size()) + " elements, "
		                      "the xMax vector had " + std::to_string(xMax.size()) + " elements, and "
		                      "the start point x0 had " + std::to_string(n) + " elements.";
		
		throw std::runtime_error(message);
	}
        else if(A.rows() != m)
        {	
        	std::string message = "[ERROR] [QP SOLVER] least_squares(): "
        	                      "Dimensions for the equality constraint do not match. "
        	                      "The y vector had " + std::to_string(y.size()) + " elements, and "
        	                      "the A matrix had " + std::to_string(A.rows()) + " rows.";
        	
        	throw std::runtime_error(message);
        }
	else
	{
		// Convert to standard form 0.5*x'*H*x + x'*f subject to B*x >= z
		// where "x" is now [lambda' x' ]'
		
		// H = [ 0  A ]
		//     [ A' W ]
		Eigen::MatrixXd H(m+n,m+n);
		H.block(0,0,m,m).setZero();
		H.block(0,m,m,n) = A;
		H.block(m,0,n,m) = A.transpose();
		H.block(m,m,n,n) = W;
		
		// B = [ 0 -I ]
		//     [ 0  I ]
		Eigen::MatrixXd B(2*n,m+n);
		B.block(0,0,2*n,m).setZero();
		B.block(n,m,  n,n).setIdentity();
		B.block(0,m,  n,n) = -B.block(n,m,n,n);

		// z = [ -xMax ]
		//     [  xMin ]
		Eigen::VectorXd z(2*n);
		z.head(n) = -xMax;
		z.tail(n) =  xMin;

		// f = [   -y  ]
		//     [ -W*xd ]
		Eigen::VectorXd f(m+n);
		f.head(m) = -y;
		f.tail(n) = -W*xd;
		
		Eigen::VectorXd startPoint(m+n);
		startPoint.head(m) = (A*W.partialPivLu().inverse()*A.transpose()).partialPivLu().solve(A*xd - y);
		startPoint.tail(n) = x0;
		
		return (solve(H,f,B,z,startPoint)).tail(n);                                         // Convert to standard form and solve
	}
}                  
