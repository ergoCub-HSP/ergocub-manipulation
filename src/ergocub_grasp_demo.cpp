    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                    Demonstration of bimanual grasping with the ergoCub robot                  //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

// #include <ergoCub.h>
// #include <ergoCubConfigurations.h>
#include <iCub2.h>
#include <iCub2Configurations.h>
#include <yarp/os/RpcServer.h>


double shortTime = 2.5;

// These are reference points with respect to the robot (for erboCub)
// double graspWidth    = 0.15;
// double graspDist     = 0.45;
// double graspRest     = 0.40;
// double torsoHeight   = 0.90;
// double torsoDist     = 0.40;
// double nominalHeight = torsoHeight + 0.35;
// double graspHeight   = nominalHeight;
// double graspLow      = graspHeight - 0.1;

// double roll  = 0.0*M_PI/180;
// double pitch = 0.0;
// double yaw   = 0.0*M_PI/180;

// These are reference points with respect to the robot (for iCub2)
// double graspWidth    = 0.153 / 2.0;
double graspWidth    = 0.180 / 2.0;
double graspDist     = 0.3;
double graspRest     = 0.25;
double torsoHeight   = 0.63;
double nominalHeight = torsoHeight + 0.1;
double graspHeight   = nominalHeight;
double torsoDist     = 0.3;
double graspLow      = graspHeight - 0.05;

double roll  = 0.0*M_PI/180; // x axis
double pitch = 0.0; //  y axis
double yaw   = -6.0*M_PI/180; // z axis

Eigen::Isometry3d leftHandGrasp = Eigen::Translation3d(graspDist, graspWidth, graspHeight)
                                * Eigen::AngleAxisd(-roll,  Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd( yaw,   Eigen::Vector3d::UnitZ());
                                 
Eigen::Isometry3d rightHandGrasp = Eigen::Translation3d(graspDist, -graspWidth, graspHeight)
                                 * Eigen::AngleAxisd( roll,  Eigen::Vector3d::UnitX())
                                 * Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(-yaw,   Eigen::Vector3d::UnitZ());
                                 
// These are used for creating a Payload object but not really important
double mass = 0.1;
Eigen::Matrix<double,3,3> inertia = (Eigen::MatrixXd(3,3) << 1e-06,   0.0,   0.0,
                                                               0.0, 1e-06,   0.0,
                                                               0.0,   0.0, 1e-06).finished();
// List of joints to control
// std::vector<std::string> jointList = {"torso_roll", "torso_pitch", "torso_yaw",
// 			              "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_yaw", "l_wrist_roll", "l_wrist_pitch",
// 		                      "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_yaw", "r_wrist_roll", "r_wrist_pitch"};
std::vector<std::string> jointList = {"torso_pitch", "torso_roll", "torso_yaw",
				      "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw",
				      "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"};


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                              Main                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Ensure correct number of arguments
	if(argc != 3)
	{
		std::cerr << "[ERROR] [ERGOCUB GRASP DEMO] Path to URDF, and port name are required. "
                          << "Usage: './ergocub_grasp_demo /portName /path/to/model.urdf' " << std::endl;

                return 1;                                                                           // Close with error
	}
	else
	{
		std::string portName   = argv[1];
		std::string pathToURDF = argv[2];

		std::vector<std::string> portList;
		portList.push_back(portName + "/torso");
		portList.push_back(portName + "/left_arm");
		portList.push_back(portName + "/right_arm");

		// ergoCub robot(pathToURDF, jointList, portList);                                     // Create the ergoCub controller
		iCub2 robot(pathToURDF, jointList, portList);

		// Configure communication across the yarp network
		yarp::os::Network yarp;                                                             // First connect to the network
		yarp::os::RpcServer port;                                                           // Create a port for sending / receiving info
		port.open("/Components/Manipulation");                                              // Open the port with the name '/command'
		yarp::os::Bottle input;                                                             // Store information from the user input
		yarp::os::Bottle output;                                                            // Store information to send to the user
		std::string command;                                                                // Response message, command from user

		// Run the control loop
		bool active = true;

		while(active)
		{
			output.clear();                                                             // Clear any previous information
			port.read(input,true);                                                      // Get the input from the /command port
			command = input.get(0).asString();                                          // Convert to a string

            if(command[0] == '[')
            {
                command.erase(0,1);
                command.erase(command.size() - 1, 1);
            }
			// These actions will not execute when in grasp mode
			if(not robot.is_grasping())
			{
				if(command == "accordian")
				{
					output.addString("Suonare");

					std::vector<Eigen::Isometry3d> leftPoses;
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.08,nominalHeight)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.22,nominalHeight)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.08,nominalHeight)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.15,nominalHeight)));

					std::vector<Eigen::Isometry3d> rightPoses;
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.08,nominalHeight)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.22,nominalHeight)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.08,nominalHeight)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.15,nominalHeight)));

					std::vector<double> times;
					times.push_back(1.0*shortTime);
					times.push_back(2.5*shortTime);
					times.push_back(4.0*shortTime);
					times.push_back(5.0*shortTime);

					robot.move_to_poses(leftPoses,rightPoses,times);
				}
				else if(command == "close")
				{
					output.addString("Arrivederci");
					active = false;
				}
				else if(command == "fancy")
				{
					output.addString("Non lo so");
					
					std::vector<Eigen::Isometry3d> leftPoses;
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.05,nominalHeight+0.1)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.15,nominalHeight+0.0)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.05,nominalHeight-0.1)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.15,nominalHeight+0.0)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.05,nominalHeight+0.1)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.15,nominalHeight+0.0)));
					
					std::vector<Eigen::Isometry3d> rightPoses;
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.05,nominalHeight-0.1)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.15,nominalHeight+0.0)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.05,nominalHeight+0.1)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.15,nominalHeight+0.0)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.05,nominalHeight-0.1)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.15,nominalHeight+0.0)));
					
					std::vector<double> times;
					times.push_back(1.5*shortTime);
					times.push_back(2.5*shortTime);
					times.push_back(3.5*shortTime);
					times.push_back(4.5*shortTime);
					times.push_back(5.5*shortTime);
					times.push_back(6.5*shortTime);

					robot.move_to_poses(leftPoses,rightPoses,times);
				}
				else if(command == "finished")
				{
					if(robot.is_finished()) output.addString("Si");
					else                    output.addString("No");
				}
				else if(command == "grasp")
				{
					if(not robot.is_grasping())
					{
                        			bool block = input.get(1).asBool();

						output.addString("Grazie");

						robot.move_to_pose(leftHandGrasp, rightHandGrasp, shortTime);
						
						yarp::os::Time::delay(1.1*shortTime);

						Eigen::Isometry3d boxPose(Eigen::Translation3d(graspDist,0.0,graspHeight)); // Pose of box relative to robot

						robot.grasp_object( Payload( robot.left_hand_pose().inverse()*boxPose, mass, inertia ) );

						// robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspRest,0.0,graspHeight)),2.0);

						if (block)  yarp::os::Time::delay(shortTime);
					}
				}
				else if(command == "egrasprlc")
				{
					if(not robot.is_grasping())
					{
                        bool block = input.get(1).asBool();

						output.addString("Grazie");

						robot.move_to_pose(leftHandGrasp, rightHandGrasp, shortTime);

						yarp::os::Time::delay(1.1*shortTime);

						Eigen::Isometry3d boxPose(Eigen::Translation3d(graspDist,0.0,graspHeight)); // Pose of box relative to robot

						robot.grasp_object( Payload( robot.left_hand_pose().inverse()*boxPose, mass, inertia ) );

                        yarp::os::Time::delay(shortTime);

                        // Left/Right motion
                        std::vector<Eigen::Isometry3d> leftPoses;
                        leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth+0.1, graspHeight)));
                        leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth-0.1, graspHeight)));
                        leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(graspRest, graspWidth, graspHeight)));

                        std::vector<Eigen::Isometry3d> rightPoses;
                        rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth+0.1, graspHeight)));
                        rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth-0.1, graspHeight)));
                        rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(graspRest,-graspWidth, graspHeight)));

						std::vector<double> times;
                        times.push_back(1.0*shortTime);
                        times.push_back(2.0*shortTime);
                        times.push_back(3.0*shortTime);

                        robot.move_to_poses(leftPoses,rightPoses,times);

                        // yarp::os::Time::delay(3.0*shortTime);
					}
				}
				else if(command == "mgraspround")
				{
					if(not robot.is_grasping())
					{
                        bool block = input.get(1).asBool();

						output.addString("Grazie");

						robot.move_to_pose(leftHandGrasp, rightHandGrasp, shortTime);

						yarp::os::Time::delay(1.1*shortTime);

						Eigen::Isometry3d boxPose(Eigen::Translation3d(graspDist,0.0,graspHeight)); // Pose of box relative to robot

						robot.grasp_object( Payload( robot.left_hand_pose().inverse()*boxPose, mass, inertia ) );

                        yarp::os::Time::delay(shortTime);

                        // Left/Right motion
                        std::vector<Eigen::Isometry3d> leftPoses;
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.10,graspHeight+0.00)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight+0.15)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth-0.10,graspHeight+0.00)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight-0.05)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight+0.00)));

                        std::vector<Eigen::Isometry3d> rightPoses;
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.10,graspHeight+0.00)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight+0.15)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth-0.10,graspHeight+0.00)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight-0.05)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight+0.00)));

						std::vector<double> times;
						times.push_back(1.0*shortTime);
						times.push_back(2.0*shortTime);
						times.push_back(3.0*shortTime);
						times.push_back(4.0*shortTime);
						times.push_back(5.0*shortTime);

                        robot.move_to_poses(leftPoses,rightPoses,times);
					}
				}
				else if(command == "testgrasp")
				{
					robot.move_to_pose(leftHandGrasp, rightHandGrasp, shortTime);

					output.addString("Grazie");

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "home")
				{
                    			bool block = input.get(1).asBool();

					output.addString("Casa");
					robot.move_to_position(home,shortTime);

			    		if (block) yarp::os::Time::delay(shortTime);
				}
				else if(command == "in")
				{
					output.addString("Capito");

					robot.translate(Eigen::Vector3d(0.0,-0.025, 0.0),
							Eigen::Vector3d(0.0, 0.025, 0.0),
							shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "out")
				{
					output.addString("Capito");

					robot.translate(Eigen::Vector3d(0.0, 0.025,0.0),
						        Eigen::Vector3d(0.0,-0.025,0.0),
						        shortTime);
				}
				else if(command == "ready")
				{
                    			bool block = input.get(1).asBool();

					output.addString("Pronto");
					robot.move_to_position(ready,shortTime);

                    			yarp::os::Time::delay(shortTime);
				}
				else if(command == "scissor")
				{
					output.addString("Tagliare");
					
					std::vector<Eigen::Isometry3d> leftPoses;
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.10,nominalHeight+0.1)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.10,nominalHeight-0.1)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.10,nominalHeight+0.1)));
					leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.10,nominalHeight+0.0)));

					std::vector<Eigen::Isometry3d> rightPoses;
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.10,nominalHeight-0.1)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.10,nominalHeight+0.1)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.10,nominalHeight-0.1)));
					rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.10,nominalHeight+0.0)));
					
					std::vector<double> times;
					times.push_back(1.0*shortTime);
					times.push_back(2.0*shortTime);
					times.push_back(4.0*shortTime);
					times.push_back(5.0*shortTime);

					robot.move_to_poses(leftPoses,rightPoses,times);
				}
				else if(command == "shake")
				{
                    			bool block = input.get(1).asBool();

					output.addString("Piacere");
					robot.move_to_position(shake,shortTime);

					if (block) yarp::os::Time::delay(shortTime);
				}
				else if(command == "stop")
				{
					output.addString("Fermare");
					robot.halt();
				}
				else if(command == "wave")
				{
                    			bool block = input.get(1).asBool();

					output.addString("Ciao");

					std::vector<Eigen::VectorXd> wave;
					wave.push_back(wave1);
					wave.push_back(wave2);
					wave.push_back(wave1);
					wave.push_back(wave2);
					wave.push_back(home);

					std::vector<double> times;
					times.push_back(1.0*shortTime);
					times.push_back(1.5*shortTime);
					times.push_back(2.0*shortTime);
					times.push_back(2.5*shortTime);
					times.push_back(4.0*shortTime);

					robot.move_to_positions(wave,times);

                    			if (block) yarp::os::Time::delay(4.0*shortTime);
				}
				else 	output.addString("Cosa");
			}
			// THESE ACTIONS WILL ONLY EXECUTE IN GRASP MODE
			else
			{
				if(command == "aft")
				{
					output.addString("Indietro");

					Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspRest, graspWidth, graspHeight));
					Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspRest,-graspWidth, graspHeight));
					
					robot.move_to_pose(leftTarget,rightTarget,shortTime);
					
//					robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspRest,0.0,graspHeight)),
//					                  shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "close")
				{
					output.addString("Arrivederci");
					active = false;
				}
				else if(command == "down")
				{
					output.addString("Giu'");
					
					Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth, graspLow));
					Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth, graspLow));
					
					robot.move_to_pose(leftTarget,rightTarget,shortTime);						

//					robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspDist,0.0,graspHeight-0.05)),
//					                  shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "finished")
				{
					if(robot.is_finished()) output.addString("Si");
					else                    output.addString("No");
				}
				else if(command == "fore")
				{
					output.addString("Avanti");

					Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth, graspHeight));
					Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth, graspHeight));
					
					robot.move_to_pose(leftTarget,rightTarget,shortTime);

//					robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspDist+0.05,0.0,graspHeight)),
//					                  shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "left")
				{
					output.addString("Sinistra");
					
					Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth+0.1, graspHeight));
					Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth+0.1, graspHeight));
					
					robot.move_to_pose(leftTarget,rightTarget,shortTime);

//					robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspDist,0.1,graspHeight)),
//					                  shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "release")
				{
					if(robot.is_grasping())
					{
                				bool block = input.get(1).asBool();

						output.addString("Capito");			
						
						Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth, graspHeight));
						Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth, graspHeight));
						
						robot.move_to_pose(leftTarget,rightTarget,shortTime);						

//						robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspDist,0.0,graspHeight)),
//						                  shortTime);                       // Move forward to drop off location

						yarp::os::Time::delay(shortTime);                   // Wait for the motion to complete

						robot.release_object();                             // Deactivate grasp mode

						robot.move_to_position(ready,shortTime);            // Move to ready configuration

						if (block) yarp::os::Time::delay(4.0*shortTime);
					}
				}
				else if(command == "right")
				{
					output.addString("Destra");
					
					Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth-0.1, graspHeight));
					Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth-0.1, graspHeight));
					
					robot.move_to_pose(leftTarget,rightTarget,shortTime);

//					robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspDist,-0.1,graspHeight)),
//					                  shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else if(command == "round")
				{
					if(not robot.is_grasping())
					{
						output.addString("Non posso");
					}
					else
					{
						output.addString("Girare");
						
						std::vector<Eigen::Isometry3d> leftPoses;
						// for ergoCub
						// leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.10,graspHeight+0.00)));
						// leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight+0.15)));
						// leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth-0.10,graspHeight+0.00)));
						// leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight-0.10)));
						// leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight+0.00)));

						// for iCub2
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.10,graspHeight+0.00)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight+0.15)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth-0.10,graspHeight+0.00)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight-0.05)));
						leftPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, graspWidth+0.00,graspHeight+0.00)));						
						
						std::vector<Eigen::Isometry3d> rightPoses;
						// for ergoCub						
						// rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.10,graspHeight+0.00)));
						// rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight+0.15)));
						// rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth-0.10,graspHeight+0.00)));
						// rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight-0.10)));
						// rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight+0.00)));

						// for iCub2
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.10,graspHeight+0.00)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight+0.15)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth-0.10,graspHeight+0.00)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight-0.05)));
						rightPoses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-graspWidth+0.00,graspHeight+0.00)));
						
						std::vector<double> times;
						times.push_back(1.0*shortTime);
						times.push_back(2.0*shortTime);
						times.push_back(3.0*shortTime);
						times.push_back(4.0*shortTime);
						times.push_back(5.0*shortTime);
						
						robot.move_to_poses(leftPoses,rightPoses,times);
/*
						std::vector<Eigen::Isometry3d> poses;
						poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.10,graspHeight+0.00)));
						poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.00,graspHeight+0.15)));
						poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist,-0.10,graspHeight+0.00)));
						poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.00,graspHeight-0.10)));
						poses.push_back(Eigen::Isometry3d(Eigen::Translation3d(torsoDist, 0.00,graspHeight+0.00)));

						std::vector<double> times;
						times.push_back(1.0*shortTime);
						times.push_back(2.0*shortTime);
						times.push_back(3.0*shortTime);
						times.push_back(4.0*shortTime);
						times.push_back(5.0*shortTime);

						robot.move_object(poses,times);
*/	
					}
				}
/*				else if(command == "spin")
				{
					if(not robot.is_grasping())
					{
						output.addString("Non posso");
					}
					else
					{
						output.addString("Girare");

						Eigen::Translation3d translation(torsoDist,0.0,graspHeight);

						std::vector<Eigen::Isometry3d> poses;
						poses.push_back(Eigen::Isometry3d(translation*Eigen::AngleAxisd(0.5,Eigen::Vector3d::UnitX())));
						poses.push_back(Eigen::Isometry3d(translation*Eigen::AngleAxisd(-0.5,Eigen::Vector3d::UnitX())));
						poses.push_back(Eigen::Isometry3d(translation*Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX())));

						std::vector<double> times;
						times.push_back(1.0*shortTime);
						times.push_back(2.0*shortTime);
						times.push_back(3.0*shortTime);

						robot.move_object(poses,times);
					}
				}
*/
				else if(command == "stop")
				{
					output.addString("Fermare");
					robot.halt();
				}
				else if(command == "up")
				{
					output.addString("Su");
					
					Eigen::Isometry3d leftTarget  = Eigen::Isometry3d(Eigen::Translation3d(graspDist, graspWidth, graspHeight+0.1));
					Eigen::Isometry3d rightTarget = Eigen::Isometry3d(Eigen::Translation3d(graspDist,-graspWidth, graspHeight+0.1));
					
					robot.move_to_pose(leftTarget,rightTarget,shortTime);

//					robot.move_object(Eigen::Isometry3d(Eigen::Translation3d(graspDist,0.0,graspHeight+0.1)),
//					                  shortTime);

					yarp::os::Time::delay(shortTime);
				}
				else output.addString("Cosa");
			}

			port.reply(output);
		}

		return 0;                                                                           // No problems with main
	}
}
