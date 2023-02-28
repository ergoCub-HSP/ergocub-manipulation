  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Pre-determined configurations for the iCub2                         //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ERGOCUBCONFIGURATIONS_H_
#define ERGOCUBCONFIGURATIONS_H_

#include <Eigen/Core>

Eigen::VectorXd home       = (Eigen::VectorXd(17) <<  0.0,  0.0,  0.00,                                     // Torso joints
                                                      0.0,  0.0,  0.00,  0.0,  0.0,  0.0,  0.0,             // Left arm
                                                      0.0,  0.0,  0.00,  0.0,  0.0,  0.0,  0.0).finished(); // Right arm
                                                                                  
                               
Eigen::VectorXd idealGrasp = (Eigen::VectorXd(17) <<  0.0,  0.0,  0.00,
                                                     -0.5,  0.5,  0.50,  0.9, -0.5, -0.1,  0.0,
                                                     -0.5,  0.5,  0.50,  0.9, -0.5, -0.1,  0.0).finished();
                          
Eigen::VectorXd shake      = (Eigen::VectorXd(17) <<  0.0,  0.0,  0.00,
                                                      0.2,  0.3,  0.00,  0.3,  0.0,  0.0,  0.0,
                                                     -0.5,  0.4,  0.50,  0.8, -0.8,  0.0,  0.0).finished();
                          
Eigen::VectorXd ready      = (Eigen::VectorXd(17) <<  0.0,  0.0,  0.00,
                                                     -0.2,  0.4,  0.00,  0.8, -0.4,  0.0,  0.0,
                                                     -0.2,  0.4,  0.00,  0.8, -0.4,  0.0,  0.0).finished();
                          
Eigen::VectorXd wave1      = (Eigen::VectorXd(17) <<  0.3, -0.2,  0.00,
                                                      0.0,  0.21,  0.00,  0.0,  0.0,  0.0,  0.0,
                                                      0.0,  0.5, -1.20,  1.4, -0.1,  0.0,  0.0).finished();
                            
Eigen::VectorXd wave2      = (Eigen::VectorXd(17) <<  0.3, -0.2,  0.00,
                                                      0.0,  0.21,  0.00,  0.0,  0.0,  0.0,  0.0,
                                                      0.0,  0.5, -1.20,  1.0, -0.1,  0.0,  0.0).finished();

#endif
