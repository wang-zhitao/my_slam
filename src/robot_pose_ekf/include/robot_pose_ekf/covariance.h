#ifndef COVARIANCE_H
#define COVARIANCE_H
#include <boost/array.hpp>
const boost::array<double, 36> ODOM_POSE_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0,
                                                        0, 1e-3, 0, 0, 0, 0,
                                                        0, 0, 1e6, 0, 0, 0,
                                                        0, 0, 0, 1e6, 0, 0,
                                                        0, 0, 0, 0, 1e6, 0,
                                                        0, 0, 0, 0, 0, 1e3}};
const boost::array<double, 36> ODOM_POSE_COVARIANCE2 = {{1e-9, 0, 0, 0, 0, 0,
                                                         0, 1e-3, 1e-9, 0, 0, 0,
                                                         0, 0, 1e6, 0, 0, 0,
                                                         0, 0, 0, 1e6, 0, 0,
                                                         0, 0, 0, 0, 1e6, 0,
                                                         0, 0, 0, 0, 0, 1e-9}};
const boost::array<double, 36> ODOM_TWIST_COVARIANCE = {{1e-3, 0, 0, 0, 0, 0,
                                                         0, 1e-3, 0, 0, 0, 0,
                                                         0, 0, 1e6, 0, 0, 0,
                                                         0, 0, 0, 1e6, 0, 0,
                                                         0, 0, 0, 0, 1e6, 0,
                                                         0, 0, 0, 0, 0, 1e3}};
const boost::array<double, 36> ODOM_TWIST_COVARIANCE2 = {{1e-9, 0, 0, 0, 0, 0,
                                                          0, 1e-3, 1e-9, 0, 0, 0,
                                                          0, 0, 1e6, 0, 0, 0,
                                                          0, 0, 0, 1e6, 0, 0,
                                                          0, 0, 0, 0, 1e6, 0,
                                                          0, 0, 0, 0, 0, 1e-9}};


boost::array<double, 9> covariance_li = {{-1,0,0,0,0,0,0,0,0}};
boost::array<double, 9> covariance_or = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};
boost::array<double, 9> covariance_an = {{1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6}};

#endif                                                          