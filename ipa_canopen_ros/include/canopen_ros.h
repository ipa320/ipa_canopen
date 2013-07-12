#ifndef CANOPEN_ROS_H
#define CANOPEN_ROS_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <cstdlib>


class JointLimits
    {
        private:
            std::vector<double> MaxVelocities_;
            std::vector<double> LowerLimits_;
            std::vector<double> UpperLimits_;
            std::vector<double> Offsets_;
            int DOF_;

        public:

            void setMaxVelocities(std::vector<double> max_vel)
            {
                MaxVelocities_ = max_vel;
            }

            void setLowerLimits(std::vector<double> low_lim)
            {
                LowerLimits_ = low_lim;
            }

            void setUpperLimits(std::vector<double> up_lim)
            {
                UpperLimits_ = up_lim;
            }

            void setOffsets(std::vector<double> offs)
            {
                Offsets_ = offs;
            }

            void setDOF(int dof)
            {
                DOF_ = dof;
            }

            std::vector<double> getMaxVelocities()
            {
                return MaxVelocities_;
            }

            std::vector<double> getLowerLimits()
            {
                return LowerLimits_;
            }

            std::vector<double> getUpperLimits()
            {
                return UpperLimits_;
            }

            std::vector<double> getOffsets()
            {
                return Offsets_;
            }

            int getDOF()
            {
                return DOF_;
            }


    };

#endif // CANOPEN_ROS_H
