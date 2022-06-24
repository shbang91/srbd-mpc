#include "srbd-mpc/types.hpp"

class MPCOutputData {
    public:
        MPCOutputData() {}
        virtual ~MPCOutputData() = default;
    // protected:
        // state results
        Eigen::VectorXd euler_angle_;
        Eigen::VectorXd com_pos_;
        Eigen::VectorXd euler_angle_rate_;
        Eigen::VectorXd com_vel_;
        // control results
        Eigen::VectorXd reaction_forces; // L, R
};