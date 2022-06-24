#include "srbd-mpc/types.hpp"

struct ContactState
{
    /* data */
    void setZero(){

    }
};


class MPCInputData {
    public:
        MPCInputData() {
        com_pos_.setZero();
        euler_angle_.setZero();
        com_vel_.setZero();
        euler_angle_rate_.setZero();
        body_inertia_.setZero();
        l_foot_pose_.setZero();
        r_foot_pose_.setZero();
        T = 0.0;
        N = 0;
        }
        virtual ~MPCInputData() = default;
    // protected:
        // contact planning info
        std::vector<ContactState> l_foot_state_;
        std::vector<ContactState> r_foot_state_;
        // robot states
        Eigen::Vector3d com_pos_;
        Eigen::Vector3d euler_angle_;
        Eigen::Vector3d com_vel_;
        Eigen::Vector3d euler_angle_rate_;
        Eigen::Matrix<double, 6, 1> body_inertia_; // in order of xx, yy, zz, xy, yz, zx
        Eigen::Matrix<double, 7, 1> l_foot_pose_; // current left foot pose q.x, q.y, q.z, q.w, x,y,z,
        Eigen::Matrix<double, 7, 1> r_foot_pose_; // current right foot pose q.x, q.y, q.z, q.w, x,y,z
        // mpc problem formulation
        double T; // mpc prediction horizon time
        int N; // number of node
        // reference traj
        Eigen::VectorXd ref_euler_angle_;
        Eigen::VectorXd ref_com_pos_;
        Eigen::VectorXd ref_euler_angle_rate_;
        Eigen::VectorXd ref_com_vel_;
};