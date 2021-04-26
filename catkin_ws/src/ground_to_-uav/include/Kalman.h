#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <utility>
#include "UWB_message_type.h"
#include <geometry_msgs/Vector3.h>

class Kalman
{
public:
    struct State
    {
        uint32_t sys_time;
        Eigen::VectorXd x;
        Eigen::MatrixXd A, B, P_cov;
        Eigen::VectorXd u; //use input u to calculate x
		UWB_messages::ObservationMessage *obs_msg = nullptr; //if this sys_time has a observation.
        State(){}
        State(uint32_t sys_time, Eigen::VectorXd x, Eigen::MatrixXd P_cov, Eigen::VectorXd u):
            sys_time(sys_time), x(x), P_cov(P_cov), u(u){}
        ~State()
        {
            if(obs_msg != nullptr)
                delete obs_msg;
        }
    };
    std::deque<State> states;
    int queue_max_ms = 1000;
    Eigen::MatrixXd A_trans, B_input, Q_cov;
    Eigen::MatrixXd H_obs, R_noise;
    Eigen::MatrixXd P_cov_0;
    UWB_messages::ObservationMessage latest_msg;
	Kalman()
	{
		A_trans = Eigen::Matrix<double, 6, 6>::Zero();
		A_trans.block(0,0,3,3) = Eigen::Matrix<double, 3,3>::Identity();
		A_trans.block(0,3,3,3) = Eigen::Matrix<double, 3,3>::Identity();
		A_trans.block(3,3,3,3) = Eigen::Matrix<double, 3,3>::Identity();
		B_input = Eigen::Matrix<double, 6, 3>::Zero();
		B_input.topRows(3) = Eigen::Matrix<double, 3,3>::Identity() * 0.5;
		B_input.bottomRows(3) = Eigen::Matrix<double, 3,3>::Identity();
		Q_cov = 0.001 * Eigen::Matrix<double, 6,6>::Identity();
		P_cov_0 = 0.2 * Eigen::Matrix<double, 6,6>::Identity();
		Eigen::VectorXd R_noise_vec(6);
		R_noise_vec << 0.1, 0.1, 0.2, 0.1, 0.1, 0.1;
		R_noise = R_noise_vec.asDiagonal();
//		H_obs = Eigen::Matrix<double, 6,6>::Identity();
		H_obs = Eigen::Matrix<double, 6,6>::Zero();
		// H_obs = Eigen::Matrix<double, 3, 6>::Zero();
		 H_obs.block(0,0,3,3) = Eigen::Matrix<double, 3,3>::Identity();
	}


    Kalman(Eigen::MatrixXd A_trans, Eigen::MatrixXd B_input, Eigen::MatrixXd Q_cov, Eigen::MatrixXd P_cov_0):
        A_trans(A_trans), B_input(B_input), Q_cov(Q_cov), P_cov_0(P_cov_0)
        {
            if(A_trans.rows()!= A_trans.cols() || A_trans.rows() != Q_cov.rows() ||
                   Q_cov.rows() != Q_cov.cols() || A_trans.rows() != B_input.rows() ||
				   P_cov_0.rows() != A_trans.rows() || P_cov_0.rows() != P_cov_0.cols())
                   {
                       std::cout << "ERROR: A, B, Q, P matrix size ERROR!!!" << std::endl;
                       return;
                   }
        }

	void update(UWB_messages::ObservationMessage &msg)
	{
		int state_index = getStateIndexFromTime(msg.systemTime);
		if(state_index <= 0 )
		{
			std::cout << "states.size():" << states.size() <<  "  state_index <= 0" << std::endl;
			return;
		}
		if(states[state_index].obs_msg != nullptr)
        {
		    std::cout << "states[state_index].obs_msg != nullptr" << std::endl;
		    return;
        }
		states[state_index].obs_msg = new UWB_messages::ObservationMessage(msg);
        update(state_index);
    }
	geometry_msgs::Pose getPose()
	{
		geometry_msgs::Pose pose;
		pose = latest_msg.get_geometry_pose_world(); //orientation not filtered.
		if(states.size() != 0)
        {
            pose.position.x = states.back().x[0];
            pose.position.y = states.back().x[1];
            pose.position.z = states.back().x[2];
        }
		return pose;
	}

    geometry_msgs::Vector3 getVelocity()
    {
        geometry_msgs::Vector3 velocity;
        if(states.size() != 0)
        {
            velocity.x = states.back().x[3];
            velocity.y = states.back().x[4];
            velocity.z = states.back().x[5];
        }
        return velocity;
    }

	void predict(uint32_t sys_time, Eigen::VectorXd u)
    {
		if(u.size() != B_input.cols())
		{
			std::cout << "ERROR: u size " << u.size() <<" != B_input.cols() " << B_input.cols() << std::endl;
			return;
		}
		if(!states.empty() && sys_time - states.front().sys_time > queue_max_ms)
		{
			states.pop_front();
		}
        if(states.empty())
        {
            std::cout << "states.empty(), initialize to 0 and P_cov_0 in sys_time: " << sys_time <<  std::endl;
			states.emplace_back(sys_time, Eigen::VectorXd::Zero(A_trans.rows()), P_cov_0, u);
            return;
        }
        if(sys_time <= states.back().sys_time)
        {
            std::cout << "ERROR: input sys_time is earlier than states.back().sys_time!!" << std::endl;
            return;
        }
        State state;
		state.sys_time = sys_time;
        state.u = u;
		State& last_state = states.back();
		states.push_back(state);
		predict(states.back(), last_state);
    }

private:
    void update(int state_index)
    {
        const double pos_diff_ms = 300;
        int previous_state_index = getStateIndexFromTime(states[state_index].obs_msg->systemTime - pos_diff_ms); //for calculating velocity
        if(state_index <= 0 || previous_state_index <= 0)
        {
            std::cout << "states.size():" << states.size() <<  "  state_index <= 0" << std::endl;
            return;
        }
        predict(states[state_index], states[state_index - 1]);

        tf::Vector3 position_vec = states[state_index].obs_msg->get_tf_transform_world().getOrigin();
        Eigen::Vector3d vel_from_diff;

        vel_from_diff = (states[state_index].x.head(3) - states[previous_state_index].x.head(3)) * 1.0/
                                 (states[state_index].sys_time - states[previous_state_index].sys_time);

        Eigen::VectorXd z(6);
        z << position_vec.x(),position_vec.y(),position_vec.z(),
                vel_from_diff[0], vel_from_diff[1], vel_from_diff[2];

        tf::Transform tf_world_cam = states[state_index].obs_msg->panTiltAngle.get_tf_transform();
        tf::Matrix3x3 rotation_mat = tf_world_cam.getBasis();
        Eigen::Matrix<double, 3, 3> T_rot;
        T_rot << 	rotation_mat[0][0], rotation_mat[0][1], rotation_mat[0][2],
                rotation_mat[1][0], rotation_mat[1][1], rotation_mat[1][2],
                rotation_mat[2][0], rotation_mat[2][1], rotation_mat[2][2];

        Eigen::MatrixXd R_noise_rot = Eigen::Matrix<double, 6, 6>::Zero();
        R_noise_rot.block(0,0,3,3) = T_rot * R_noise.block(0,0,3,3) * T_rot.transpose();
        R_noise_rot.block(3,3,3,3) = T_rot * R_noise.block(3,3,3,3) * T_rot.transpose();

        Eigen::MatrixXd K_gain;
        K_gain = states[state_index].P_cov * H_obs.transpose() *
                 (H_obs * states[state_index].P_cov * H_obs.transpose() + R_noise_rot).inverse();

        states[state_index].x = states[state_index].x + K_gain*(z - H_obs * states[state_index].x);
        states[state_index].P_cov = states[state_index].P_cov - K_gain * H_obs * states[state_index].P_cov;

        while(state_index + 1 < states.size() && states[state_index + 1].obs_msg == nullptr)
        {
            predict(states[state_index + 1], states[state_index]);
            state_index++;
        }
        if(state_index + 1 < states.size())
            update(state_index + 1);
    }

	void predict(State& state, const State& last_state)
	{
		double delta_t = (state.sys_time - last_state.sys_time) / 1000.0;
		state.A = A_trans;
		state.B = B_input;
		state.A.block(0,3,3,3) *= delta_t;
		state.B.topRows(3) *= delta_t * delta_t;
		state.B.bottomRows(3) *= delta_t;
		state.x = state.A * last_state.x + state.B * state.u;
		state.P_cov = state.A * last_state.P_cov * state.A.transpose() + Q_cov;
	}


	int getStateIndexFromTime(uint32_t sys_time)
	{
		if(states.size() == 0) return -1;
		int front_index = 0, back_index = states.size() - 1;
		while(front_index < back_index)
		{
			int middle = (front_index + back_index)/2;
			if(states[middle].sys_time < sys_time) front_index = middle + 1;
			else back_index = middle;
		}
		if(front_index == 0) 
		{
			std::cout << "ERROR: getStateIndexFromTime( " << sys_time << 
						") is earlier than states.front().sys_time " <<
						states.front().sys_time << std::endl;
			return 0;
		}
		if(std::abs((double)sys_time - states[front_index].sys_time) < std::abs((double)sys_time - states[front_index - 1].sys_time))
			return front_index;
		else
			return front_index - 1;
	}

};

#endif