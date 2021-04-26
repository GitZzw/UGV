//
// Created by ljy on 2021/2/1.
//

#ifndef SRC_POSEMEAN_H
#define SRC_POSEMEAN_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <utility>
#include "UWB_message_type.h"
#include <geometry_msgs/Vector3.h>


class PoseMean
{
public:
    Eigen::Matrix3Xd position_sum;
    Eigen::Matrix4Xd orientation_sum;
    PoseMean():position_sum(3,0),orientation_sum(4,0){}
    void addPosition(Eigen::Vector3d position)
    {
        if(position.sum() != position.sum())
        {
            std::cout << "position is NAN, ignored." << std::endl;
            return;
        }
        position_sum.conservativeResize(3, position_sum.cols()+1);
        position_sum.col(position_sum.cols()-1) = position;
    }
    void addOrientation(Eigen::Vector4d orientation)
    {
        if(orientation.sum() != orientation.sum())
        {
            std::cout << "orientation is NAN, ignored." << std::endl;
            return;
        }
        orientation_sum.conservativeResize(4, orientation_sum.cols()+1);
        orientation_sum.col(orientation_sum.cols()-1) = orientation;
    }

    void clearPositionSum()
    {
        position_sum = Eigen::Matrix3Xd(3,0);
    }

    void clearOrientationSum()
    {
        orientation_sum = Eigen::Matrix4Xd(4,0);
    }
    void clear()
    {
        clearPositionSum();
        clearOrientationSum();
    }

    int getDataSize()
    {
        return std::max(position_sum.cols(), orientation_sum.cols());
    }

    Eigen::Vector3d getPositionMean()
    {
        return position_sum.rowwise().mean().eval();
    }

    Eigen::Vector4d getOrientationMean()
    {
        Eigen::EigenSolver<Eigen::Matrix4d> es(orientation_sum * orientation_sum.transpose());
        Eigen::MatrixXcd evecs = es.eigenvectors();//获取矩阵特征向量4*4，这里定义的MatrixXcd必须有c，表示获得的是complex复数矩阵
        Eigen::MatrixXcd evals = es.eigenvalues();//获取矩阵特征值 4*1
        Eigen::MatrixXd evalsReal;//注意这里定义的MatrixXd里没有c
        evalsReal=evals.real();//获取特征值实数部分
        Eigen::MatrixXf::Index evalsMax;
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);//得到最大特征值的位置
        Eigen::Vector4d orientation_averaged;
        orientation_averaged << evecs.real()(0, evalsMax),
                evecs.real()(1, evalsMax),
                evecs.real()(2, evalsMax),
                evecs.real()(3, evalsMax);//得到对应特征向量
        if(orientation_sum.rowwise().mean().coeff(3,0) * orientation_averaged[3]< 0)
        {
            orientation_averaged *= -1;
        }
        return orientation_averaged.eval();
    }

    Eigen::MatrixXd getPositionCovarianceMatrix()
    {
        Eigen::Vector3d position_averaged = getPositionMean();
        Eigen::VectorXd ones = Eigen::VectorXd::Ones(position_sum.cols());
        Eigen::MatrixXd cov_matrix;
        cov_matrix = (position_sum - position_averaged * ones.transpose())
                     * (position_sum - position_averaged * ones.transpose()).transpose()
                     /(position_sum.cols() - 1);
        return cov_matrix;
    }

};


#endif //SRC_POSEMEAN_H
