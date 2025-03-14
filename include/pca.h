/******************************************************************************
Copyright 2022 RoboSense All rights reserved.
Suteng Innovation Technology Co., Ltd. www.robosense.ai

This software is provided to you directly by RoboSense and might
only be used to access RoboSense LiDAR. Any compilation,
modification, exploration, reproduction and redistribution are
restricted without RoboSense's prior consent.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef PCA_H
#define PCA_H

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

template <typename Type>
class PcaT
{
public:
  enum class SortOrder : uint8_t
  {
    NONE,
    ASCENDING,
    DESCENDING
  };

  PcaT()  = default;
  ~PcaT() = default;

  void setInput(const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& _input_matrix)
  {
    input_matrix_ = _input_matrix;
  }

  void compute(SortOrder _order = SortOrder::ASCENDING)
  {
    // Compute a centered version of input matrix
    centered_matrix_ = input_matrix_.rowwise() - input_matrix_.colwise().mean();
    mean_matrix_     = input_matrix_ - centered_matrix_;
    //
    // Compute covariance matrix
    covariance_matrix_ = (centered_matrix_.adjoint() * centered_matrix_) / (Type)(input_matrix_.rows() - 1);
    //
    // Use SelfAdjointEigenSolver to get eigen values and eigen vectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>> eigen_solver(covariance_matrix_);
    eigen_values_  = eigen_solver.eigenvalues();
    eigen_vectors_ = eigen_solver.eigenvectors();

    if (_order != SortOrder::NONE)
      sortEigenVectors(_order);

    // Projection is w * X'
    const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& w = eigen_vectors_.adjoint();
    projection_matrix_ = w.topRows(input_matrix_.cols()) * centered_matrix_.adjoint();
  }

  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> reprojection()
  {
    return projection_matrix_.transpose() * eigen_vectors_ + getMean();
    //return (eigen_vectors_ * projection_matrix_ + get_mean().transpose()).transpose();
  }

  const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& getInputMatrix() const { return input_matrix_; }
  const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& getCenteredMatrix() const { return centered_matrix_; }
  const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& getCovarianceMatrix() const { return covariance_matrix_; }
  const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& getProjectionMatrix() const { return projection_matrix_; }
  const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& getMean() const { return mean_matrix_; }

  const Eigen::Matrix<Type, 1, Eigen::Dynamic>& getEigenValues() const { return eigen_values_; };
  const Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>& getEigenVectors() const { return eigen_vectors_; }

private:
  //
  // Private Methods
  //

  void sortEigenVectors(SortOrder _order = SortOrder::ASCENDING)
  {
    // Stuff below is done to sort eigen values. This can be done in other ways too.
    std::vector<std::pair<int, int>> eigen_value_index_vector;
    for (int i = 0; i < eigen_values_.size(); ++i)
    {
      eigen_value_index_vector.push_back(std::make_pair(eigen_values_[i], i));
    }

    if (_order == SortOrder::ASCENDING)
      std::sort(std::begin(eigen_value_index_vector), std::end(eigen_value_index_vector),
                std::greater<std::pair<int, int>>());
    else
      std::sort(std::begin(eigen_value_index_vector), std::end(eigen_value_index_vector),
                std::less<std::pair<int, int>>());

    auto sorted_eigen_values = Eigen::Matrix<Type, 1, Eigen::Dynamic>(eigen_values_.cols());
    auto sorted_eigen_vectors =
      Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>(eigen_vectors_.rows(), eigen_vectors_.cols());
    for (int i = 0; i < eigen_values_.size(); ++i)
    {
      sorted_eigen_values[i] =
        eigen_values_[eigen_value_index_vector[i].second];  //can also be eigen_value_index_vector[i].first
      sorted_eigen_vectors.col(i) = eigen_vectors_.col(eigen_value_index_vector[i].second);
    }

    eigen_values_  = sorted_eigen_values;
    eigen_vectors_ = sorted_eigen_vectors;
  }

  //
  // Private Attributes
  //

  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> input_matrix_;
  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> centered_matrix_;
  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> mean_matrix_;
  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> covariance_matrix_;
  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> projection_matrix_;

  Eigen::Matrix<Type, 1, Eigen::Dynamic> eigen_values_;
  Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> eigen_vectors_;
};

#endif