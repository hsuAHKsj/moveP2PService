#pragma once
#include <vector>
#include <Eigen/Dense>

using namespace std;
struct OutputData {
    Eigen::VectorXd lengths;
    Eigen::VectorXi types;
    Eigen::MatrixXd path;
};

struct XYThetaList {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<char> prop;
};
