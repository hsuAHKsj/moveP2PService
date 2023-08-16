#include <globalPlanInterface.h>
#include <iostream>
#include <fstream>

void printXYThetaList(const XYThetaList& xyThetaList)
{
    // 打印计算结果
    for (size_t i = 0; i < xyThetaList.x.size(); i++) {
        std::cout << "[" << xyThetaList.prop[i] << "]: x: " << xyThetaList.x[i] << ", y: " << xyThetaList.y[i]
        << ", t: " << xyThetaList.theta[i] <<  std::endl;
    }
}

int main()
{
    bool success = initGlobalPlanningAlg("../config");

    // 设置交融半径
    double blendingRaduis = 0.2;
    settingRadius(blendingRaduis);

    // 计算参数化路径信息
    const XYThetaList& xytList = computeGloalPath("../config/startEndPoints.json");

    // 计算插值后的轨迹运动
    std::vector<OutputData> outputDataList = computeReedsSheppPaths(xytList);

    printXYThetaList(xytList);

    std::ofstream f("../data/output.txt");
    if (f.is_open()) {
        for (int i = 0; i < outputDataList.size(); i++) {
            const OutputData& output = outputDataList[i];
            f << output.path << std::endl;
        }
        f.close(); 
    }
    else {
        std::cout << "Failed to open file for writing." << std::endl;
    }
}