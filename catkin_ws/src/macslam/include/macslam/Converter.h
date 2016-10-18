/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

//C++
#include <opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <Eigen/Dense>

//MCP SLAM
#include <helper/estd.h>

#include "thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <macslam_msgs/machCvKeyPoint.h>

using namespace std;

namespace macslam
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);    

    static macslam_msgs::machCvKeyPoint toCvKeyPointMsg(const cv::KeyPoint &kp);
    static cv::KeyPoint fromCvKeyPointMsg(const macslam_msgs::machCvKeyPoint &Msg);

//    template<typename TC, typename TM> static void ContainerToMessageVector(TC Container, TM MsgVect);
    template<typename TMes, typename TMat> static void CvMatToMsgArrayFixedSize(cv::Mat &InMat, TMes &MsgArray)
    {
//        cout << "InMat: rows|cols: " << InMat.rows << "|" << InMat.cols << endl;
//        cout << "MsgArray.size(): " << MsgArray.size() << endl;

        if((!(((InMat.cols)*(InMat.rows))==MsgArray.size())) && (!((InMat.cols*InMat.rows)==0)))
        {
            cout << "In \"Converter::CvMatToMsgArrayFixedSize(cv::Mat &InMat, TMes &MsgArray)\": dimension mismatch" << endl;
            throw estd::infrastructure_ex();
        }

        if(!(InMat.rows || InMat.cols)) //0D --> fill with zeros
        {
//            cout << "Null filling" << endl;
            for(int idx=0;idx<MsgArray.size();++idx) MsgArray[idx] = static_cast<TMat>(0.0);
        }
        else if(InMat.cols == 1) //1D
        {            
            for(int idx=0;idx<MsgArray.size();++idx) MsgArray[idx] = InMat.at<TMat>(idx);
        }
        else
        {
            for(int idx=0;idx<InMat.rows;++idx)
                for(int idy=0;idy<InMat.cols;++idy)
                    MsgArray[idx*InMat.rows+idy] = InMat.at<TMat>(idx,idy);
        }
    }
    template<typename TMes, typename TMat> static void CvMatToMsgArrayFixedSize(const cv::Mat &InMat, TMes &MsgArray)
    {
//        cout << "InMat: rows|cols: " << InMat.rows << "|" << InMat.cols << endl;
//        cout << "MsgArray.size(): " << MsgArray.size() << endl;

        if((!(((InMat.cols)*(InMat.rows))==MsgArray.size())) && (!((InMat.cols*InMat.rows)==0)))
        {
            cout << "In \"Converter::CvMatToMsgArrayFixedSize(cv::Mat &InMat, TMes &MsgArray)\": dimension mismatch" << endl;
            throw estd::infrastructure_ex();
        }

        if(!(InMat.rows || InMat.cols)) //0D --> fill with zeros
        {
//            cout << "Null filling" << endl;
            for(int idx=0;idx<MsgArray.size();++idx) MsgArray[idx] = static_cast<TMat>(0.0);
        }
        else if(InMat.cols == 1) //1D
        {
            for(int idx=0;idx<MsgArray.size();++idx) MsgArray[idx] = InMat.at<TMat>(idx);
        }
        else
        {
            for(int idx=0;idx<InMat.rows;++idx)
                for(int idy=0;idy<InMat.cols;++idy)
                    MsgArray[idx*InMat.rows+idy] = InMat.at<TMat>(idx,idy);
        }
    }

    template<typename TMes, typename TMat> static void MsgArrayFixedSizeToCvMat(cv::Mat &InMat, TMes &MsgArray)
    {
        if(MsgArray.size() == 0)
        {
            cout << "In \"Converter::MsgArrayFixedSizeToCvMat(...)\": empty input array" << endl;
//            return;
            throw estd::infrastructure_ex();
        }
        if(InMat.cols == 1) //1D
        {
            for(int idx=0;idx<MsgArray.size();++idx) InMat.at<TMat>(idx) = MsgArray[idx];
        }
        else
        {
            for(int idx=0;idx<InMat.rows;++idx)
                for(int idy=0;idy<InMat.cols;++idy)
                    InMat.at<TMat>(idx,idy)=MsgArray[idx*InMat.rows+idy];
        }

    }
    template<typename TMes, typename TMat> static void MsgArrayFixedSizeToCvMat(cv::Mat &InMat, const TMes &MsgArray)
    {
        if(MsgArray.size() == 0)
        {
            cout << "In \"Converter::MsgArrayFixedSizeToCvMat(...)\": empty input array" << endl;
//            return;
            throw estd::infrastructure_ex();
        }
        if(InMat.cols == 1) //1D
        {
            for(int idx=0;idx<MsgArray.size();++idx) InMat.at<TMat>(idx) = MsgArray[idx];
        }
        else
        {
            for(int idx=0;idx<InMat.rows;++idx)
                for(int idy=0;idy<InMat.cols;++idy)
                    InMat.at<TMat>(idx,idy)=MsgArray[idx*InMat.rows+idy];
        }

    }

};



}// ns

#endif // CONVERTER_H
