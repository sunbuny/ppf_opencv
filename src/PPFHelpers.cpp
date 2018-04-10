//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Author: Tolga Birdal <tbirdal AT gmail.com>

//#include "Precomp.h"
//#include <Eigen/Dense>
#include <PPFHelpers.h>


namespace ppf_match_3d {
    // TODO flann适配
//    typedef cv::flann::L2<float> Distance_32F;
//    typedef cv::flann::GenericIndex<Distance_32F> FlannIndex;
    typedef nanoflann::KDTreeEigenMatrixAdaptor< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor> >  FlannIndex;
    void shuffle(int *array, size_t n);

    Mat genRandomMat(int rows, int cols, double mean, double stddev);

    void getRandQuat(Vec4d &q);

    void getRandomRotation(Matx33d &R);

    void meanCovLocalPC(const Mat &pc, const int point_count, Matx33d &CovMat, Vec3d &Mean);

    void meanCovLocalPCInd(const Mat &pc, const int *Indices, const int point_count, Matx33d &CovMat, Vec3d &Mean);

    static std::vector<std::string> split(const std::string &text, char sep) {
        std::vector<std::string> tokens;
        std::size_t start = 0, end = 0;
        while ((end = text.find(sep, start)) != std::string::npos) {
            tokens.push_back(text.substr(start, end - start));
            start = end + 1;
        }
        tokens.push_back(text.substr(start));
        return tokens;
    }


    Mat loadPLYSimple(const char *fileName, int withNormals) {
        Mat cloud;
        int numVertices = 0;
        int numCols = 3;
        int has_normals = 0;

        std::ifstream ifs(fileName);

        if (!ifs.is_open())
            std::cerr << "Error opening input file: " << fileName <<std::endl;
//            CV_Error(Error::StsError, String("Error opening input file: ") + String(fileName) + "\n");

        std::string str;
        while (str.substr(0, 10) != "end_header") {
            std::vector<std::string> tokens = split(str, ' ');
            if (tokens.size() == 3) {
                if (tokens[0] == "element" && tokens[1] == "vertex") {
                    numVertices = atoi(tokens[2].c_str());
                } else if (tokens[0] == "property") {
                    if (tokens[2] == "nx" || tokens[2] == "normal_x") {
                        has_normals = -1;
                        numCols += 3;
                    } else if (tokens[2] == "r" || tokens[2] == "red") {
                        //has_color = true;
                        numCols += 3;
                    } else if (tokens[2] == "a" || tokens[2] == "alpha") {
                        //has_alpha = true;
                        numCols += 1;
                    }
                }
            } else if (tokens.size() > 1 && tokens[0] == "format" && tokens[1] != "ascii")
                std::cerr << "Cannot read file, only ascii ply format is currently supported..." << std::endl;
//                CV_Error(Error::StsBadArg, String("Cannot read file, only ascii ply format is currently supported..."));
            std::getline(ifs, str);
        }
        withNormals &= has_normals;

//        cloud = Mat(numVertices, withNormals ? 6 : 3, CV_32FC1);
        cloud = Mat(numVertices, withNormals ? 6 : 3);

        for (int i = 0; i < numVertices; i++) {
//            float *data = cloud.ptr<float>(i);
//            float *data = cloud.data()+i*(withNormals ? 6 : 3);
            float *data = cloud.row(i).data();
            int col = 0;
            for (; col < (withNormals ? 6 : 3); ++col) {
                ifs >> data[col];
            }
            // 弹掉不要的
            for (; col < numCols; ++col) {
                float tmp;
                ifs >> tmp;
            }
            if (withNormals) {
                // normalize to unit norm
                double norm = sqrt(data[3] * data[3] + data[4] * data[4] + data[5] * data[5]);
                if (norm > 0.00001) {
                    data[3] /= static_cast<float>(norm);
                    data[4] /= static_cast<float>(norm);
                    data[5] /= static_cast<float>(norm);
                }
            }
        }

        //cloud *= 5.0f;
        return cloud;
    }

    void writePLY(Mat PC, const char *FileName) {
        std::ofstream outFile(FileName);

        if (!outFile.is_open())
            std::cerr << "Error opening output file: " << FileName << std::endl;
//            CV_Error(Error::StsError, String("Error opening output file: ") + String(FileName) + "\n");

        ////
        // Header
        ////

        const int pointNum = (int) PC.rows();
        const int vertNum = (int) PC.cols();

        outFile << "ply" << std::endl;
        outFile << "format ascii 1.0" << std::endl;
        outFile << "element vertex " << pointNum << std::endl;
        outFile << "property float x" << std::endl;
        outFile << "property float y" << std::endl;
        outFile << "property float z" << std::endl;
        if (vertNum == 6) {
            outFile << "property float nx" << std::endl;
            outFile << "property float ny" << std::endl;
            outFile << "property float nz" << std::endl;
        }
        outFile << "end_header" << std::endl;

        ////
        // Points
        ////

        for (int pi = 0; pi < pointNum; ++pi) {
//            const float *point = PC.ptr<float>(pi);
            const float *point = PC.row(pi).data();
            outFile << point[0] << " " << point[1] << " " << point[2];

            if (vertNum == 6) {
                outFile << " " << point[3] << " " << point[4] << " " << point[5];
            }

            outFile << std::endl;
        }

        return;
    }

    void writePLYVisibleNormals(Mat PC, const char *FileName) {
        std::ofstream outFile(FileName);

        if (!outFile.is_open())
            std::cerr << "Error opening output file: " << FileName << std::endl;
//            CV_Error(Error::StsError, String("Error opening output file: ") + String(FileName) + "\n");

        ////
        // Header
        ////

        const int pointNum = (int) PC.rows();
        const int vertNum = (int) PC.cols();
        const bool hasNormals = vertNum == 6;

        outFile << "ply" << std::endl;
        outFile << "format ascii 1.0" << std::endl;
        outFile << "element vertex " << (hasNormals ? 2 * pointNum : pointNum) << std::endl;
        outFile << "property float x" << std::endl;
        outFile << "property float y" << std::endl;
        outFile << "property float z" << std::endl;
        if (hasNormals) {
            outFile << "property uchar red" << std::endl;
            outFile << "property uchar green" << std::endl;
            outFile << "property uchar blue" << std::endl;
        }
        outFile << "end_header" << std::endl;

        ////
        // Points
        ////

        for (int pi = 0; pi < pointNum; ++pi) {
//            const float *point = PC.ptr<float>(pi);
            const float *point = PC.row(pi).data();

            outFile << point[0] << " " << point[1] << " " << point[2];

            if (hasNormals) {
                outFile << " 127 127 127" << std::endl;
                outFile << point[0] + point[3] << " " << point[1] + point[4] << " " << point[2] + point[5];
                outFile << " 255 0 0";
            }

            outFile << std::endl;
        }

        return;
    }

    Mat samplePCUniform(Mat PC, int sampleStep) {
        int numRows = PC.rows() / sampleStep;
//        Mat sampledPC = Mat(numRows, PC.cols, PC.type());
        //TODO 需要测试
        Mat sampledPC(numRows, PC.cols());

        int c = 0;
        for (int i = 0; i < PC.rows() && c < numRows; i += sampleStep) {
//            PC.row(i).copyTo(sampledPC.row(c++));
            sampledPC.row(c++) = PC.row(i);
        }

        return sampledPC;
    }

    Mat samplePCUniformInd(Mat PC, int sampleStep, std::vector<int> &indices) {
        int numRows = round((double) PC.rows() / (double) sampleStep);
        indices.resize(numRows);
        Mat sampledPC(numRows, PC.cols());

        int c = 0;
        for (int i = 0; i < PC.rows() && c < numRows; i += sampleStep) {
            indices[c] = i;
//            PC.row(i).copyTo(sampledPC.row(c++));
            sampledPC.row(c++) = PC.row(i);
        }

        return sampledPC;
    }
//TODO Flann
//    void *indexPCFlann(Mat pc) {
//        Mat dest_32f;
////        pc.colRange(0, 3).copyTo(dest_32f);
//        dest_32f = pc.leftCols(3);
////        return new FlannIndex(dest_32f, cvflann::KDTreeSingleIndexParams(8));
////        return new FlannIndex(dest_32f, nanoflann::SearchParams(8));
//        FlannIndex* res = new FlannIndex(dest_32f, 8);
//        res->index->buildIndex();
//        return res;
//    }
//
//    void destroyFlann(void *flannIndex) {
//        delete ((FlannIndex *) flannIndex);
//    }
//
//// For speed purposes this function assumes that PC, Indices and Distances are created with continuous structures
//    void queryPCFlann(void *flannIndex, Mat &pc, Mat &indices, Mat &distances) {
//        queryPCFlann(flannIndex, pc, indices, distances, 1);
//    }
//
//    void queryPCFlann(void *flannIndex, Mat &pc, Mat &indices, Mat &distances, const int numNeighbors) {
//        Mat obj_32f;
////        pc.colRange(0, 3).copyTo(obj_32f);
//        obj_32f = pc.leftCols(3);
////        ((FlannIndex *) flannIndex)->findNeighbors(obj_32f, indices, distances, numNeighbors, cvflann::SearchParams(32));
//
//        // do a knn search
//        const size_t num_results = numNeighbors;
//        std::vector<size_t>   ret_indexes(num_results);
//        std::vector<float> out_dists_sqr(num_results);
//        nanoflann::KNNResultSet<float> resultSet(num_results);
//
//        resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
//        ((FlannIndex *) flannIndex)->index->findNeighbors(resultSet,)
//    }

// uses a volume instead of an octree
// TODO: Right now normals are required.
// This is much faster than sample_pc_octree
    Mat
    samplePCByQuantization(Mat pc, Vec2f &xrange, Vec2f &yrange, Vec2f &zrange, float sampleStep, int weightByCenter) {
        std::vector<std::vector<int> > map;

        int numSamplesDim = (int) (1.0 / sampleStep);

        float xr = xrange[1] - xrange[0];
        float yr = yrange[1] - yrange[0];
        float zr = zrange[1] - zrange[0];

        int numPoints = 0;

        map.resize((numSamplesDim + 1) * (numSamplesDim + 1) * (numSamplesDim + 1));

        // OpenMP might seem like a good idea, but it didn't speed this up for me
        //#pragma omp parallel for
        for (int i = 0; i < pc.rows(); i++) {
            const float *point = pc.row(i).data();

            // quantize a point
            const int xCell = (int) ((float) numSamplesDim * (point[0] - xrange[0]) / xr);
            const int yCell = (int) ((float) numSamplesDim * (point[1] - yrange[0]) / yr);
            const int zCell = (int) ((float) numSamplesDim * (point[2] - zrange[0]) / zr);
            const int index = xCell * numSamplesDim * numSamplesDim + yCell * numSamplesDim + zCell;

            /*#pragma omp critical
                {*/
            map[index].push_back(i);
            //  }
        }

        for (unsigned int i = 0; i < map.size(); i++) {
            numPoints += (map[i].size() > 0);
        }

//        Mat pcSampled = Mat(numPoints, pc.cols, CV_32F);
        Mat pcSampled(numPoints, pc.cols());
        int c = 0;

        for (unsigned int i = 0; i < map.size(); i++) {
            double px = 0, py = 0, pz = 0;
            double nx = 0, ny = 0, nz = 0;

            std::vector<int> curCell = map[i];
            int cn = (int) curCell.size();
            if (cn > 0) {
                if (weightByCenter) {
                    int xCell, yCell, zCell;
                    double xc, yc, zc;
                    double weightSum = 0;
                    zCell = i % numSamplesDim;
                    yCell = ((i - zCell) / numSamplesDim) % numSamplesDim;
                    xCell = ((i - zCell - yCell * numSamplesDim) / (numSamplesDim * numSamplesDim));

                    xc = ((double) xCell + 0.5) * (double) xr / numSamplesDim + (double) xrange[0];
                    yc = ((double) yCell + 0.5) * (double) yr / numSamplesDim + (double) yrange[0];
                    zc = ((double) zCell + 0.5) * (double) zr / numSamplesDim + (double) zrange[0];

                    for (int j = 0; j < cn; j++) {
                        const int ptInd = curCell[j];
                        float *point = pc.row(ptInd).data();
                        const double dx = point[0] - xc;
                        const double dy = point[1] - yc;
                        const double dz = point[2] - zc;
                        const double d = sqrt(dx * dx + dy * dy + dz * dz);
                        double w = 0;

                        if (d > EPS) {
                            // it is possible to use different weighting schemes.
                            // inverse weigthing was just good for me
                            // exp( - (distance/h)**2 )
                            //const double w = exp(-d*d);
                            w = 1.0 / d;
                        }

                        //float weights[3]={1,1,1};
                        px += w * (double) point[0];
                        py += w * (double) point[1];
                        pz += w * (double) point[2];
                        nx += w * (double) point[3];
                        ny += w * (double) point[4];
                        nz += w * (double) point[5];

                        weightSum += w;
                    }
                    px /= (double) weightSum;
                    py /= (double) weightSum;
                    pz /= (double) weightSum;
                    nx /= (double) weightSum;
                    ny /= (double) weightSum;
                    nz /= (double) weightSum;
                } else {
                    for (int j = 0; j < cn; j++) {
                        const int ptInd = curCell[j];
                        float *point = pc.row(ptInd).data();

                        px += (double) point[0];
                        py += (double) point[1];
                        pz += (double) point[2];
                        nx += (double) point[3];
                        ny += (double) point[4];
                        nz += (double) point[5];
                    }

                    px /= (double) cn;
                    py /= (double) cn;
                    pz /= (double) cn;
                    nx /= (double) cn;
                    ny /= (double) cn;
                    nz /= (double) cn;

                }

                float *pcData = pcSampled.row(c).data();
                pcData[0] = (float) px;
                pcData[1] = (float) py;
                pcData[2] = (float) pz;

                // normalize the normals
                double norm = sqrt(nx * nx + ny * ny + nz * nz);

                if (norm > EPS) {
                    pcData[3] = (float) (nx / norm);
                    pcData[4] = (float) (ny / norm);
                    pcData[5] = (float) (nz / norm);
                } else {
                    pcData[3] = 0.0f;
                    pcData[4] = 0.0f;
                    pcData[5] = 0.0f;
                }
                //#pragma omp atomic
                c++;

                curCell.clear();
            }
        }

        map.clear();
        return pcSampled;
    }

    void shuffle(int *array, size_t n) {
        size_t i;
        for (i = 0; i < n - 1; i++) {
            size_t j = i + rand() / (RAND_MAX / (n - i) + 1);
            int t = array[j];
            array[j] = array[i];
            array[i] = t;
        }
    }

// compute the standard bounding box
    void computeBboxStd(Mat pc, Vec2f &xRange, Vec2f &yRange, Vec2f &zRange) {
//        Mat pcPts = pc.colRange(0, 3);
        Mat pcPts = pc.leftCols(3);
        int num = pcPts.rows();

        float *points = (float *) pcPts.data();

        xRange[0] = points[0];
        xRange[1] = points[0];
        yRange[0] = points[1];
        yRange[1] = points[1];
        zRange[0] = points[2];
        zRange[1] = points[2];

        for (int ind = 0; ind < num; ind++) {
            const float *row = (float *) (pcPts.row(ind).data());
            const float x = row[0];
            const float y = row[1];
            const float z = row[2];

            if (x < xRange[0])
                xRange[0] = x;
            if (x > xRange[1])
                xRange[1] = x;

            if (y < yRange[0])
                yRange[0] = y;
            if (y > yRange[1])
                yRange[1] = y;

            if (z < zRange[0])
                zRange[0] = z;
            if (z > zRange[1])
                zRange[1] = z;
        }
    }

    Mat normalizePCCoeff(Mat pc, float scale, float *Cx, float *Cy, float *Cz, float *MinVal, float *MaxVal) {
        double minVal = 0, maxVal = 0;

        Mat x, y, z, pcn;
//        pc.col(0).copyTo(x);
//        pc.col(1).copyTo(y);
//        pc.col(2).copyTo(z);
        x = pc.col(0);
        y = pc.col(1);
        z = pc.col(2);

//        float cx = (float) cv::mean(x)[0];
//        float cy = (float) cv::mean(y)[0];
//        float cz = (float) cv::mean(z)[0];

        float cx = (float) x.colwise().sum()[0]/x.rows();
        float cy = (float) y.colwise().sum()[0]/y.rows();
        float cz = (float) z.colwise().sum()[0]/z.rows();

//        cv::minMaxIdx(pc, &minVal, &maxVal);

        x.array() -= cx;
        y.array() -= cy;
        z.array() -= cz;
//        pcn.create(pc.rows, 3, CV_32FC1);
        pcn.resize(pc.rows(),3);
//        x.copyTo(pcn.col(0));
//        y.copyTo(pcn.col(1));
//        z.copyTo(pcn.col(2));
        pcn.col(0) = x;
        pcn.col(1) = y;
        pcn.col(2) = z;
//todo need to be tested
//        cv::minMaxIdx(pcn, &minVal, &maxVal);
        minVal = pcn.minCoeff();
        maxVal = pcn.maxCoeff();

        pcn *= (float) scale/ ((float) maxVal - (float) minVal);

        *MinVal = (float) minVal;
        *MaxVal = (float) maxVal;
        *Cx = (float) cx;
        *Cy = (float) cy;
        *Cz = (float) cz;

        return pcn;
    }

    Mat transPCCoeff(Mat pc, float scale, float Cx, float Cy, float Cz, float MinVal, float MaxVal) {
        Mat x, y, z, pcn;
//        pc.col(0).copyTo(x);
//        pc.col(1).copyTo(y);
//        pc.col(2).copyTo(z);
        x = pc.col(0);
        y = pc.col(1);
        z = pc.col(2);

        x.array() -= Cx;
        y.array() -= Cy;
        z.array() -= Cz;

//        pcn.create(pc.rows, 3, CV_32FC1);
        pcn.resize(pc.rows(),3);
//        x.copyTo(pcn.col(0));
//        y.copyTo(pcn.col(1));
//        z.copyTo(pcn.col(2));
        pcn.col(0) = x;
        pcn.col(1) = y;
        pcn.col(2) = z;

        pcn = (float) scale * (pcn) / ((float) MaxVal - (float) MinVal);

        return pcn;
    }

    Mat transformPCPose(Mat pc, const Matx44d &Pose) {
//        Mat pct = Mat(pc.rows, pc.cols, CV_32F);
        Mat pct(pc.rows(),pc.cols());
        Matx33d R;
        Vec3d t;
        poseToRT(Pose, R, t);

#if defined _OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < pc.rows(); i++) {
//            const float *pcData = pc.ptr<float>(i);
            const float *pcData = pc.data()+i;
            const Vec3f n1(&pcData[3]);

            Vec4d p = Pose * Vec4d(pcData[0], pcData[1], pcData[2], 1);
//            Vec3d p2(p.val);
            Vec3d p2(p.head(3));
            // p2[3] should normally be 1
            if (fabs(p[3]) > EPS) {
//                Mat((1.0 / p[3]) * p2).reshape(1, 1).convertTo(pct.row(i).colRange(0, 3), CV_32F);
                //todo Mat中插值
                pct.row(i).head(3) =  (1.0 / (float)p[3]) *p2.cast<float>();
            }

            // If the point cloud has normals,
            // then rotate them as well
            if (pc.cols() == 6) {
                Vec3d n(n1.cast<double>()), n2;

                n2 = R * n;
//                double nNorm = cv::norm(n2);
                double nNorm = n2.norm();
                if (nNorm > EPS) {
//                    Mat((1.0 / nNorm) * n2).reshape(1, 1).convertTo(pct.row(i).colRange(3, 6), CV_32F);
                    pct.row(i).tail(3) = (1.0 / nNorm) * n2.cast<float>();
                }
            }
        }

        return pct;
    }

    Mat genRandomMat(int rows, int cols, double mean, double stddev) {
//        Mat meanMat = mean * Mat::ones(1, 1, type);
//        Mat sigmaMat = stddev * Mat::ones(1, 1, type);
//        RNG rng(time(0));

//        Mat matr(rows, cols, type);
//        rng.fill(matr, RNG::NORMAL, meanMat, sigmaMat);
        Mat matr(rows,cols);
        matr.setRandom();

        return matr;
    }

    void getRandQuat(Vec4d &q) {
        q[0] = (float) rand() / (float) (RAND_MAX);
        q[1] = (float) rand() / (float) (RAND_MAX);
        q[2] = (float) rand() / (float) (RAND_MAX);
        q[3] = (float) rand() / (float) (RAND_MAX);

//        q *= 1.0 / cv::norm(q);
        q *= 1.0 / q.norm();
        q[0] = fabs(q[0]);
    }

    void getRandomRotation(Matx33d &R) {
        Vec4d q;
        getRandQuat(q);
        quatToDCM(q, R);
    }

    void getRandomPose(Matx44d &Pose) {
        Matx33d R;
        Vec3d t;

        srand((unsigned int) time(0));
        getRandomRotation(R);

        t[0] = (float) rand() / (float) (RAND_MAX);
        t[1] = (float) rand() / (float) (RAND_MAX);
        t[2] = (float) rand() / (float) (RAND_MAX);

        rtToPose(R, t, Pose);
    }

    Mat addNoisePC(Mat pc, double scale) {
        Mat randT = genRandomMat(pc.rows(), pc.cols(), 0, scale);
        return randT + pc;
    }

/*
The routines below use the eigenvectors of the local covariance matrix
to compute the normals of a point cloud.
The algorithm uses FLANN and Joachim Kopp's fast 3x3 eigenvector computations
to improve accuracy and increase speed
Also, view point flipping as in point cloud library is implemented
*/

    void meanCovLocalPC(const Mat &pc, const int point_count, Matx33d &CovMat, Vec3d &Mean) {
//        cv::calcCovarMatrix(pc.rowRange(0, point_count), CovMat, Mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
        CovMat = (pc.transpose()*pc).cast<double>();
        Mean = (pc.colwise().sum()/(pc.rows() - 1)).cast<double>();
        CovMat *= 1.0 / (point_count - 1);
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                CovMat(j, k) -= Mean[j] * Mean[k];
    }

    void meanCovLocalPCInd(const Mat &pc, const int *Indices, const int point_count, Matx33d &CovMat, Vec3d &Mean) {
        int i, j, k;
// todo 一个非常巧妙的计算协方差矩阵方法，避免了两次遍历
//        CovMat = Matx33d::all(0);
//        Mean = Vec3d::all(0);
        CovMat = Matx33d::Zero();
        Mean   = Vec3d::Zero();
        for (i = 0; i < point_count; ++i) {
//            const float *cloud = pc.ptr<float>(Indices[i]);
            const float *cloud = pc.row(Indices[i]).data();
            for (j = 0; j < 3; ++j) {
                for (k = 0; k < 3; ++k)
                    CovMat(j, k) += cloud[j] * cloud[k];
                Mean[j] += cloud[j];
            }
        }
        Mean *= 1.0 / point_count;
        CovMat *= 1.0 / point_count;

        for (j = 0; j < 3; ++j)
            for (k = 0; k < 3; ++k)
                CovMat(j, k) -= Mean[j] * Mean[k];
    }

    int computeNormalsPC3d(const Mat &PC, Mat &PCNormals, const int NumNeighbors, const bool FlipViewpoint,
                           const Vec3f &viewpoint) {
        int i;

        if (PC.cols() != 3 && PC.cols() != 6) // 3d data is expected
        {
            //return -1;
//            CV_Error(cv::Error::BadImageSize, "PC should have 3 or 6 elements in its columns");
            std::cerr << "PC should have 3 or 6 elements in its columns" << std::endl;
        }

//        PCNormals.create(PC.rows, 6, CV_32F);
        PCNormals.resize(PC.rows(),6);
//        Mat PCInput = PCNormals.colRange(0, 3);
        PCNormals.leftCols(3) = PC.leftCols(3);
        Mat PCInput = PC.leftCols(3);
//        Mat Distances(PC.rows, NumNeighbors, CV_32F);
//        Mat Indices(PC.rows, NumNeighbors, CV_32S);

//        PC.rowRange(0, PC.rows).colRange(0, 3).copyTo(PCNormals.rowRange(0, PC.rows).colRange(0, 3));
//
//        void *flannIndex = indexPCFlann(PCInput);
//
//        queryPCFlann(flannIndex, PCInput, Indices, Distances, NumNeighbors);
//        destroyFlann(flannIndex);
//        flannIndex = 0;

        FlannIndex mat_index(PCInput, 8 /* max leaf */ );
        mat_index.index->buildIndex(); // 初始化flann



#if defined _OPENMP
#pragma omp parallel for
#endif
        for (i = 0; i < PC.rows(); i++) {
            Matx33d C;
            Vec3d mu;
//            const int *indLocal = Indices.ptr<int>(i);
            // do a knn search
            const size_t num_results = NumNeighbors;
            std::vector<size_t >   ret_indexes(num_results); // 索引
            std::vector<float> out_dists_sqr(num_results);  // 距离

            nanoflann::KNNResultSet<float> resultSet(num_results);

            resultSet.init(&ret_indexes[0], &out_dists_sqr[0] );
            std::vector<float> query_pt(3);
            Eigen::VectorXf::Map(&query_pt[0],3) = PC.row(i);
//            std::vector<float> query_pt = PC.row(i);
            mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(32));

            // compute covariance matrix
            meanCovLocalPCInd(PCNormals, (int*)ret_indexes.data(), NumNeighbors, C, mu);

            // eigenvectors of covariance matrix
//            Mat eigVect, eigVal;
//            Eigen::EigenSolver<Mat> eig(C);
//            PCNormals.row(i).tail(3) = eig.eigenvectors().col(0);
//            eig.eigenvalues();
//            eigen(C, eigVal, eigVect);
//            eigVect.row(2).convertTo(PCNormals.row(i).colRange(3, 6), CV_32F);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
            solver.compute(C, Eigen::ComputeEigenvectors);
            PCNormals.row(i).tail(3) = solver.eigenvectors().col(0).cast<float>();
            if (FlipViewpoint) {
                Vec3f nr(PCNormals.row(i).data()+ 3);
                Vec3f pci(PCNormals.row(i).data());
                flipNormalViewpoint(pci, viewpoint, nr);
//                Mat(nr).reshape(1, 1).copyTo(PCNormals.row(i).colRange(3, 6));
                PCNormals.row(i).tail(3) = nr;
            }
        }

        return 1;
    }

} // namespace ppf_match_3d


