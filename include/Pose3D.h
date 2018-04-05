#pragma once

#include <vector>
#include <string>
#include <memory>
#include <iostream>
//#include "Precomp.h"
#include <c_utils.h>

//TODO 转为eigen


namespace ppf_match_3d {

//! @addtogroup surface_matching
//! @{

    class Pose3D;

    typedef std::shared_ptr<Pose3D> Pose3DPtr;

    class PoseCluster3D;

    typedef std::shared_ptr<PoseCluster3D> PoseCluster3DPtr;

/**
* @brief Class, allowing the storage of a pose. The data structure stores both
* the quaternions and the matrix forms. It supports IO functionality together with
* various helper methods to work with poses
*
*/
    class Pose3D {
    public:
        Pose3D() {
            alpha = 0;
            modelIndex = 0;
            numVotes = 0;
            residual = 0;

            pose = Eigen::Matrix4d::Zero();
        }

        Pose3D(double Alpha, size_t ModelIndex = 0, size_t NumVotes = 0) {
            alpha = Alpha;
            modelIndex = ModelIndex;
            numVotes = NumVotes;
            residual = 0;

            pose = Eigen::Matrix4d::Zero();
        }

        /**
         *  \brief Updates the pose with the new one
         *  \param [in] NewPose New pose to overwrite
         */
        void updatePose(Matx44d &NewPose);

        /**
         *  \brief Updates the pose with the new one
         */
        void updatePose(Matx33d &NewR, Vec3d &NewT);

        /**
         *  \brief Updates the pose with the new one, but this time using quaternions to represent rotation
         */
        void updatePoseQuat(Vec4d &Q, Vec3d &NewT);

        /**
         *  \brief Left multiplies the existing pose in order to update the transformation
         *  \param [in] IncrementalPose New pose to apply
         */
        void appendPose(Matx44d &IncrementalPose);

        void printPose();

        Pose3DPtr clone();

        int writePose(FILE *f);

        int readPose(FILE *f);

        int writePose(const std::string &FileName);

        int readPose(const std::string &FileName);

        virtual ~Pose3D() {}

        double alpha, residual;
        size_t modelIndex, numVotes;
        Matx44d pose;

        double angle;
        Vec3d t;
        Vec4d q;
    };

/**
* @brief When multiple poses (see Pose3D) are grouped together (contribute to the same transformation)
* pose clusters occur. This class is a general container for such groups of poses. It is possible to store,
* load and perform IO on these poses.
*/
    class PoseCluster3D {
    public:
        PoseCluster3D() {
            numVotes = 0;
            id = 0;
        }

        PoseCluster3D(Pose3DPtr newPose) {
            poseList.clear();
            poseList.push_back(newPose);
            numVotes = newPose->numVotes;
            id = 0;
        }

        PoseCluster3D(Pose3DPtr newPose, int newId) {
            poseList.push_back(newPose);
            this->numVotes = newPose->numVotes;
            this->id = newId;
        }

        virtual ~PoseCluster3D() {}

        /**
         *  \brief Adds a new pose to the cluster. The pose should be "close" to the mean poses
         *  in order to preserve the consistency
         *  \param [in] newPose Pose to add to the cluster
         */
        void addPose(Pose3DPtr newPose);

        int writePoseCluster(FILE *f);

        int readPoseCluster(FILE *f);

        int writePoseCluster(const std::string &FileName);

        int readPoseCluster(const std::string &FileName);

        std::vector<Pose3DPtr> poseList;
        size_t numVotes;
        int id;
    };

//! @}

} // namespace ppf_match_3d
