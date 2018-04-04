//
// Created by sun on 18-4-4.
//

#pragma once


#include <vector>
#include "Pose3D.h"
#include "THashInt.h"


namespace ppf_match_3d {

//! @addtogroup surface_matching
//! @{

/**
  * @brief Struct, holding a node in the hashtable
  */
    typedef struct THash {
        int id;
        int i, ppfInd;
    } THash;

/**
  * @brief Class, allowing the load and matching 3D models.
  * Typical Use:
  * @code
  * // Train a model
  * ppf_match_3d::PPF3DDetector detector(0.05, 0.05);
  * detector.trainModel(pc);
  * // Search the model in a given scene
  * vector<Pose3DPtr> results;
  * detector.match(pcTest, results, 1.0/5.0,0.05);
  * @endcode
  */
    class  PPF3DDetector
            {
            public:

            /**
             * \brief Empty constructor. Sets default arguments
             */
            PPF3DDetector();

            /**
              * Constructor with arguments
              * @param [in] relativeSamplingStep Sampling distance relative to the object's diameter. Models are first sampled uniformly in order to improve efficiency. Decreasing this value leads to a denser model, and a more accurate pose estimation but the larger the model, the slower the training. Increasing the value leads to a less accurate pose computation but a smaller model and faster model generation and matching. Beware of the memory consumption when using small values.
              * @param [in] relativeDistanceStep The discretization distance of the point pair distance relative to the model's diameter. This value has a direct impact on the hashtable. Using small values would lead to too fine discretization, and thus ambiguity in the bins of hashtable. Too large values would lead to no discrimination over the feature vectors and different point pair features would be assigned to the same bin. This argument defaults to the value of RelativeSamplingStep. For noisy scenes, the value can be increased to improve the robustness of the matching against noisy points.
              * @param [in] numAngles Set the discretization of the point pair orientation as the number of subdivisions of the angle. This value is the equivalent of RelativeDistanceStep for the orientations. Increasing the value increases the precision of the matching but decreases the robustness against incorrect normal directions. Decreasing the value decreases the precision of the matching but increases the robustness against incorrect normal directions. For very noisy scenes where the normal directions can not be computed accurately, the value can be set to 25 or 20.
              */
            PPF3DDetector(const double relativeSamplingStep, const double relativeDistanceStep=0.05, const double numAngles=30);

            virtual ~PPF3DDetector();

            /**
              *  Set the parameters for the search
              *  @param [in] positionThreshold Position threshold controlling the similarity of translations. Depends on the units of calibration/model.
              *  @param [in] rotationThreshold Position threshold controlling the similarity of rotations. This parameter can be perceived as a threshold over the difference of angles
              *  @param [in] useWeightedClustering The algorithm by default clusters the poses without weighting. A non-zero value would indicate that the pose clustering should take into account the number of votes as the weights and perform a weighted averaging instead of a simple one.
              */
            void setSearchParams(const double positionThreshold=-1, const double rotationThreshold=-1, const bool useWeightedClustering=false);

            /**
              *  \brief Trains a new model.
              *
              *  @param [in] Model The input point cloud with normals (Nx6)
              *
              *  \details Uses the parameters set in the constructor to downsample and learn a new model. When the model is learnt, the instance gets ready for calling "match".
              */
            void trainModel(const Mat& Model);

            /**
              *  \brief Matches a trained model across a provided scene.
              *
              *  @param [in] scene Point cloud for the scene
              *  @param [out] results List of output poses
              *  @param [in] relativeSceneSampleStep The ratio of scene points to be used for the matching after sampling with relativeSceneDistance. For example, if this value is set to 1.0/5.0, every 5th point from the scene is used for pose estimation. This parameter allows an easy trade-off between speed and accuracy of the matching. Increasing the value leads to less points being used and in turn to a faster but less accurate pose computation. Decreasing the value has the inverse effect.
              *  @param [in] relativeSceneDistance Set the distance threshold relative to the diameter of the model. This parameter is equivalent to relativeSamplingStep in the training stage. This parameter acts like a prior sampling with the relativeSceneSampleStep parameter.
              */
            void match(const Mat& scene, std::vector<Pose3DPtr> &results, const double relativeSceneSampleStep=1.0/5.0, const double relativeSceneDistance=0.03);

            void read(const FileNode& fn);
            void write(FileStorage& fs) const;

            protected:

            double angle_step, angle_step_radians, distance_step;
            double sampling_step_relative, angle_step_relative, distance_step_relative;
            Mat sampled_pc, ppf;
            int num_ref_points;
            hashtable_int* hash_table;
            THash* hash_nodes;

            double position_threshold, rotation_threshold;
            bool use_weighted_avg;

            int scene_sample_step;

            void clearTrainingModels();

            private:
            void computePPFFeatures(const Vec3d& p1, const Vec3d& n1,
            const Vec3d& p2, const Vec3d& n2,
            Vec4d& f);

            bool matchPose(const Pose3D& sourcePose, const Pose3D& targetPose);

            void clusterPoses(std::vector<Pose3DPtr>& poseList, int numPoses, std::vector<Pose3DPtr> &finalPoses);

            bool trained;
            };

//! @}

} // namespace ppf_match_3d

