#pragma once


#include "Pose3D.h"
#include <Eigen/SVD>
#include <vector>

namespace ppf_match_3d {

//! @addtogroup surface_matching
//! @{

/**
* @brief This class implements a very efficient and robust variant of the iterative closest point (ICP) algorithm.
* The task is to register a 3D model (or point cloud) against a set of noisy target data. The variants are put together
* by myself after certain tests. The task is to be able to match partial, noisy point clouds in cluttered scenes, quickly.
* You will find that my emphasis is on the performance, while retaining the accuracy.
* This implementation is based on Tolga Birdal's MATLAB implementation in here:
* http://www.mathworks.com/matlabcentral/fileexchange/47152-icp-registration-using-efficient-variants-and-multi-resolution-scheme
* The main contributions come from:
* 1. Picky ICP:
* http://www5.informatik.uni-erlangen.de/Forschung/Publikationen/2003/Zinsser03-ARI.pdf
* 2. Efficient variants of the ICP Algorithm:
* http://docs.happycoders.org/orgadoc/graphics/imaging/fasticp_paper.pdf
* 3. Geometrically Stable Sampling for the ICP Algorithm: https://graphics.stanford.edu/papers/stabicp/stabicp.pdf
* 4. Multi-resolution registration:
* http://www.cvl.iis.u-tokyo.ac.jp/~oishi/Papers/Alignment/Jost_MultiResolutionICP_3DIM03.pdf
* 5. Linearization of Point-to-Plane metric by Kok Lim Low:
* https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
*/
    class ICP {
    public:

        enum {
            ICP_SAMPLING_TYPE_UNIFORM = 0,
            ICP_SAMPLING_TYPE_GELFAND = 1
        };

        ICP() {
            m_tolerance = 0.005f;
            m_rejectionScale = 2.5f;
            m_maxIterations = 250;
            m_numLevels = 6;
            m_sampleType = ICP_SAMPLING_TYPE_UNIFORM;
            m_numNeighborsCorr = 1;
        }

        virtual ~ICP() {}

        /**
           *  \brief ICP constructor with default arguments.
           *  @param [in] iterations
           *  @param [in] tolerence Controls the accuracy of registration at each iteration of ICP.
           *  @param [in] rejectionScale Robust outlier rejection is applied for robustness. This value
                  actually corresponds to the standard deviation coefficient. Points with
                  rejectionScale * &sigma are ignored during registration.
           *  @param [in] numLevels Number of pyramid levels to proceed. Deep pyramids increase speed but
                  decrease accuracy. Too coarse pyramids might have computational overhead on top of the
                  inaccurate registrtaion. This parameter should be chosen to optimize a balance. Typical
                  values range from 4 to 10.
           *  @param [in] sampleType Currently this parameter is ignored and only uniform sampling is
                  applied. Leave it as 0.
           *  @param [in] numMaxCorr Currently this parameter is ignored and only PickyICP is applied. Leave it as 1.
           */
        ICP(const int iterations, const float tolerence = 0.05f, const float rejectionScale = 2.5f,
            const int numLevels = 6, const int sampleType = ICP::ICP_SAMPLING_TYPE_UNIFORM, const int numMaxCorr = 1) {
            m_tolerance = tolerence;
            m_numNeighborsCorr = numMaxCorr;
            m_rejectionScale = rejectionScale;
            m_maxIterations = iterations;
            m_numLevels = numLevels;
            m_sampleType = sampleType;
        }

        /**
           *  \brief Perform registration
           *
           *  @param [in] srcPC The input point cloud for the model. Expected to have the normals (Nx6). Currently,
           *  CV_32F is the only supported data type.
           *  @param [in] dstPC The input point cloud for the scene. It is assumed that the model is registered on the scene. Scene remains static. Expected to have the normals (Nx6). Currently, CV_32F is the only supported data type.
           *  @param [out] residual The output registration error.
           *  @param [out] pose Transformation between srcPC and dstPC.
           *  \return On successful termination, the function returns 0.
           *
           *  \details It is assumed that the model is registered on the scene. Scene remains static, while the model transforms. The output poses transform the models onto the scene. Because of the point to plane minimization, the scene is expected to have the normals available. Expected to have the normals (Nx6).
           */
        int registerModelToScene(const Mat &srcPC, const Mat &dstPC,  double &residual,  Matx44d &

        pose);

        /**
           *  \brief Perform registration with multiple initial poses
           *
           *  @param [in] srcPC The input point cloud for the model. Expected to have the normals (Nx6). Currently,
           *  CV_32F is the only supported data type.
           *  @param [in] dstPC The input point cloud for the scene. Currently, CV_32F is the only supported data type.
           *  @param [in,out] poses Input poses to start with but also list output of poses.
           *  \return On successful termination, the function returns 0.
           *
           *  \details It is assumed that the model is registered on the scene. Scene remains static, while the model transforms. The output poses transform the models onto the scene. Because of the point to plane minimization, the scene is expected to have the normals available. Expected to have the normals (Nx6).
           */
        int registerModelToScene(const Mat &srcPC, const Mat &dstPC, std::vector<Pose3DPtr> &poses);

    private:
        float m_tolerance;
        int m_maxIterations;
        float m_rejectionScale;
        int m_numNeighborsCorr;
        int m_numLevels;
        int m_sampleType;

    };

//! @}

} // namespace ppf_match_3d



