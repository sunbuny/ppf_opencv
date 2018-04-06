#include <iostream>
#include <iomanip>
//#include <c_utils.h>
#include <hash_murmur.h>
#include <Pose3D.h>


#pragma ide diagnostic ignored "CannotResolve"
int main() {
    Vec3d v,v1;
    v.fill(1);
    v1.fill(2);
    ppf_match_3d::TNormalize3(v);
    ppf_match_3d::TNormalize3(v1);
    std::cout << "TNormalize3 " << v << std::endl;
    double ans = ppf_match_3d::TAngle3Normalized(v,v);
    std::cout << "TAngle3Normalized " << ans << std::endl;
    std::cout << std::setprecision(20) <<  "v.dot(v1) " << v.dot(v1) << std::endl;

    Matx33d R = Eigen::AngleAxisd(M_PI/3,Eigen::Vector3d::UnitX()).matrix();
    Vec3d   t;
    t.fill(1);
    Matx44d pose;
    ppf_match_3d::rtToPose(R,t,pose);
    std::cout << "rtToPose: " << pose << std::endl;
    Matx33d R_tmp;
    ppf_match_3d::poseToR(pose,R_tmp);
    std::cout << "poseToR: " << R_tmp <<std::endl;
    Vec3d t_tmp;
    ppf_match_3d::poseToRT(pose,R_tmp,t_tmp);
    std::cout << "pose to R t: " << R_tmp << std::endl << t_tmp <<std::endl;
    {
        Vec3d axis = Eigen::Vector3d::UnitX();
        double angle = M_PI/6;
        Matx33d R;
        ppf_match_3d::aaToR(axis,angle,R);
        std::cout << "aaToR: " << R <<std::endl;
        std::cout << "eigen: " << Eigen::AngleAxisd(angle,axis).matrix() << std::endl;
    }

    {
        double angle = M_PI/6;
        Matx33d R;
        ppf_match_3d::getUnitXRotation(angle,R);
        std::cout << "getUnitXRotation: " << R <<std::endl;
    }

    {
        Matx33d R = Eigen::AngleAxisd(M_PI/3,Eigen::Vector3d::UnitX()).matrix();
        Vec3d axis;
        double angle;
        ppf_match_3d::dcmToAA(R,axis,&angle);
        std::cout << "dcmToAA: axis :" << axis << std::endl << " angle: " << angle << std::endl;

        Matx33d R_tmp;
        ppf_match_3d::aaToDCM(axis,angle,R_tmp);
        std::cout << "aaToDCM: " <<R << std::endl<<"----------" << std::endl<<R_tmp << std::endl;
    }

    {
        uint32_t *key = new uint32_t [4];             /* input scalar */
//    char key[4] = {0};
        key[0] = static_cast<uint32_t>(floor(1));
        key[1] = static_cast<uint32_t>(floor(1));
        key[2] = static_cast<uint32_t>(floor(1));
        key[3] = static_cast<uint32_t>(floor(1));


        uint32_t* outMatrix = new uint32_t [4];              /* output matrix */


        ppf_match_3d::hashMurmurx64(key,16,42,outMatrix);
        std::cout << *outMatrix <<std::endl;
    }

    {
        ppf_match_3d::Pose3DPtr pose3DPtr = std::make_shared<ppf_match_3d::Pose3D>();
        pose3DPtr->numVotes = 0;
        pose3DPtr->printPose();
    }

    {

    }
    return 0;
}
