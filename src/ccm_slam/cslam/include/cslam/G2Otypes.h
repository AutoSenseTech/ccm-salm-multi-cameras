#ifndef CSLAM_G2OTYPES_H_
#define CSLAM_G2OTYPES_H_

#include "thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
namespace g2o
{
class  EdgeSE3ProjectXYZPlus: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZPlus();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            Vector2d obs(_measurement);
            Eigen::Matrix3d Rrl = Trl.block<3,3>(0,0);
            Eigen::Vector3d trl = Trl.block<3,1>(0,3);
            _error = obs-cam_project(Rrl*((v1->estimate().map(v2->estimate())))+trl);
            error = _error;

        }

        bool isDepthPositive() {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
            Eigen::Matrix3d Rrl = Trl.block<3,3>(0,0);
            Eigen::Vector3d trl = Trl.block<3,1>(0,3);
            return (Rrl*(v1->estimate().map(v2->estimate())) + trl)(2)>0.0;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d & trans_xyz) const;

        double fx, fy, cx, cy;

        Eigen::Matrix4d Trl;
        Eigen::Vector2d error;

};


class  EdgeSE3ProjectXYZOnlyPosePlus: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPosePlus(){}

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Vector2d obs(_measurement);
        Eigen::Matrix3d Rrl = Trl.block<3,3>(0,0);
        Eigen::Vector3d trl = Trl.block<3,1>(0,3);
        _error = obs-cam_project(Rrl*(v1->estimate().map(Xw)) + trl);
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Eigen::Matrix3d Rrl = Trl.block<3,3>(0,0);
        Eigen::Vector3d trl = Trl.block<3,1>(0,3);
        return (Rrl * (v1->estimate().map(Xw)) + trl)(2)>0.0;
    }


    virtual void linearizeOplus();

    Vector2d cam_project(const Vector3d & trans_xyz) const;

    Vector3d Xw;

    double fx, fy, cx, cy;

    Eigen::Matrix4d Trl;
};

}
#endif