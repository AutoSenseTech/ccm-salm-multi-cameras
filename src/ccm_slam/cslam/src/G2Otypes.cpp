#include <cslam/G2Otypes.h>
namespace g2o
{
    Vector2d project2d(const Vector3d& v)
    {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    EdgeSE3ProjectXYZPlus::EdgeSE3ProjectXYZPlus() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>()
    {
    }

    bool EdgeSE3ProjectXYZPlus::read(std::istream& is)
    {
        return true;
    }

    bool EdgeSE3ProjectXYZPlus::write(std::ostream& os) const
    {
        return true;
    }

    void EdgeSE3ProjectXYZPlus::linearizeOplus()
    {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());  //Tlw
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3d xyz = vi->estimate();  //Pw
        Vector3d xyz_transL = T.map(xyz); //Pl;

        Eigen::Matrix3d Rrl = Trl.block<3,3>(0,0);
        Eigen::Vector3d trl = Trl.block<3,1>(0,3);
        Vector3d xyz_transR = Rrl*xyz_transL + trl;//Pr = Trl * Tlw * Pw;


        double xL = xyz_transL[0];
        double yL = xyz_transL[1];
        double zL = xyz_transL[2];

        double xR = xyz_transR[0];
        double yR = xyz_transR[1];
        double invzR = 1.0/xyz_transR[2];
        double invzR_2 = invzR*invzR;

        Eigen::Matrix<double, 2, 3> A; // derror/dPr
        A(0,0) = fx * invzR;
        A(0,1) = 0;
        A(0,2) = -fx*xR*invzR_2;

        A(1, 0) = 0;
        A(1,1) = fy*invzR;
        A(1,2 ) = -fy * yR * invzR_2;


        Eigen::Matrix<double, 3, 6> S; //dPl/dPosel
        S(0,0) = 0;
        S(0,1) = zL;
        S(0,2) = -yL;
        S(0,3) = 1;
        S(0,4) = 0;
        S(0,5) = 0;

        S(1,0) = -zL;
        S(1,1) = 0;
        S(1,2) = xL;
        S(1,3) = 0;
        S(1,4) = 1;
        S(1,5) = 0;

        S(2,0) = yL;
        S(2,1) = -xL;
        S(2,2) = 0;
        S(2,3) = 0;
        S(2,4) = 0;
        S(2,5) = 1;

        Matrix<double, 2, 6> jacobian;
        jacobian = -1 * A * Rrl * S;

        _jacobianOplusXi =  -1 * A * Rrl*T.rotation().toRotationMatrix();

        _jacobianOplusXj = jacobian;

    }



    Vector2d EdgeSE3ProjectXYZPlus::cam_project(const Vector3d & trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    bool EdgeSE3ProjectXYZOnlyPosePlus::read(std::istream & is)
    {
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPosePlus::write(std::ostream& os) const
    {
        return true;
    }

    void EdgeSE3ProjectXYZOnlyPosePlus::linearizeOplus() {
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        SE3Quat T(vi->estimate());  //Tlw
        Vector3d xyz_transL = T.map(Xw);  //PL;

        double xL = xyz_transL[0];
        double yL = xyz_transL[1];
        double zL = xyz_transL[2];

        Eigen::Matrix3d Rrl = Trl.block<3,3>(0,0);
        Eigen::Vector3d trl = Trl.block<3,1>(0,3);
        Vector3d xyz_transR = Rrl*xyz_transL + trl;//Pr = Trl * Tlw * Pw;

        double xR = xyz_transR[0];
        double yR = xyz_transR[1];

        double invzR = 1.0/xyz_transR[2];
        double invzR_2 = invzR*invzR;

        Eigen::Matrix<double, 2, 3> A;
        A(0,0) = fx * invzR;
        A(0,1) = 0;
        A(0,2) = -fx*xR * invzR_2;

        A(1, 0) = 0;
        A(1,1) = fy*invzR;
        A(1,2 ) = -fy*yR*invzR_2;

        Eigen::Matrix<double, 3, 6> S;
        S(0,0) = 0;
        S(0,1) = zL;
        S(0,2) = -yL;
        S(0,3) = 1;
        S(0,4) = 0;
        S(0,5) = 0;

        S(1,0) = -zL;
        S(1,1) = 0;
        S(1,2) = xL;
        S(1,3) = 0;
        S(1,4) = 1;
        S(1,5) = 0;

        S(2,0) = yL;
        S(2,1) = -xL;
        S(2,2) = 0;
        S(2,3) = 0;
        S(2,4) = 0;
        S(2,5) = 1;

        Matrix<double, 2, 6> jacobian;
        jacobian = -1 * A * Rrl * S;

        _jacobianOplusXi = jacobian;

    }

    Vector2d EdgeSE3ProjectXYZOnlyPosePlus::cam_project(const Vector3d & trans_xyz) const{
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }
}
