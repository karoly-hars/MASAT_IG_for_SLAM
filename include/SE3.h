#ifndef SE3_H
#define SE3_H


#include <sstream>
#include <assert.h>
#include <Eigen/Geometry>


typedef Eigen::Quaternion<double> Quat;
typedef Eigen::Vector3d Vec;


class SE3{

private:
    Vec trans;
    Quat rot;

public:
    SE3(){}

   ~SE3(){}

   SE3(double v_x, double v_y, double v_z, double q_w, double q_x, double q_y, double q_z){
        trans = Vec(v_x,v_y,v_z);
        rot = Quat(q_w,q_x,q_y,q_z);
    }

    SE3(const SE3& s){
        this->trans=s.trans;
        this->rot=s.rot;
    }

    SE3(Vec trans, Quat rot){
        this->trans=trans;
        this->rot=rot;
    }

    std::string toString() const {
        std::ostringstream strs;
        strs <<trans.x() << " " << trans.y() <<" " << trans.z() <<
        " "<<rot.x() << " " << rot.y() <<" " << rot.z()<<" "<<rot.w();
        return strs.str();
    }

    inline SE3 inv() const {
        Quat p;
        p.w() = 0;
        p.vec() = trans;

        Quat rotatedP = rot.inverse() * p * rot;
        Vec to_add = -rotatedP.vec();
        return SE3(to_add, rot.inverse());
    }

    inline SE3 operator * (const SE3& s) const {
        Quat p;
        p.w() = 0;
        p.vec() = s.trans;

        Quat rotatedP = rot * p * rot.inverse();

        Vec to_add = rotatedP.vec();
        return SE3(trans+to_add, rot*s.rot);
    }

    inline double operator [] (int i) const {
            assert( 0 <=i  && i<7 );
            if(i==0) return trans.x();
            else if(i==1) return trans.y();
            else if(i==2) return trans.z();
            else if(i==3) return rot.x();
            else if(i==4) return rot.y();
            else if(i==5) return rot.z();
            else return rot.w();
    }
};

#endif // SE3_H
