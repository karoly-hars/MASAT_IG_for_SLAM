#ifndef SE2_H
#define SE2_H


#include <sstream>
#include <assert.h>
#include <cmath>


inline double normalize_theta(double theta){
    if (theta >= -M_PI && theta < M_PI)
        return theta;

    double multiplier = floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;

    return theta;
}


class SE2{

private:
    double x;
    double y;
    double th;

public:
    SE2(){
        this->th=0;
        this->x=0;
        this->y=0;
    }

    ~SE2(){}

    SE2(double x, double y, double th){
        this->th=normalize_theta(th);
        this->x=x;
        this->y=y;
    }

    SE2(const SE2& s){
        this->th=normalize_theta(s.th);
        this->x=s.x;
        this->y=s.y;
    }

    inline SE2 operator * (const SE2& s) const {
        double R_cos=cos(th);
        double R_sin=sin(th);

        double nx = R_cos*s.x - R_sin*s.y + x; // x
        double ny = R_sin*s.x + R_cos*s.y + y; // y

        return SE2(nx, ny, atan2(sin(th+s.th), cos(th+s.th)));
    }

    inline SE2 inv() const {
        double R_cos = cos(th);
        double R_sin = sin(th);

        double nx = (-x*R_cos - y*R_sin);
        double ny = (-y*R_cos + x*R_sin);

        return SE2(nx, ny, atan2(-R_sin, R_cos));
    }

    inline double operator [] (int i) const {
        assert(0<=i && i<3);
        if(i==0) return x;
        else if(i==1) return y;
        else return th;
    }

    std::string toString() const {
        std::ostringstream strs;
        strs << x << " " << y << " " << th;
        return strs.str();
    }
};

#endif // SE2_H
