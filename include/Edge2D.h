#ifndef EDGE2D_H
#define EDGE2D_H

#include <Eigen/Dense>
#include "SE2.h"

typedef Eigen::MatrixXd Matrix;

class Edge2D{
	
public:
    Edge2D(){};
	
    ~Edge2D(){};
	
    Edge2D(int fromID, int toID, SE2 m, Matrix I)
    {
		this->fromID = fromID;
		this->toID = toID;
		this->m = m;
		this->I = I;
    }

    int getFromID() const
    {
		return fromID;
    }
	
    int getToID() const
    {
		return toID;
    }
	SE2 getM(){
		return m;
	}
	
	Matrix getI(){
		return I;
	}
	
	Edge2D getBackwards(){
		return Edge2D(toID, fromID, m.inv(), I);
	}
private:
    int toID;
    int fromID;
	SE2 m;
	Matrix I;
};

#endif // EDGE2D_H
