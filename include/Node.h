#ifndef NODE_H
#define NODE_H


class Node{

public:
    Node(){
        ID = 0;
    }

    ~Node(){}

    Node(int ID){
        this->ID=ID;
    }

    Node(const Node& v){
        this->ID=v.ID;
    }

    int getID(){
        return ID;
    }

    inline bool operator < (const Node v) const {
        return (this->ID < v.ID);
    }

private:
    int ID;
};

#endif // NODE_H
