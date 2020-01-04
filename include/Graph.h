#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <list>
#include <queue>
#include <algorithm>
#include <string>
#include "Node.h"
#include "Edge2D.h"
#include "Edge3D.h"

class Graph{

public:
    Graph();
    ~Graph();

    void readFile(std::string inputPath);
    void writeFile(std::string outputPath);
    void runMASAT();
    
private:
    int D; // dimension: 2 or 3
    int EdgeNum;
    
    std::map<Node, std::list<Edge2D>> structure2D;
    std::map<Node, std::list<Edge3D>> structure3D;
    std::map<Node, SE2> positions2D;
    std::map<Node, SE3> positions3D;
    std::list<Edge2D> edges2D;
    std::list<Edge3D> edges3D;
    
    void addEdge2D(Edge2D e);
    void addEdge3D(Edge3D e);
    
    void readFile2D(std::string inputPath);
    void readFile3D(std::string inputPath);
    
    void writeFile2D(std::string outputPath);
    void writeFile3D(std::string outputPath);
    
    void runMASAT2D();
    void runMASAT3D();
};

#endif // GRAPH_H
