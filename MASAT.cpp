#include <iostream>
#include <ctime>
#include "include/Graph.h"


double getTime(clock_t time1, clock_t time2){
    double ticks= time1-time2;
    return (1000.0*ticks)/(CLOCKS_PER_SEC);
}


int main(int argc, char **argv){
    if (argc != 3) {
        std::cerr<<"Please use the following syntax: ./MASAT <PATH_TO_INPUT_FILE> <PATH_TO_OUTPUT_FILE>"<<std::endl;
        return 1;
    }

    Graph g;
    g.readFile(argv[1]);

    clock_t begin=clock();
    g.runMASAT();
    clock_t end=clock();
    std::cout<<"Runtime: "<<getTime(end, begin)<<" ms."<<std::endl;
    g.writeFile(argv[2]);

    return 0;
}
