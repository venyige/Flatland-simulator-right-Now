#ifndef INTERFACE_H
#define INTERFACE_H
#include<chrono>
#include<vector>
namespace now{
using namespace std;
enum tailChar{u='|', ul='\\', l='-',  dl='/',  d='|', dr='\\', r='-', ur='/'};
typedef struct{
    char X;
    char Y;
    char tailC=tailChar::u;
    char replacement=' ';
}tail;

class interface_d
{
public:
    interface_d(){}
    /*** public Typedefs to share with guardian agent*/

    typedef struct {
         char X=0;///< actual x postition
         char Y=0;///< actual y postition
    }mv_b;///< Block position
    typedef struct:mv_b {
         char dir=0;///< Snapped directions, multiplicators of pi/4
         double icDist;///< inter-cellar distance, that is the remaining in between snappings
        double linVel=0;///< linear velocity
        double linAcc=0;///< actual linear acceleration
    }mv_tb;///< Movable block
    typedef struct :mv_tb{
        double angle=0;///< actual direction angle (rad*4/pi)
        double angVel=0;///< actual angular velocity (rotation speed)
        double angAcc=0;///< actual angular acceleration
    }mv_t;///< Dynamic data tailored to car
    typedef struct :mv_b{
         char dir=0;///< Snapped directions, multiplicators of pi/4
    }mv_f;///< Force, has position and named direction
    mv_t car;
    vector<mv_tb> blocks;
    vector<mv_b> walls;
    vector<mv_f> forces;

};
}
#endif // INTERFACE_H
