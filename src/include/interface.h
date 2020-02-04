#ifndef INTERFACE_H
#define INTERFACE_H
#include<chrono>
#include<vector>
#include<mutex>
#include<directions.h>
namespace now{
using namespace std;
enum tailChar{u='|', ul='\\', l='-',  dl='/',  d='|', dr='\\', r='-', ur='/'};
typedef struct{
    char X;
    char Y;
    char tailC=tailChar::u;
    char replacement=' ';
}tail;
/** public Typedefs and structs to share between driver and guardian agent*/
class interface_d
{
public:
    interface_d(){}
    typedef struct {
         char X=0;///< actual x postition
         char Y=0;///< actual y postition
    }mv_b;///< Block position
    typedef struct:mv_b {
         d_t dir=0;///< Snapped directions, multiplicators of pi/4
         double icDist;///< inter-cellar distance, that is the remaining in between snappings
        double linVel=0;///< linear velocity
        double linAcc=0;///< actual linear acceleration
    }mv_tb;///< Movable block
    typedef struct T:mv_tb{
        T& operator =(T& inCar){
            if(this!=&inCar){
                this->X=inCar.X;
                this->Y=inCar.Y;
                this->dir=inCar.dir;
                this->icDist=inCar.icDist;
                this->linVel=inCar.linVel;
                this->linAcc=inCar.linAcc;
                this->angle=inCar.angle;
                this->angVel=inCar.angVel;
                this->angAcc=inCar.angAcc;
            }
            return *this;
        }
        double angle=0;///< actual direction angle (rad*4/pi)
        double angVel=0;///< actual angular velocity (rotation speed)
        double angAcc=0;///< actual angular acceleration
    }mv_t;///< Dynamic data tailored to car
    typedef struct T2:mv_b{
        T2(char _X, char _Y, char c):mv_b{_X,_Y}{dir=char(c);}
         d_t dir=0;///< Snapped directions, multiplicators of pi/4
    }mv_f;///< Force, has position and named direction
    mv_t car;///< The car data structure instance
    vector<mv_tb> blocks;///< Movable blocks
    vector<mv_b> walls;///< Immobile wall blocks
    vector<mv_f> forces;///< Force field blocks
    unsigned char emBrake;///< Emergency brake mask issued by guardian agent
    volatile double tLag;///< Worst case lag from command to execution, updated continuously in driver
    mutex _mutex;///< Interface mutex to share with guardian agent
    static constexpr double epsilon=1e-16;///< Epsilon to judge practical zero value of double precision variables
    static constexpr unsigned int tSlot=10; ///< Sleep time for threads (ms)
    enum blockType{noblock, wall_t, mobile_t, ffield_t};
};

}
#endif // INTERFACE_H
