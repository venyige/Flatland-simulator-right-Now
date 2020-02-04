#ifndef DI_MATH_H
#define DI_MATH_H
/** \file di_math.h
 * \brief Driver's Inline Math functions:
 * The utility functions take place in this block
 * in order to make the driver thread function "physics"
 * more readable*/
#include<iostream>
#include<chrono>
#include<cmath>
#include<memory>
#include<array>
#include<interface.h>
#include<algorithm>



namespace now {
using namespace std;
/** \brief Get the next postition
 * Getting the next available position in the direction of the current pacing.
*/
inline void getNextPos(
        interface_d::mv_b& nextBlockPos, ///< Reference to next position struct
        const interface_d::mv_tb& moveBlock,///< Movable block, or recast car struct
        const size_t rowC,///< Rowcount
        const size_t colC,///< Columncount
        const unsigned int numSteps=1
        ){
    int X=int(moveBlock.X),Y=int(moveBlock.Y);
    d_t locDir=((abs(moveBlock.linVel)>interface_d::epsilon)&&signbit(moveBlock.linVel))?int(d_t(moveBlock.dir)+4):int(moveBlock.dir);
    if(locDir.hasUp()){
        Y-=int(numSteps);
    }else if(locDir.hasDown()){
        Y+=int(numSteps);
    }
    if(locDir.hasLeft()){
        X-=int(numSteps);
    }else if(locDir.hasRight()){
        X+=int(numSteps);
    }
    X%=colC;
    Y%=rowC;
    if(X<0){
        X+=colC;
    }
    if(Y<0){
        Y+=rowC;
    }
    nextBlockPos.X=X;
    nextBlockPos.Y=Y;
}

/** \brief Check key pairs:
 * Checking of the opposite pairs of WASD buttons.*/
inline void checkKbrdKeypair(unsigned char k1, ///< Key 1 -  W or A
                             unsigned char k2, ///< Key 2 - S or D
                             double& vel, ///< Velocity
                             double& acc, ///< Acceleration
                             const double maxAcc, ///< Maximum of acceleration
                             const double epsilon ///< Epsilon value as defined in the main thread function
                             ){
    if(k1 && !k2){
        if(vel<-epsilon){
            acc=2.0*maxAcc;
        }else{
            acc=maxAcc;
        }
    }else if(!k1 && k2){
        if(vel>epsilon){
            acc=-2.0*maxAcc;
        }else{
            acc=-maxAcc;
        }
    }else if((k1 && k2)||(!k1 &&!k2)){

        if(vel<-epsilon){
            acc=maxAcc;
        }else if(vel>epsilon){
            acc=-maxAcc;
        }else{
            acc=0.0;
        }
    }

}

inline void angular_snap(double& distance, interface_d& intFac){
    if(distance>0.5){
        intFac.car.dir++;
        distance-=1.0;
    }else if(distance<-0.5){
        intFac.car.dir--;
        distance+=1.0;
    }
}

inline void linear_snap(double& distance, interface_d::mv_tb& moveBlock, char rowMax, char colMax){
    const double sqrt2=sqrt(2.0);//< Freeze sqrt(2) value in order to boost the calculations.

    switch (int(moveBlock.dir)) {
    case 0://< Up
        if(distance>0.5){
            if(moveBlock.Y==0){
                moveBlock.Y=rowMax;
            }else{
                moveBlock.Y--;
            }
            distance-=1.0;
        }else if(distance<-0.5){
            if(moveBlock.Y==rowMax){
                moveBlock.Y=0;
            }else{
                moveBlock.Y++;
            }
            distance+=1.0;
        }
        break;
    case 2://< Left
        if(distance>0.5){
            if(moveBlock.X==0){
                moveBlock.X=colMax;
            }else{
                moveBlock.X--;
            }
            distance-=1.0;
        }else if(distance<-0.5){
            if(moveBlock.X==colMax){
                moveBlock.X=0;
            }else{
                moveBlock.X++;
            }
            distance+=1.0;
        }
        break;
    case 4://< Down
        if(distance>0.5){
            if(moveBlock.Y==rowMax){
                moveBlock.Y=0;
            }else{
                moveBlock.Y++;
            }
            distance-=1.0;
        }else if(distance<-0.5){
            if(moveBlock.Y==0){
                moveBlock.Y=rowMax;
            }else{
                moveBlock.Y--;
            }
            distance+=1.0;
        }
        break;
    case 6://< Right
        if(distance>0.5){
            if(moveBlock.X==colMax){
                moveBlock.X=0;
            }else{
                moveBlock.X++;
            }
            distance-=1.0;
        }else if(distance<-0.5){
            if(moveBlock.X==0){
                moveBlock.X=colMax;
            }else{
                moveBlock.X--;
            }
            distance+=1.0;
        }
        break;
    case 1://< Up-Left
        if(distance>sqrt2/2.0){
            if(moveBlock.X==0){
                moveBlock.X=colMax;
            }else{
                moveBlock.X--;
            }
            if(moveBlock.Y==0){
                moveBlock.Y=rowMax;
            }else{
                moveBlock.Y--;
            }
            distance-=sqrt2;
        }else if(distance<-sqrt2/2.0){
            if(moveBlock.X==colMax){
                moveBlock.X=0;
            }else{
                moveBlock.X++;
            }
            if(moveBlock.Y==rowMax){
                moveBlock.Y=0;
            }else{
                moveBlock.Y++;
            }
            distance+=sqrt2;
        }
        break;
    case 3://< Down-Left
        if(distance>sqrt2/2.0){
            if(moveBlock.X==0){
                moveBlock.X=colMax;
            }else{
                moveBlock.X--;
            }
            if(moveBlock.Y==rowMax){
                moveBlock.Y=0;
            }else{
                moveBlock.Y++;
            }
            distance-=sqrt2;
        }else if(distance<-sqrt2/2.0){
            if(moveBlock.X==colMax){
                moveBlock.X=0;
            }else{
                moveBlock.X++;
            }
            if(moveBlock.Y==0){
                moveBlock.Y=rowMax;
            }else{
                moveBlock.Y--;
            }
            distance+=sqrt2;
        }
        break;
    case 5://< Down-Right
        if(distance>sqrt2/2.0){
            if(moveBlock.X==colMax){
                moveBlock.X=0;
            }else{
                moveBlock.X++;
            }
            if(moveBlock.Y==rowMax){
                moveBlock.Y=0;
            }else{
                moveBlock.Y++;
            }
            distance-=sqrt2;
        }else if(distance<-sqrt2/2.0){
            if(moveBlock.X==0){
                moveBlock.X=colMax;
            }else{
                moveBlock.X--;
            }
            if(moveBlock.Y==0){
                moveBlock.Y=rowMax;
            }else{
                moveBlock.Y--;
            }
            distance+=sqrt2;
        }
        break;
    case 7://< Up-Right
        if(distance>sqrt2/2.0){
            if(moveBlock.X==colMax){
                moveBlock.X=0;
            }else{
                moveBlock.X++;
            }
            if(moveBlock.Y==0){
                moveBlock.Y=rowMax;
            }else{
                moveBlock.Y--;
            }
            distance-=sqrt2;
        }else if(distance<-sqrt2/2.0){
            if(moveBlock.X==0){
                moveBlock.X=colMax;
            }else{
                moveBlock.X--;
            }
            if(moveBlock.Y==rowMax){
                moveBlock.Y=0;
            }else{
                moveBlock.Y++;
            }
            distance+=sqrt2;
        }
        break;
    }

}

inline void tail_proc(interface_d& intFac,  tail& tailC, char rowMax, char colMax){
    switch (int(intFac.car.dir)) {
    case 0:
        if(intFac.car.Y==rowMax){
            tailC.Y=0;
        }else{
            tailC.Y=intFac.car.Y+1;
        }
        tailC.X=intFac.car.X;
        tailC.tailC=tailChar::u;
        break;
    case 1:
        if(intFac.car.Y==rowMax){
            tailC.Y=0;
        }else{
            tailC.Y=intFac.car.Y+1;
        }
        if(intFac.car.X==colMax){
            tailC.X=0;
        }else{
            tailC.X=intFac.car.X+1;
        }
        tailC.tailC=tailChar::ul;
        break;
    case 2:
        if(intFac.car.X==colMax){
            tailC.X=0;
        }else{
            tailC.X=intFac.car.X+1;
        }
        tailC.Y=intFac.car.Y;
        tailC.tailC=tailChar::l;
        break;
    case 3:
        if(intFac.car.Y==0){
            tailC.Y=rowMax;
        }else{
            tailC.Y=intFac.car.Y-1;
        }
        if(intFac.car.X==colMax){
            tailC.X=0;
        }else{
            tailC.X=intFac.car.X+1;
        }
        tailC.tailC=tailChar::dl;
        break;
    case 4:
        if(intFac.car.Y==0){
            tailC.Y=rowMax;
        }else{
            tailC.Y=intFac.car.Y-1;
        }
        tailC.X=intFac.car.X;
        tailC.tailC=tailChar::d;
        break;
    case 5:
        if(intFac.car.Y==0){
            tailC.Y=rowMax;
        }else{
            tailC.Y=intFac.car.Y-1;
        }
        if(intFac.car.X==0){
            tailC.X=colMax;
        }else{
            tailC.X=intFac.car.X-1;
        }
        tailC.tailC=tailChar::dr;
        break;
    case 6:
        if(intFac.car.X==0){
            tailC.X=colMax;
        }else{
            tailC.X=intFac.car.X-1;
        }
        tailC.Y=intFac.car.Y;
        tailC.tailC=tailChar::r;
        break;
    case 7:
        if(intFac.car.X==0){
            tailC.X=colMax;
        }else{
            tailC.X=intFac.car.X-1;
        }
        if(intFac.car.Y==rowMax){
            tailC.Y=0;
        }else{
            tailC.Y=intFac.car.Y+1;
        }
        tailC.tailC=tailChar::ur;
        break;
    }

}

template<size_t _rowC, size_t _colC>
inline void initScene( const shared_ptr<array<array<char, _colC>, _rowC>>& _sceneMat, interface_d& intFac,  tail& tailC){
    size_t rowMax=_rowC-1, colMax=_colC-1;
for(size_t iii=0; iii<_rowC; iii++)
    for (size_t jjj=0;jjj<_colC;jjj++) {

        switch ((*_sceneMat)[iii][jjj]) {
        case 'O':
            intFac.car.X=jjj;
            intFac.car.Y=iii;
            tailC.X=jjj;
            if(iii==rowMax){
                tailC.Y=0;
                tailC.replacement=(*_sceneMat)[0][jjj];
            }else {
                tailC.Y=iii+1;
                tailC.replacement=(*_sceneMat)[iii+1][jjj];
            }
            break;
        case '~':
            intFac.blocks.push_back(interface_d::mv_tb{char(jjj), char(iii)});
            break;
        case '#':
            intFac.walls.push_back(interface_d::mv_b{char(jjj), char(iii)});
            break;
        case '^':
        case '<':
        case 'v':
        case '>':
            intFac.forces.push_back(interface_d::mv_f(char(jjj), char(iii), char((*_sceneMat)[iii][jjj])));
            break;

        }
    }
    }



}
#endif // DI_MATH_H
