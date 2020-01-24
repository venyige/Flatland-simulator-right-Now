#ifndef DI_MATH_H
#define DI_MATH_H

#include<iostream>
#include<chrono>
#include<cmath>
#include<memory>
#include<array>
#include<interface.h>
#include<algorithm>



namespace now {
using namespace std;

/*** Driver's Inline Math functions:
 * the utility functions take place in this block
 * in order to make the driver thread function "physics"
 * more readable*/
inline void getNextPos(interface_d::mv_b& nextBlockPos, const interface_d::mv_tb& moveBlock, const char rowC, const char colC){
    const char maxRow=rowC-1;
    const char maxCol=colC-1;
    switch (signbit(moveBlock.linVel)?(moveBlock.dir+4)%8:moveBlock.dir) {
    case 0:///< Up
        if(moveBlock.Y==0){
            nextBlockPos.Y=maxRow;
        }else{
            nextBlockPos.Y=moveBlock.Y-1;
        }
        nextBlockPos.X=moveBlock.X;
        break;
    case 1:///< Up-left
        if(moveBlock.Y==0){
            nextBlockPos.Y=maxRow;
        }else{
            nextBlockPos.Y=moveBlock.Y-1;
        }
        if(moveBlock.X==0){
            nextBlockPos.X=maxCol;
        }else{
            nextBlockPos.X=moveBlock.X-1;
        }
        break;
    case 2:///< Left
        if(moveBlock.X==0){
            nextBlockPos.X=maxCol;
        }else{
            nextBlockPos.X=moveBlock.X-1;
        }
        nextBlockPos.Y=moveBlock.Y;
        break;
    case 3:///< Down-left
        if(moveBlock.Y==maxRow){
            nextBlockPos.Y=0;
        }else{
            nextBlockPos.Y=moveBlock.Y+1;
        }
        if(moveBlock.X==0){
            nextBlockPos.X=maxCol;
        }else{
            nextBlockPos.X=moveBlock.X-1;
        }
        break;
    case 4:///< Down
        if(moveBlock.Y==maxRow){
            nextBlockPos.Y=0;
        }else{
            nextBlockPos.Y=moveBlock.Y+1;
        }
        nextBlockPos.X=moveBlock.X;
        break;
    case 5:///< Down-right
        if(moveBlock.Y==maxRow){
            nextBlockPos.Y=0;
        }else{
            nextBlockPos.Y=moveBlock.Y+1;
        }
        if(moveBlock.X==maxCol){
            nextBlockPos.X=0;
        }else{
            nextBlockPos.X=moveBlock.X+1;
        }
        break;
    case 6:///< Right
        if(moveBlock.X==maxCol){
            nextBlockPos.X=0;
        }else{
            nextBlockPos.X=moveBlock.X+1;
        }
        nextBlockPos.Y=moveBlock.Y;
        break;
    case 7:///< Up-right
        if(moveBlock.Y==0){
            nextBlockPos.Y=maxRow;
        }else{
            nextBlockPos.Y=moveBlock.Y-1;
        }
        if(moveBlock.X==maxCol){
            nextBlockPos.X=0;
        }else{
            nextBlockPos.X=moveBlock.X+1;
        }
        break;


    }

}

inline void checkKbrdKeypair(unsigned char k1, unsigned char k2, double& vel, double& acc, const double maxAcc, const double epsilon){
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
        if(intFac.car.dir==7){
            intFac.car.dir=0;
        }else{
            intFac.car.dir++;
        }
        distance-=1.0;
    }else if(distance<-0.5){
        if(intFac.car.dir==0){
            intFac.car.dir=7;
        }else{
            intFac.car.dir--;
        }
        distance+=1.0;
    }
}

inline void linear_snap(double& distance, interface_d::mv_tb& moveBlock, char rowMax, char colMax){
    const double sqrt2=sqrt(2.0);///< Freeze sqrt(2) value in order to boost the calculations.

    switch (moveBlock.dir) {
    case 0:///< Up
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
    case 2:///< Left
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
    case 4:///< Down
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
    case 6:///< Right
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
    case 1:///< Up-Left
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
    case 3:///< Down-Left
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
    case 5:///< Down-Right
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
    case 7:///< Up-Right
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
    switch (intFac.car.dir) {
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
            intFac.forces.push_back(interface_d::mv_f{char(jjj), char(iii), (*_sceneMat)[iii][jjj]});
            break;

        }
    }
    }



}
#endif // DI_MATH_H
