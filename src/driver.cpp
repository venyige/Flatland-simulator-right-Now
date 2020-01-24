#ifndef DRIVER_CPP
#define DRIVER_CPP
#include "driver.h"
#include<vector>
#include<cmath>

namespace now{
template<size_t rowC, size_t colC>
driver<rowC, colC>::driver(shared_ptr<array<array<char, colC>, rowC>>& sceneMat,
                           mutex& sceneMut,
                           mutex& kbdMut,
                           volatile const char& kbStat,
                           volatile const bool& quitter,
                           const double& maxLinSpeed,
                           const double& maxAngSpeed,
                           const double& maxAcc):
    _sceneMat(sceneMat),
    _sceneMutex(sceneMut),
    _kbdMutex(kbdMut),
    _kbStat(kbStat),
    _quitter(quitter),
    _maxLinSpeed(maxLinSpeed),
    _maxAngSpeed(maxAngSpeed),
    _maxAcc(maxAcc)
{/*
    forwardOn=-1.0;
    backwardOn=-1.0;
    leftOn=-1.0;
    rightOn=-1.0;

    forwardProcessed=false;
    backwardProcessed=false;
    leftProcessed=false;
    rightProcessed=false;
    */
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
/*** Car driver physics thread function.
 * 1. Initialization step preparing the scene data to display, and
 *  extracting the trafic limiting object into the interface class's
 *  appropriate vectors in order to make them available to the guardian
 *  agent.
 *
 * After initialization, the thread's main loop starts with:
 * 2. Updating the linear data based on:
 * - actual linear acceleration,
 * - actual linear velocity,
 * - actual residual (not jet snapped) distance,
 * - car direction.
 * 3. Updating the angular data based on:
 * - actual angular acceleration,
 * - actual rotation speed,
 * - residual angle data.
 * 4. Updating both the linear and angular acceleration data based on
 * the "quasi realtime" keyboard status flags.*/
template<size_t rowC, size_t colC>
void driver<rowC, colC>::physics(){
    size_t colMax=_colC-1;
    size_t rowMax=_rowC-1;
    const double sqrt2=sqrt(2.0);///< Freeze sqrt(2) value in order to boost the calculations.
    double maxDistance=0.0;///< Theoretical maximum distance (either linear, or angular) with maximum allowed speed in the given time term.
    double epsilon=1e-32;///< Epsilon to judge practical zero value of double precision variables
    using hrClk=chrono::high_resolution_clock ;
    hrClk::time_point lastTime;
    double intervalD;///< t in seconds
    double distance;///< space: v0*t+(a/2)*t^2 or rot: u0*t+(omg/2)*t^2
    double velocity;///< For both linear and angular movement
    tail tailC;///< Structure typedef to store tail data

    /*** To initialize the scene matrix, we lock the scene mutex*/
    while(_sceneMutex.try_lock());
    for(size_t iii=0; iii<_rowC; iii++)
        for (size_t jjj=0;jjj<_colC;jjj++) {

            switch ((*_sceneMat)[iii][jjj]) {
            case 'O':
                this->intFac.car.X=jjj;
                this->intFac.car.Y=iii;
                headReplacement=' ';
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
                this->intFac.blocks.push_back(interface_d::mv_tb{char(jjj), char(iii)});
                break;
            case '#':
                this->intFac.walls.push_back(interface_d::mv_b{char(jjj), char(iii)});
                break;
            case '^':
            case '<':
            case 'v':
            case '>':
                this->intFac.walls.push_back(interface_d::mv_f{char(jjj), char(iii), (*_sceneMat)[iii][jjj]});
                break;

            }
        }
    /*** Unlock the scene mutex allowing the display driver to show the scene*/
    _sceneMutex.unlock();
    while(!_quitter){
/*** To accurately calculate the actual position and velocity data
 * chrono::high_resolution_clock features being utilized, casting
 * the high resolution data in seconds to double precision float.*/
        intervalD = chrono::duration_cast<chrono::duration<double>>( hrClk::now()-lastTime).count();

        lastTime=hrClk::now();
        /*** Updating the linear data of the car*/
        distance=0.0;
        velocity=0.0;



        if(abs(intFac.car.linAcc)>epsilon){
            distance=(intFac.car.linAcc * intervalD*intervalD)/2.0;
            velocity=intFac.car.linAcc * intervalD;
        }
        if(abs(intFac.car.linVel)>epsilon){
            distance+=intFac.car.linVel*intervalD;
            intFac.car.linVel+=velocity;
        }else{
            intFac.car.linVel=velocity;
        }

        if(intFac.car.linVel>_maxLinSpeed){
            intFac.car.linVel=_maxLinSpeed;
        }else if(intFac.car.linVel<-_maxLinSpeed){
            intFac.car.linVel=-_maxLinSpeed;
        }
        maxDistance=_maxLinSpeed*intervalD;///< maximum velocity * elapsed time

        if(abs(distance)>epsilon){
            if((distance)>maxDistance){
                distance=maxDistance;
            }else if((distance)<-maxDistance){
                distance=-maxDistance;
            }
            distance+=intFac.car.icDist;
        }else{
            distance=0.0;
        }

        /*** Now we process the linear distance, that is snap if necessary,
         * and the residual to be stored in the inter-cellar member*/

        switch (intFac.car.dir) {
        case 0:///< Up
            if(distance>0.5){
                if(intFac.car.Y==0){
                    intFac.car.Y=rowMax;
                }else{
                    intFac.car.Y--;
                }
                distance-=1.0;
            }else if(distance<-0.5){
                if(intFac.car.Y==rowMax){
                    intFac.car.Y=0;
                }else{
                    intFac.car.Y++;
                }
                distance+=1.0;
            }
            break;
        case 2:///< Left
            if(distance>0.5){
                if(intFac.car.X==0){
                    intFac.car.X=colMax;
                }else{
                    intFac.car.X--;
                }
                distance-=1.0;
            }else if(distance<-0.5){
                if(intFac.car.X==colMax){
                    intFac.car.X=0;
                }else{
                    intFac.car.X++;
                }
                distance+=1.0;
            }
            break;
        case 4:///< Down
            if(distance>0.5){
                if(intFac.car.Y==rowMax){
                    intFac.car.Y=0;
                }else{
                    intFac.car.Y++;
                }
                distance-=1.0;
            }else if(distance<-0.5){
                if(intFac.car.Y==0){
                    intFac.car.Y=rowMax;
                }else{
                    intFac.car.Y--;
                }
                distance+=1.0;
            }
            break;
        case 6:///< Right
            if(distance>0.5){
                if(intFac.car.X==colMax){
                    intFac.car.X=0;
                }else{
                    intFac.car.X++;
                }
                distance-=1.0;
            }else if(distance<-0.5){
                if(intFac.car.X==0){
                    intFac.car.X=colMax;
                }else{
                    intFac.car.X--;
                }
                distance+=1.0;
            }
            break;
        case 1:///< Up-Left
            if(distance>sqrt2/2.0){
                if(intFac.car.X==0){
                    intFac.car.X=colMax;
                }else{
                    intFac.car.X--;
                }
                if(intFac.car.Y==0){
                    intFac.car.Y=rowMax;
                }else{
                    intFac.car.Y--;
                }
                distance-=sqrt2;
            }else if(distance<-sqrt2/2.0){
                if(intFac.car.X==colMax){
                    intFac.car.X=0;
                }else{
                    intFac.car.X++;
                }
                if(intFac.car.Y==rowMax){
                    intFac.car.Y=0;
                }else{
                    intFac.car.Y++;
                }
                distance+=sqrt2;
            }
            break;
        case 3:///< Down-Left
            if(distance>sqrt2/2.0){
                if(intFac.car.X==0){
                    intFac.car.X=colMax;
                }else{
                    intFac.car.X--;
                }
                if(intFac.car.Y==rowMax){
                    intFac.car.Y=0;
                }else{
                    intFac.car.Y++;
                }
                distance-=sqrt2;
            }else if(distance<-sqrt2/2.0){
                if(intFac.car.X==colMax){
                    intFac.car.X=0;
                }else{
                    intFac.car.X++;
                }
                if(intFac.car.Y==0){
                    intFac.car.Y=rowMax;
                }else{
                    intFac.car.Y--;
                }
                distance+=sqrt2;
            }
            break;
        case 5:///< Down-Right
            if(distance>sqrt2/2.0){
                if(intFac.car.X==colMax){
                    intFac.car.X=0;
                }else{
                    intFac.car.X++;
                }
                if(intFac.car.Y==rowMax){
                    intFac.car.Y=0;
                }else{
                    intFac.car.Y++;
                }
                distance-=sqrt2;
            }else if(distance<-sqrt2/2.0){
                if(intFac.car.X==0){
                    intFac.car.X=colMax;
                }else{
                    intFac.car.X--;
                }
                if(intFac.car.Y==0){
                    intFac.car.Y=rowMax;
                }else{
                    intFac.car.Y--;
                }
                distance+=sqrt2;
            }
            break;
        case 7:///< Up-Right
            if(distance>sqrt2/2.0){
                if(intFac.car.X==colMax){
                    intFac.car.X=0;
                }else{
                    intFac.car.X++;
                }
                if(intFac.car.Y==0){
                    intFac.car.Y=rowMax;
                }else{
                    intFac.car.Y--;
                }
                distance-=sqrt2;
            }else if(distance<-sqrt2/2.0){
                if(intFac.car.X==0){
                    intFac.car.X=colMax;
                }else{
                    intFac.car.X--;
                }
                if(intFac.car.Y==rowMax){
                    intFac.car.Y=0;
                }else{
                    intFac.car.Y++;
                }
                distance+=sqrt2;
            }
            break;
        }
        /*** If there is no linear speed remained, the position is to be
         * snapped to the actual grid point. Othervise it has to
         * be updated according to the remaining distance value.*/
        if(abs(intFac.car.linVel)>epsilon){
            intFac.car.icDist=distance;
        }else{
            intFac.car.icDist=0.0;
        }

        /*** Updating the angular data of the car.*/

        distance=0.0;
        velocity=0.0;
        if(abs(intFac.car.angAcc)>epsilon){
            distance=(intFac.car.angAcc * intervalD*intervalD)/2.0;
            velocity=intFac.car.angAcc * intervalD;
        }
        if(abs(intFac.car.angVel)>epsilon){
            distance+=intFac.car.angVel*intervalD;
            intFac.car.angVel+=velocity;
        }else{
            intFac.car.angVel=velocity;
        }

        if(intFac.car.angVel>_maxAngSpeed){
            intFac.car.angVel=_maxAngSpeed;
        }else if(intFac.car.angVel<-_maxAngSpeed){
            intFac.car.angVel=-_maxAngSpeed;
        }



        maxDistance=_maxAngSpeed * intervalD;///< maximum velocity * elapsed time

        if(abs(distance)>epsilon){
            if((distance)>maxDistance){
                distance=maxDistance;
            }else if((distance)<-maxDistance){
                distance=-maxDistance;
            }
            distance+=intFac.car.angle;
        }else{
            distance=0.0;
        }

        /*** Now we process the rotation (distance), that is snap if necessary,
         * and the remaining to be stored in the angle member. First check if there
         * is any force field at the actual car position, so to override the direction.*/
        for(auto iii:intFac.forces){
            if((static_cast<interface_d::mv_f>(iii).X==intFac.car.X)&&(static_cast<interface_d::mv_f>(iii).Y==intFac.car.Y)){
                switch (static_cast<interface_d::mv_f>(iii).dir) {
                case '^':
                    intFac.car.dir=0;
                    break;
                case '<':
                    intFac.car.dir=2;
                    break;
                case 'v':
                    intFac.car.dir=4;
                    break;
                case '>':
                    intFac.car.dir=6;
                    break;
                }
            }
        }


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
        /*** If there is no rotation speed remained, the rotation is to be
         * snapped to any of the clear named directions. Othervise it has to
         * be updated according to the remaining angle value (distance)*/
        if(abs(intFac.car.angVel)>epsilon){
            intFac.car.angle=distance;
        }else{
            intFac.car.angle=0.0;
        }

        /*** Now trying to gain control on the keyboard mutex
         * in order to read its value without the chance of being
         * overwritten during the operation.*/
        while(_kbdMutex.try_lock());
        if(kbStatBuf!=_kbStat){
            kbStatBuf=_kbStat;
        }
        _kbdMutex.unlock();
checkKbrdKeypair(kbStatBuf&1,
                 kbStatBuf&4,
                 intFac.car.linVel,
                 intFac.car.linAcc,
                 _maxAcc,
                 epsilon);
/*        if((kbStatBuf&1)&&!(kbStatBuf&4)){
            if(intFac.car.linVel<-epsilon){
                intFac.car.linAcc=2.0*_maxAcc;
            }else{
                intFac.car.linAcc=_maxAcc;
            }
        }else if(!(kbStatBuf&1) && (kbStatBuf&4)){
            if(intFac.car.linVel>epsilon){
                intFac.car.linAcc=-2.0*_maxAcc;
            }else{
                intFac.car.linAcc=-_maxAcc;
            }
        }else if(((kbStatBuf&1)&&(kbStatBuf&4))||(!(kbStatBuf&1)&&!(kbStatBuf&4))){

            if(intFac.car.linVel<-epsilon){
                intFac.car.linAcc=_maxAcc;
            }else if(intFac.car.linVel>epsilon){
                intFac.car.linAcc=-_maxAcc;
            }else{
                intFac.car.linAcc=0.0;
            }
        }


        if((kbStatBuf&2)&&!(kbStatBuf&8)){
            if(intFac.car.angVel<-epsilon){
                intFac.car.angAcc=2.0*_maxAcc;
            }else{
                intFac.car.angAcc=_maxAcc;
            }
        }else if(!(kbStatBuf&2) &&( kbStatBuf&8)){
            if(intFac.car.angVel>epsilon){
                intFac.car.angAcc=-2.0*_maxAcc;
            }else{
                intFac.car.angAcc=-_maxAcc;
            }
        }else if(((kbStatBuf&2)&&(kbStatBuf&8))||(!(kbStatBuf&2)&&!(kbStatBuf&8))){

            if(intFac.car.angVel<-epsilon){
                intFac.car.angAcc=_maxAcc;
            }else if(intFac.car.angVel>epsilon){
                intFac.car.angAcc=-_maxAcc;
            }else{
                intFac.car.angAcc=0.0;
            }
        }
*/
checkKbrdKeypair(kbStatBuf&2,
                 kbStatBuf&8,
                 intFac.car.angVel,
                 intFac.car.angAcc,
                 _maxAcc,
                 epsilon);

        while(_sceneMutex.try_lock());
        for(auto& iii: (*_sceneMat)){
            iii.fill(' ');///< fill with spaces (ascii 32)
        }
        for(auto iii:intFac.walls){
            (*_sceneMat)[iii.Y][iii.X]='#';
        }
        for(auto iii:intFac.blocks){
            (*_sceneMat)[iii.Y][iii.X]='~';
        }
        for(auto iii:intFac.forces){
            (*_sceneMat)[iii.Y][iii.X]=static_cast<interface_d::mv_f>(iii).dir;
        }

        //        int testRAndX = rand() % 80 + 0;
        //        int testRAndY = rand() % 25 + 0;
        //        (*_sceneMat)[testRAndY][testRAndX]='F';
        (*_sceneMat)[intFac.car.Y][intFac.car.X]='O';
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
        (*_sceneMat)[tailC.Y][tailC.X]=tailC.tailC;
        _sceneMutex.unlock();
        /*** Send the driver thread to sleep letting the others do their work.*/
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}
}
#endif // DRIVER_CPP
