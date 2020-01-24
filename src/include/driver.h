#ifndef DRIVER_H
#define DRIVER_H
#include<iostream>
#include<mutex>
//#include<chrono>
//#include<array>
//#include<memory>
#include<thread>
//#include<interface.h>
#include<di_math.h>

namespace now{
using namespace std;

template<const size_t rowC, const size_t colC>
class driver
{
public:

    driver(shared_ptr<array<array<char, colC>, rowC>>&,
           mutex&,mutex&,
           volatile const char&,
           volatile bool&,
           const double&,
           const double&,
           const double&,
           string&);
    void physics();
    thread driverThread(){return thread([this]{this->physics();});}
    const interface_d& getInterface() const{return intFac;}

private:
    interface_d intFac;
    size_t _carX, _carY;
    const shared_ptr<array<array<char, colC>, rowC>>& _sceneMat;
    mutex& _sceneMutex;
    mutex& _kbdMutex;
    char headReplacement;///< In case car head gets in a force block

    volatile const char& _kbStat;
    volatile bool& _quitter;
    char kbStatBuf;


    const double& _maxLinSpeed;
    const double& _maxAngSpeed;
    const double& _maxAcc;
    string& driverMsg;
};

template<size_t rowC, size_t colC>
driver<rowC, colC>::driver(shared_ptr<array<array<char, colC>, rowC>>& sceneMat,
                           mutex& sceneMut,
                           mutex& kbdMut,
                           volatile const char& kbStat,
                           volatile bool& quitter,
                           const double& maxLinSpeed,
                           const double& maxAngSpeed,
                           const double& maxAcc,
                           string& msg):
    _sceneMat(sceneMat),
    _sceneMutex(sceneMut),
    _kbdMutex(kbdMut),
    _kbStat(kbStat),
    _quitter(quitter),
    _maxLinSpeed(maxLinSpeed),
    _maxAngSpeed(maxAngSpeed),
    _maxAcc(maxAcc),
    driverMsg(msg){}

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
    size_t colMax=colC-1;
    size_t rowMax=rowC-1;
    double maxDistance=0.0;///< Theoretical maximum distance (either linear, or angular) with maximum allowed speed in the given time term.
    double epsilon=1e-32;///< Epsilon to judge practical zero value of double precision variables
    using hrClk=chrono::high_resolution_clock ;
    hrClk::time_point lastTime;
    double intervalD;///< t in seconds
    double distance;///< space: v0*t+(a/2)*t^2 or rot: u0*t+(omg/2)*t^2
    double velocity;///< For both linear and angular movement
    tail tailC;///< Structure typedef to store tail data
    bool inForce=false;
    interface_d::mv_b movBuf;///< Temporary storage for car or moving block position. Block type is the proper choice having no other members than X and Y.
    interface_d::mv_b nextBlockPos;///< Temporary storage for checking if collided, whether there is at least one other block in row - stopping condition.
    bool isMobile=true;///< Indicator to check if the collided movable block is mobile
    bool removeMobile=false;


    /*** To initialize the scene matrix, we lock the scene mutex*/
    while(_sceneMutex.try_lock());
    initScene<rowC, colC>( _sceneMat, intFac, tailC);

    /*** Unlock the scene mutex allowing the display driver to show the scene*/
    _sceneMutex.unlock();
    while(!_quitter){
        /*** To accurately calculate the actual position and velocity data
 * chrono::high_resolution_clock features being utilized, casting
 * the high resolution data in seconds to double precision float.*/
        intervalD = chrono::duration_cast<chrono::duration<double>>( hrClk::now()-lastTime).count();

        lastTime=hrClk::now();

        maxDistance=_maxLinSpeed*intervalD;///< maximum velocity * elapsed time
        /*** MOBILE BLOCK CHECK
         * Reads all movable blocks' acceleration.
         * If greater than epsilon, processes the movement.*/

        std::vector<interface_d::mv_tb>::iterator iib = intFac.blocks.begin();
        while (iib != intFac.blocks.end())
        {
            interface_d::mv_tb& iii=*iib;
            distance=0.0;
            velocity=0.0;

            if(iii.linAcc<-epsilon){ ///< Still decelerates
                if(signbit(iii.linVel)){
                    iii.linVel=0.0;
                    iii.linAcc=0.0;
                    iii.icDist=0.0;
                }else{
                    distance=(iii.linAcc * intervalD*intervalD)/2.0;
                    velocity=iii.linAcc * intervalD;
                    if(abs(iii.linVel)>epsilon){
                        distance+=iii.linVel*intervalD;
                        iii.linVel+=velocity;
                    }else{
                        iii.linVel=velocity;
                    }
                    if(iii.linVel>_maxLinSpeed){
                        iii.linVel=_maxLinSpeed;
                    }else if(iii.linVel<-_maxLinSpeed){
                        iii.linVel=-_maxLinSpeed;
                    }
                    if(abs(distance)>epsilon){
                        if((distance)>maxDistance){
                            distance=maxDistance;
                        }else if((distance)<-maxDistance){
                            distance=-maxDistance;
                        }
                        distance+=iii.icDist;
                    }else{
                        distance=0.0;
                        iii.icDist=0.0;
                    }


                    /*** Now we process the linear distance, that is snap if necessary,
                 * and the residual to be stored in the inter-cellar member.
                 * Prior to the snapping, the next position is investigated to
                 * process the mobile block and wall collision cases. These both are
                 * stopping events.
        */
                    getNextPos(nextBlockPos, iii, rowC, colC);
                    isMobile=true; ///< mobility indicator, seems appropriate here
                    /*** Block checking loop
                 * If a block or wall is in track of the moving block, it stops
                 * and zeros all members. */
                    for(interface_d::mv_f fii: intFac.forces){
                        if(fii.X==nextBlockPos.X && fii.Y==nextBlockPos.Y){
                            removeMobile=true;
                            break;
                        }
                    }
                    if(removeMobile){
                        intFac.blocks.erase(iib++);
                        continue;
                    }
                    if(isMobile){
                        for(interface_d::mv_b wii: intFac.blocks){
                            if(wii.X==nextBlockPos.X && wii.Y==nextBlockPos.Y){
                                isMobile=false;
                                break;
                            }
                        }
                    }
                    if(isMobile){
                        for(interface_d::mv_b wii: intFac.walls){
                            if(wii.X==nextBlockPos.X && wii.Y==nextBlockPos.Y){
                                isMobile=false;
                                break;
                            }
                        }
                    }
                    if(!isMobile){
                        iii.icDist=0.0;
                        iii.linVel=0.0;
                        iii.linAcc=0.0;
                    }else{
                        linear_snap(distance, iii, rowMax, colMax);
                        if(iii.linVel>epsilon){
                            iii.icDist=distance;
                        }else{
                            iii.icDist=0.0;
                            iii.linVel=0.0;
                            iii.linAcc=0.0;
                        }
                    }

                }
            }
            iib++;
        }
        /*** Updating the linear data of the car*/
        distance=0.0;
        velocity=0.0;


        inForce=false;
        for(auto iii:intFac.forces){
            if((static_cast<interface_d::mv_f>(iii).X==intFac.car.X)&&(static_cast<interface_d::mv_f>(iii).Y==intFac.car.Y)){
                inForce=true;
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

        if(inForce){
            intFac.car.linVel=_maxLinSpeed;
            intFac.car.linAcc=_maxAcc;
            distance=intFac.car.icDist+maxDistance;
        }else{
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
        }
        /*** Now we process the linear distance, that is snap if necessary,
         * and the residual to be stored in the inter-cellar member.
         * Prior to the snapping, the car position is stored in order to
         * process the mobile block collision case.
*/
        movBuf.X=intFac.car.X;
        movBuf.Y=intFac.car.Y;
        linear_snap(distance, intFac.car, rowMax, colMax);
        /*** Mobile block checking loop
         * If one block is in the track, replaces its velocity
         * with the car's, so the car stops, and the block moves.
         * If more blocks are in row, stops the car. */
        for(interface_d::mv_tb& iii: intFac.blocks){
            if((iii.X==intFac.car.X) && (iii.Y==intFac.car.Y) ){
                /*** Check if more blocks in row. Either '~' or
                 * '#' blocks count in this situation.*/
                getNextPos(nextBlockPos, intFac.car, rowC, colC);
                isMobile=true;
                for(interface_d::mv_tb jjj: intFac.blocks){
                    if(jjj.X==nextBlockPos.X && jjj.Y==nextBlockPos.Y){
                        isMobile=false;
                        break;
                    }
                }
                if(isMobile){
                    for(interface_d::mv_b kkk: intFac.walls){
                        if(kkk.X==nextBlockPos.X && kkk.Y==nextBlockPos.Y){
                            isMobile=false;
                            break;
                        }
                    }

                }
                if(isMobile){
                    iii.linAcc=-1.0;
                    iii.linVel=abs(intFac.car.linVel);
                    iii.icDist=intFac.car.icDist;
                    iii.dir=signbit(intFac.car.linVel)?(intFac.car.dir+4)%8:intFac.car.dir;
                }
                intFac.car.X=movBuf.X;
                intFac.car.Y=movBuf.Y;
                intFac.car.icDist=0.0;
                intFac.car.linVel=0.0;
                break;
            }

        }

        /*** Check for crash. If crash, write message and break cycle - obviously never gets here because of the following fine-check*/
        for(interface_d::mv_b iii: intFac.walls){
            if((iii.X==intFac.car.X) && (iii.Y==intFac.car.Y) ){
                driverMsg="The car crashed to the wall element at Line:"+to_string(intFac.car.Y)+", and Row:"+to_string(intFac.car.X);
                _quitter=true;
            }
        }
        /*** If there is no residual linear speed, the position is to be
         * snapped to the actual grid point. Othervise it has to
         * be updated according to the remaining distance value.
         * Also the fine-check is for collision. If the residual velocity
         * and intercellar distance is positively points towards a wall element,
         * crash event occurs.
*/
        if(abs(intFac.car.linVel)>epsilon){
            intFac.car.icDist=distance;
            if(!signbit(distance) && !signbit(intFac.car.linVel)){
                getNextPos(nextBlockPos, intFac.car, rowC, colC);
                for(interface_d::mv_b iii: intFac.walls){
                    if((nextBlockPos.X==iii.X) &&(nextBlockPos.Y==iii.Y)){
                        driverMsg="The car crashed to the wall element at Line:"+to_string(nextBlockPos.Y)+", and Row:"+to_string(nextBlockPos.X);
                        _quitter=true;

                    }
                }
            }
        }else{
            intFac.car.icDist=0.0;
        }

        /*** Updating the angular data of the car.*/

        distance=0.0;
        velocity=0.0;
        maxDistance=_maxAngSpeed * intervalD;///< maximum velocity * elapsed time
        if(inForce){
            distance=0.0;
            intFac.car.angVel=0.0;
            intFac.car.angAcc=0.0;
        }else{
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
        }
        /*** Now we process the rotation (distance), that is snap if necessary,
         * and the remaining to be stored in the angle member. First check if there
         * is any force field at the actual car position, so to override the direction.*/
        angular_snap(distance, intFac);


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

        (*_sceneMat)[intFac.car.Y][intFac.car.X]='O';

        tail_proc(intFac,tailC,rowMax, colMax);

        (*_sceneMat)[tailC.Y][tailC.X]=tailC.tailC;

        _sceneMutex.unlock();

        /*** Send the driver thread to sleep letting the others do their work.*/
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}
}
//#include "../driver.cpp"
#endif // DRIVER_H

