#ifndef DRIVER_H
#define DRIVER_H
#include<iostream>
#include<mutex>
//#include<chrono>
//#include<array>
//#include<memory>
#include<thread>
#include<interface.h>
#include<di_math.h>

namespace now{
using namespace std;
/** \brief Car driver:
 * Car driver class to hold the driver thread's main loop function - \see physics */
template<const size_t rowC, const size_t colC>
class driver
{
public:
/** Data initializer constructor
*/
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
    interface_d& getInterface(){return intFac;}

private:
    interface_d intFac;
    size_t _carX, _carY;
    const shared_ptr<array<array<char, colC>, rowC>>& _sceneMat;///< Scene matrix
    mutex& _sceneMutex;///< Scene mutex, shared with the display driver
    mutex& _kbdMutex;///< Keyboard mutex, shared with the keyboard driver

    volatile const char& _kbStat;///< Keyboard state byte
    volatile bool& _quitter;///< Quitter control boolean
    char kbStatBuf;///< Local copy of keyboard state byte


    const double& _maxLinSpeed;///< Maximum linear speed const from main()
    const double& _maxAngSpeed;///< Maximum angular speed const from main()
    const double& _maxAcc;///< Maximum linear/angular acceleration const from main()
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

/**
* <h3>Car driver physics thread function</h3>
* <ol>
 * <li> Initialization step preparing the scene data to display, and
 *  extracting the trafic limiting object into the interface class's
 *  appropriate vectors in order to make them available to the guardian
 *  agent.</li><br>
 * <b><i>After initialization the main loop starts:</i></b>
 * <li> Updating the linear data based on:
 * - actual linear acceleration,
 * - actual linear velocity,
 * - actual residual (not jet snapped) distance,
 * - car direction.
 * </li>
 * <li> Updating the angular data based on:
 * - actual angular acceleration,
 * - actual rotation speed,
 * - residual angle data.
 * </li>
 * <li> Updating both the linear and angular acceleration data based on
 * the "quasi realtime" keyboard status flags.</li>
 * </ol>*/
template<size_t rowC, size_t colC>
void driver<rowC, colC>::physics(){
    /**
    * <h3> Local variables worth brief:</h3>
    * \b maxDistance Theoretical maximum distance (either linear, or angular) with maximum allowed speed in the given time term.<br>
    * \b scColCnt Column count<br><br>
    * \b typedef: \b hrClk High resolution clock<br>
    * \b lastTime The logged point in time of the latest cycle of the loop<br>
    * \b intervalD t in seconds<br>
    * \b distance space: \f$v_0\cdot t+\frac{a}{2}\cdot t^2\f$ or rot: \f$\omega_0\cdot t+\frac{\epsilon}{2}\cdot t^2\f$<br>
    * \b velocity For both linear and angular movement<br>
    * \b tailC Structure to store tail data<br>
    * \b movBuf Temporary storage for car or moving block position. Block type is the proper choice having no other members than X and Y.<br>
    * \b nextBlockPos Temporary storage for checking if collided, whether there is at least one other block in row - stopping condition.<br>
    \snippet this local_vars*/
    size_t colMax=colC-1;
    size_t rowMax=rowC-1;
//[local_vars]
    double maxDistance=0.0;
    using hrClk=chrono::high_resolution_clock ;
    hrClk::time_point lastTime;
    double intervalD;
    double distance;
    double velocity;
    tail tailC;
    interface_d::mv_b movBuf;
    interface_d::mv_b nextBlockPos;
//[local_vars]
    bool inForce=false;
    bool isMobile=true;
    bool removeMobile=false;
uint8_t tempBrake;

/** \brief To initialize the scene matrix, we lock the scene mutex: \snippet this 1*/
//[1]
    while(_sceneMutex.try_lock());
//[1]
    /** \brief Scene initialization
     *  Initializing scene before thread main loop starts
     * \b _sceneMat Scene matrix
     * \b intFac Interface to share dynamic scene data
     * \b tailC type:tail car tail data  \snippet this 1.1*/
    //[1.1]
    initScene<rowC, colC>( _sceneMat, intFac, tailC);
    //[1.1]

    /** Unlock the scene mutex allowing the display driver to show the scene \snippet this 2*/
//[2]
    _sceneMutex.unlock();
//[2]
    lastTime=hrClk::now();
    while(!_quitter){
/** To accurately calculate the actual position and velocity data
 * chrono::high_resolution_clock features being utilized, casting
 * the high resolution data in seconds to double precision float.\snippet this 3.1*/
//[3.1]
        intervalD = chrono::duration_cast<chrono::duration<double>>( hrClk::now()-lastTime).count();

        lastTime=hrClk::now();
//[3.1]

        /** Try to grab the interface mutex in order to start manipulating its data \snippet this interface_mutex*/
//[interface_mutex]

        {//from here to the closing brace intFac._mutex is locked
            const std::lock_guard<std::mutex> lock(intFac._mutex);

//[interface_mutex]
        maxDistance=_maxLinSpeed*intervalD;//< \a  = maximum velocity * elapsed time
        if(intFac.tLag<intervalD){
            intFac.tLag=intervalD;
        }
/** MOBILE BLOCK CHECK
 * Reads all movable blocks' acceleration.
 * If greater than epsilon, processes the movement.\snippet this 4*/

        removeMobile=false;
//[4]
        std::vector<interface_d::mv_tb>::iterator iib = intFac.blocks.begin();
        while (iib != intFac.blocks.end())
//[4]
        {
            interface_d::mv_tb& iii=*iib;
            distance=0.0;
            velocity=0.0;

            if(iii.linAcc<-intFac.epsilon){ // Still decelerates
                if(signbit(iii.linVel)){
                    iii.linVel=0.0;
                    iii.linAcc=0.0;
                    iii.icDist=0.0;
                }else{
                    distance=(iii.linAcc * intervalD*intervalD)/2.0;
                    velocity=iii.linAcc * intervalD;
                    if(abs(iii.linVel)>intFac.epsilon){
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
                    if(abs(distance)>intFac.epsilon){
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


 /* Processing the linear distance, that is snap if necessary,
 * and the residual to be stored in the inter-cellar member.
 * Prior to the snapping, the next position is investigated to
 * process the mobile block and wall collision cases. These both are
 * stopping events.
        */
                    getNextPos(nextBlockPos, iii, rowC, colC);
                    isMobile=true; //< mobility indicator, seems appropriate here
/** Block checking loop
* If a block or wall is in track of the moving block, it stops
* and zeros all members.\snippet this 5*/
//[5]
                    for(interface_d::mv_f fii: intFac.forces){
                        if(fii.X==nextBlockPos.X && fii.Y==nextBlockPos.Y){
                            removeMobile=true;
                            break;
                        }
                    }
                    if(removeMobile){
                        intFac.blocks.erase(iib++);
                        removeMobile=false;
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
                        linear_snap(distance, iii, char(rowMax), char(colMax));
                        if(iii.linVel>intFac.epsilon){
                            iii.icDist=distance;
                        }else{
                            iii.icDist=0.0;
                            iii.linVel=0.0;
                            iii.linAcc=0.0;
                        }
                    }
//[5]
                }
            }
            iib++;
        }
        /* Updating the linear data of the car*/
        distance=0.0;
        velocity=0.0;


        inForce=false;
        for(auto iii:intFac.forces){
            if((static_cast<interface_d::mv_f>(iii).X==intFac.car.X)&&(static_cast<interface_d::mv_f>(iii).Y==intFac.car.Y)){
                inForce=true;
                intFac.car.dir=static_cast<interface_d::mv_f>(iii).dir;
            }
        }

        if(inForce){
            intFac.car.linVel=_maxLinSpeed;
            intFac.car.linAcc=_maxAcc;
            distance=intFac.car.icDist+maxDistance;
        }else{
            if(abs(intFac.car.linAcc)>intFac.epsilon){
                distance=(intFac.car.linAcc * intervalD*intervalD)/2.0;
                velocity=intFac.car.linAcc * intervalD;
            }
            if(abs(intFac.car.linVel)>intFac.epsilon){
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

            if(abs(distance)>intFac.epsilon){
                if((distance)>maxDistance){
                    distance=maxDistance;
                }else if((distance)<-maxDistance){
                    distance=-maxDistance;
                }
                distance+=intFac.car.icDist;
            }else{
                distance=0.0;
            }
/** This part's role to stop the car drifting \snippet this emg_brake*/
//[emg_brake]
                        if(!(kbStatBuf & 5)){//< No linear accelerator key is being pressed
                            if(signbit(intFac.car.linAcc)==signbit(intFac.car.linVel)){
                                intFac.car.linAcc=0.0;
                                intFac.car.linVel=0.0;
                                intFac.car.icDist=0.0;
                                distance=0.0;
                            }
                        }
//[emg_brake]
        }
/** Processing the linear distance, that is snap if necessary,
 * and the residual to be stored in the inter-cellar member.
 * Prior to the snapping, the car position is stored in order to
 * process the mobile block collision case.
*/
        movBuf.X=intFac.car.X;
        movBuf.Y=intFac.car.Y;
        linear_snap(distance, intFac.car, rowMax, colMax);
/** Mobile block checking loop
 * If one block is in the track, replaces its velocity
 * with the car's, so the car stops, and the block moves.
 * If more blocks are in row, stops the car. */
        for(interface_d::mv_tb& iii: intFac.blocks){
            if((iii.X==intFac.car.X) && (iii.Y==intFac.car.Y) ){
                /** Check if more blocks in row. Either '~' or
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
                    iii.linAcc=-_maxAcc;
                    iii.linVel=abs(intFac.car.linVel);
                    iii.icDist=intFac.car.icDist;
                    iii.dir=signbit(intFac.car.linVel)?int(d_t(intFac.car.dir)+4):int(intFac.car.dir);
                }
                intFac.car.X=movBuf.X;
                intFac.car.Y=movBuf.Y;
                intFac.car.icDist=0.0;
                intFac.car.linVel=0.0;
                break;
            }

        }

        /** Check for crash. If crash, write message and break cycle - obviously never gets here because of the following fine-check\snippet this 6*/
//[6]
        for(interface_d::mv_b iii: intFac.walls){
            if((iii.X==intFac.car.X) && (iii.Y==intFac.car.Y) ){
                driverMsg="The car crashed to the wall element at Line:"+to_string(intFac.car.Y)+", and Row:"+to_string(intFac.car.X);
                _quitter=true;
            }
        }
//[6]
        /** If there is no residual linear speed, the position is to be
         * snapped to the actual grid point. Othervise it has to
         * be updated according to the remaining distance value.
         * Also the fine-check is for collision. If the residual velocity
         * and intercellar distance is positively points towards a wall element,
         * crash event occurs.\snippet this 7*/
//[7]
        if(abs(intFac.car.linVel)>intFac.epsilon){
            intFac.car.icDist=distance;
            if(!signbit(distance) && !signbit(intFac.car.linVel)){
                getNextPos(nextBlockPos, intFac.car, rowC, colC);
                for(interface_d::mv_b iii: intFac.walls){
                    if((nextBlockPos.X==iii.X) &&(nextBlockPos.Y==iii.Y)){
                        driverMsg="The car crashed to the wall element at Line:"+to_string(nextBlockPos.Y)+", and Row:"+to_string(nextBlockPos.X)+"\n";
                        driverMsg+="Em. brake mask:"+to_string(tempBrake)+"\n";
                        driverMsg+="Linear accel.:"+to_string(intFac.car.linAcc)+", angular accel.:"+to_string(intFac.car.angAcc)+"\n";
                        driverMsg+="Linear speed:"+to_string(intFac.car.linVel)+", angular speed:"+to_string(intFac.car.angVel)+"\n";
                        driverMsg+="Intercell dist.:"+to_string(intFac.car.icDist)+", residual angle:"+to_string(intFac.car.angle)+"\n";
                        _quitter=true;

                    }
                }
            }
        }else{
            intFac.car.icDist=0.0;
        }
//[7]
        /* Updating the angular data of the car.*/

        distance=0.0;
        velocity=0.0;

        /** Maximum theoretical angular distance is calculated \snippet this 7.1*/
//[7.1]
        maxDistance=_maxAngSpeed * intervalD;//< maximum angular velocity * elapsed time
//[7.1]



        if(inForce){
            distance=0.0;
            intFac.car.angVel=0.0;
            intFac.car.angAcc=0.0;
        }else{
            if(abs(intFac.car.angAcc)>intFac.epsilon){
                distance=(intFac.car.angAcc * intervalD*intervalD)/2.0;
                velocity=intFac.car.angAcc * intervalD;
            }
            if(abs(intFac.car.angVel)>intFac.epsilon){
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

            if(abs(distance)>intFac.epsilon){
                if((distance)>maxDistance){
                    distance=maxDistance;
                }else if((distance)<-maxDistance){
                    distance=-maxDistance;
                }
                distance+=intFac.car.angle;
            }else{
                distance=0.0;
            }
/** This part's role to stop the angular drifting \snippet this ang_brake*/
//[ang_brake]
            if(!(kbStatBuf & 10)){//< No angular accelerator key is being pressed
                if(signbit(intFac.car.angAcc)==signbit(intFac.car.angVel)){
                    intFac.car.angAcc=0.0;
                    intFac.car.angVel=0.0;
                    intFac.car.angle=0.0;
                    distance=0.0;
                }
            }
//[ang_brake]
        }
        /* Processing the rotation (distance), that is snap if necessary,
         * and the remaining to be stored in the angle member. First check if there
         * is any force field at the actual car position, so to override the direction.*/
        angular_snap(distance, intFac);


        /** If there is no rotation speed remained, the rotation is to be
         * snapped to any of the clear named directions. Othervise it has to
         * be updated according to the remaining angle value (distance)\snippet this 8*/
//[8]
        if(abs(intFac.car.angVel)>intFac.epsilon){
            intFac.car.angle=distance;
        }else{
            intFac.car.angle=0.0;
        }
//[8]
        /** Now trying to gain control on the keyboard mutex
         * in order to read its value without the chance of being
         * overwritten during the operation.\snippet this 9*/
//[9]
        while(_kbdMutex.try_lock());
//[9]
        if(kbStatBuf!=_kbStat){
            kbStatBuf=_kbStat;
        }
        _kbdMutex.unlock();
/** If emergency brake mask is ON, then the according acceleration
 * key values set to zero in keyboard status buffer. \snippet this emergency_brake*/
//[emergency_brake]
        if(intFac.emBrake){
            kbStatBuf&=(uint8_t(-1)^intFac.emBrake);
        }
//[emergency_brake]
        tempBrake=intFac.emBrake;
        checkKbrdKeypair(kbStatBuf&1,
                         kbStatBuf&4,
                         intFac.car.linVel,
                         intFac.car.linAcc,
                         _maxAcc,
                         intFac.epsilon);

        checkKbrdKeypair(kbStatBuf&2,
                         kbStatBuf&8,
                         intFac.car.angVel,
                         intFac.car.angAcc,
                         _maxAcc,
                         intFac.epsilon);
}
        {//from here to the closing brace _sceneMutex is locked
            const std::lock_guard<std::mutex> lock(_sceneMutex);

            for(auto& iii: (*_sceneMat)){
                iii.fill(' ');//< fill with spaces (ascii 32)
            }
            for(auto iii:intFac.walls){
                (*_sceneMat)[iii.Y][iii.X]='#';
            }
            for(auto iii:intFac.blocks){
                (*_sceneMat)[iii.Y][iii.X]='~';
            }
            for(auto iii:intFac.forces){
                (*_sceneMat)[iii.Y][iii.X]=char(static_cast<interface_d::mv_f>(iii).dir);
            }

            (*_sceneMat)[intFac.car.Y][intFac.car.X]='O';

            tail_proc(intFac,tailC,rowMax, colMax);

            (*_sceneMat)[tailC.Y][tailC.X]=tailC.tailC;

        }

        /** Send the driver thread to sleep letting the others do their work.\snippet this 10*/
//[10]
        std::this_thread::sleep_for(std::chrono::milliseconds(interface_d::tSlot));
//[10]
    }
}
}
/** \endcode */
//#include "../driver.cpp"
#endif // DRIVER_H

