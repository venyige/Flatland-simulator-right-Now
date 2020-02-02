#ifndef GUARD_AGENT_CPP
#define GUARD_AGENT_CPP
#include "guard_agent.h"
namespace now{
guard_agent::guard_agent(const char rowC,
                         const char colC,
                         const double maxLinVel,
                         const double maxAngVel,
                         const double maxAcc,
                         interface_d& intFac,
                         ofstream& outLog):
    _outLog(outLog),
    _rowC(rowC),
    _colC(colC),
    _maxLinVel(maxLinVel),
    _maxAngVel(maxAngVel),
    _maxAcc(maxAcc),
    _intFac(intFac)
    {}

inline void carPosLog(ofstream& outLog, interface_d& intFac){

    outLog<<string(12,' ')<<"Measured timeslot="<<to_string(intFac.tLag)<<endl;
    outLog<<string(12,' ')<<"Emergency brake="<<to_string(intFac.emBrake)<<endl;
    outLog<<string(12,' ')<<"Position data: X="<<to_string(intFac.car.X)<<", Y="<<to_string(intFac.car.Y)<<endl;
    outLog<<string(12,' ')<<"Direction="<<to_string(intFac.car.dir)<<endl;
    outLog<<string(12,' ')<<"Linear acceleration="<<to_string(intFac.car.linAcc)<<endl;
    outLog<<string(12,' ')<<"Linear speed="<<to_string(intFac.car.linVel)<<endl;
    outLog<<string(12,' ')<<"Angular acceleration="<<to_string(intFac.car.angAcc)<<endl;
    outLog<<string(12,' ')<<"Angular speed="<<to_string(intFac.car.angVel)<<endl;
    outLog<<string(12,' ')<<"Residual angle="<<to_string(intFac.car.angle)<<endl;
    outLog<<string(12,' ')<<"Residual trace="<<to_string(intFac.car.icDist)<<endl<<endl;

}
inline interface_d::blockType checkCollision(char _rowC,
                           char _colC,
                           uint8_t brakeRange,
                           interface_d::mv_t& testCar,
                           interface_d::mv_b& nextPos,
                           const interface_d& intFac,
                           const function<bool(const interface_d::mv_b&)>& isWallAt,
                           bool& emergencyBrakeOn){
    emergencyBrakeOn=false;
    interface_d::blockType retVal=interface_d::blockType::noblock;
    vector<interface_d::mv_b>::const_iterator iiWall;
    vector<interface_d::mv_tb>::const_iterator iiBlk;
    vector<interface_d::mv_f>::const_iterator iiFf;
    for(uint8_t iii=0; iii<uint8_t(round(1.5*brakeRange)); iii++){
        getNextPos( nextPos, testCar, _rowC, _colC);
        iiWall=find_if(intFac.walls.begin(), intFac.walls.end(), isWallAt);
        if(iiWall!=intFac.walls.end()){
            retVal=interface_d::wall_t;
            if(iii<brakeRange){
                emergencyBrakeOn=true;
            }
            break;
        }
        iiBlk=find_if(intFac.blocks.begin(), intFac.blocks.end(), isWallAt);
        if(iiBlk!=intFac.blocks.end()){
            retVal=interface_d::mobile_t;
            break;
        }
        iiFf=find_if(intFac.forces.begin(), intFac.forces.end(), isWallAt);
        if(iiFf!=intFac.forces.end()){
            retVal=interface_d::ffield_t;
            break;
        }
        testCar.X=nextPos.X;
        testCar.Y=nextPos.Y;
    }
    return retVal;

}
void guard_agent::guard(volatile bool& quitter){
    interface_d::mv_t testCar;
    interface_d::mv_b nextPos;
    interface_d::mv_b logPos;
    bool ifLog=false;
    double brakeTime=0.0;
    unsigned char brakeRange=1;
    using hrClk=chrono::high_resolution_clock ;
    double logFreq=.2;//< log frequency if the car is not moving
    hrClk::time_point lastTime;
    bool emergencyBrakeOn=false;
    double brakeRangeDbl;
    const double sqrt2=sqrt(2.0);
    uint8_t emBrake;
    interface_d::blockType chkRes;
    /** Time slippage from command to intervention \snippet this tslip*/
//[tslip]
    const double timeSlip=safeC*double(interface_d::tSlot)*1e-3;
//[tslip]
    function<bool(const interface_d::mv_b&)> isWallAt = [&nextPos](interface_d::mv_b iiW)->bool{return (iiW.X==nextPos.X)&&(iiW.Y==nextPos.Y);};

    lastTime=hrClk::now();
    while(!quitter){
        {// from here _intFac._mutex lock to the closing brace
            const std::lock_guard<std::mutex> lock(_intFac._mutex);

        emBrake=0;

        testCar=_intFac.car;
        ifLog=false;
        if((logPos.X!=testCar.X)||(logPos.Y!=testCar.Y)){
            ifLog=true;
            lastTime=hrClk::now();
            logPos.X=testCar.X;
            logPos.Y=testCar.Y;
//            carPosLog(this->_outLog, _intFac);
        }
        if(abs(testCar.linVel)>_intFac.epsilon){
            brakeTime= abs((testCar.linVel+testCar.linAcc*timeSlip)/_maxAcc); // max value: _maxLinVel/_maxAcc;
            brakeRangeDbl=abs((testCar.linVel+testCar.linAcc*timeSlip)*(brakeTime+timeSlip))-
                    this->_maxAcc/2.0*(brakeTime-timeSlip)*(brakeTime-timeSlip)+
                    (signbit(testCar.linVel)?-testCar.icDist:testCar.icDist)+
                    (((testCar.dir==1)||
                      (testCar.dir==3)||
                      (testCar.dir==5)||
                      (testCar.dir==7))?sqrt2/2.0:0.5)+
                    _intFac.tLag*_maxLinVel*safeC;
            brakeRange=uint8_t(round(brakeRangeDbl/(((testCar.dir==1)||
                                                     (testCar.dir==3)||
                                                     (testCar.dir==5)||
                                                     (testCar.dir==7))?sqrt2:1.0)));
        }else{
            brakeRange=1;
        }

        emergencyBrakeOn=false;
        /** Check if taking control or warning is needed - linear motion: \snippet this f_l_brake*/
//[f_l_brake]
        if((chkRes=checkCollision(_rowC,
                          _colC,
                          brakeRange,
                          testCar,
                          nextPos,
                          _intFac,
                          isWallAt,
                          emergencyBrakeOn))!=interface_d::blockType::noblock){
            switch (chkRes) {
            case interface_d::blockType::wall_t:
                if(emergencyBrakeOn){
                    if(!signbit(testCar.linVel)){
                        emBrake|=1; //forward linear acceleration lock
                    }else{
                        emBrake|=4; //backward linear acceleration lock
                    }
                    if(ifLog){
                        this->_outLog<<"Collision emergency brake applied!\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl<<"emergency brake: "<<to_string(emBrake)<<endl;
                    }
                }else{
                    if(ifLog)
                        this->_outLog<<"Warning!\nCollision emergency.\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                }
                break;
            case interface_d::blockType::mobile_t:
                if(ifLog)
                    this->_outLog<<"Warning!\nObstacle on track.\nMovable block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                break;
            case interface_d::blockType::ffield_t:
                if(ifLog)
                    this->_outLog<<"Warning!\nForce field danger.\nForce field position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                break;
            default:
                break;
            }
        }
//[f_l_brake]
        testCar=_intFac.car;
        if((abs(testCar.linVel)>_intFac.epsilon)&&(abs(testCar.angVel)>_intFac.epsilon)){
            if(signbit(testCar.angVel)){
                if(testCar.dir==0){
                    testCar.dir=7;
                }else{
                    testCar.dir--;
                }
            }else {
                if(testCar.dir==7){
                    testCar.dir=0;
                }else{
                    testCar.dir++;
                }

            }
            if((chkRes=checkCollision(_rowC,
                              _colC,
                              brakeRange,
                              testCar,
                              nextPos,
                              _intFac,
                              isWallAt,
                                      emergencyBrakeOn))!=interface_d::blockType::noblock){
                switch (chkRes) {
                case interface_d::blockType::wall_t:
                    if(emergencyBrakeOn){
                        if(!signbit(testCar.angVel)){
                            emBrake|=2; //forward angular acceleration lock
                        }else{
                            emBrake|=8; //backward angular acceleration lock
                        }
                        if(ifLog){
                            this->_outLog<<"Collision emergency steering lock applied!\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                        }
                    }else{
                        if(ifLog)
                            this->_outLog<<"Warning!\nCollision emergency.\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                    }
                    break;
                case interface_d::blockType::mobile_t:
                    if(ifLog)
                        this->_outLog<<"Warning!\nObstacle on track.\nMovable block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                    break;
                case interface_d::blockType::ffield_t:
                    if(ifLog)
                        this->_outLog<<"Warning!\nForce field danger.\nForce field position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                    break;
                default:
                    break;
                }
            }

        }

        if(chrono::duration_cast<chrono::duration<double>>( hrClk::now()-lastTime).count()>logFreq){
            ifLog=true;
            lastTime=hrClk::now();
//            carPosLog(this->_outLog, _intFac);
        }



        _intFac.emBrake=emBrake;
        }
//        _intFac._mutex.unlock();
        /** Send the guardian agent thread to sleep letting the others do their work.\snippet this sleep*/
//[sleep]
        std::this_thread::sleep_for(std::chrono::milliseconds(interface_d::tSlot));
//[sleep]

    }
}

}
#endif // GUARD_AGENT_CPP
