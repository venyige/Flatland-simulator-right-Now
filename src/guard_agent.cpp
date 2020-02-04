#ifndef GUARD_AGENT_CPP
#define GUARD_AGENT_CPP
#include "guard_agent.h"
namespace now{
guard_agent::guard_agent(const size_t rowC,
                         const size_t colC,
                         const double maxLinVel,
                         const double maxAngVel,
                         const double maxAcc,
                         interface_d& intFac,
                         ofstream& outLog,
                         bool ctrCar,
                         bool logOn,
                         bool logVerb,
                         bool logTml,
                         bool dtPlus):
    _outLog(outLog),
    _rowC(rowC),
    _colC(colC),
    _maxLinVel(maxLinVel),
    _maxAngVel(maxAngVel),
    _maxAcc(maxAcc),
    _intFac(intFac),
    _ctrCar(ctrCar),
    _logOn(logOn),
    _logVerb(logVerb),
    _logTm(logTml),
    _dtPlus(dtPlus)
    {}

void collectTraps(const size_t rowC,
                  const size_t colC,
                  interface_d::mv_t& tCar,
                  vector<interface_d::mv_b>& chkList,
                  vector<interface_d::mv_b>& flist,
                  const interface_d& intFac,
                  const size_t stopRange,
                  bool& isTrap,
                  vector<interface_d::mv_f>::const_iterator& iiForce,
                  vector<interface_d::mv_b>::const_iterator& iiWall,
                  interface_d::mv_b& nextPos,
                  bool& dtPlus
                  ){

    for(uint8_t rii=0;rii<stopRange;rii++){
        getNextPos(nextPos, tCar, rowC, colC);
        iiWall=find_if(intFac.walls.begin(), intFac.walls.end(), [&nextPos](const interface_d::mv_b& ffi)->bool{return ((ffi.X==nextPos.X)&&(ffi.Y==nextPos.Y));});
            if(iiWall!=intFac.walls.end()){
 //               flist.push_back(interface_d::mv_f(*iiForce));
                isTrap=true;
                return;
            }
        iiForce=find_if(intFac.forces.begin(), intFac.forces.end(), [&nextPos](const interface_d::mv_f& ffi)->bool{return ((ffi.X==nextPos.X)&&(ffi.Y==nextPos.Y));});
        if(iiForce!=intFac.forces.end()){


            if((chkList.end()==find_if(chkList.begin(), chkList.end(), [&nextPos](const interface_d::mv_b& ffi)->bool{return ((ffi.X==nextPos.X)&&(ffi.Y==nextPos.Y));}))&&
                    (flist.end()==find_if(flist.begin(), flist.end(), [&nextPos](const interface_d::mv_b& ffi)->bool{return ((ffi.X==nextPos.X)&&(ffi.Y==nextPos.Y));}))){

                flist.push_back(interface_d::mv_f(*iiForce));
            }
            if((tCar.dir-iiForce->dir)==4){
                if(dtPlus)
                    isTrap=true;
                return;
            }


            tCar.X=nextPos.X;
            tCar.Y=nextPos.Y;
            tCar.dir=iiForce->dir;


            collectTraps(rowC, colC, tCar, chkList, flist, intFac, stopRange, isTrap, iiForce,iiWall, nextPos, dtPlus);
            return;

        }
        tCar.X=nextPos.X;
        tCar.Y=nextPos.Y;

    }
}

/** Localize deathtraps*/
inline void locDeathtraps(
        interface_d& intFac,///< Interface struct with all the necessary data about scene blocks.
        vector<interface_d::mv_b>& wallVec,///< Guardien agent's own vector of wall blocks.
        const double maxLinVel,///< Max linar velocity of car.
        const double maxAcc,///< Max linar acceleration of car.
        const size_t rowC,///< Max linar acceleration of car.
        const size_t colC,///< Max linar acceleration of car.
        const function<bool(const interface_d::mv_b&)>& isWallAt,
        const function<bool(const interface_d::mv_f&)>& isForceAt,
        bool& dtPlus
        ){

    const double tStop=maxLinVel/maxAcc;
    const double dStop=maxLinVel*tStop-maxAcc*tStop*tStop/2.0;
    const size_t stopRange=size_t(round(dStop));
    interface_d::mv_t tCar;
    tCar.linAcc=tCar.linVel=maxLinVel;
    vector<interface_d::mv_b> checkedList{0};
    vector<interface_d::mv_b> checkedInOneRun{0};
    bool isTrap=false;
    vector<interface_d::mv_b>::const_iterator iiWall;
    vector<interface_d::mv_f>::const_iterator iiForce;
    interface_d::mv_b nextPos;

    for(size_t iii=0; iii<intFac.forces.size(); iii++){
        nextPos.X=intFac.forces[iii].X;
        nextPos.Y=intFac.forces[iii].Y;

        if(checkedList.end()!=find_if(checkedList.begin(), checkedList.end(), [&nextPos](const interface_d::mv_b& ffi)->bool{return ((ffi.X==nextPos.X)&&(ffi.Y==nextPos.Y));})){
            continue;
        }
        tCar.X=intFac.forces[iii].X;
        tCar.Y=intFac.forces[iii].Y;
        tCar.dir=intFac.forces[iii].dir;
        isTrap=false;
        checkedInOneRun.push_back(intFac.forces[iii]);
        collectTraps(rowC, colC, tCar, checkedList, checkedInOneRun, intFac, stopRange, isTrap, iiForce, iiWall, nextPos, dtPlus);

        if(isTrap){
            wallVec.insert(wallVec.end(), checkedInOneRun.begin(), checkedInOneRun.end());
        }
        checkedList.insert(checkedList.end(), checkedInOneRun.begin(), checkedInOneRun.end());
        checkedInOneRun.clear();
    }
}
inline void carPosLog(ofstream& outLog, interface_d& intFac, stringstream& logStream, bool logVerbose){
    if(logStream.str().length()){
        outLog<<logStream.str();
        logStream.str(string{0});
    }
    if(logVerbose){
        outLog<<string(12,' ')<<"Measured timeslot="<<to_string(intFac.tLag)<<endl;
        outLog<<string(12,' ')<<"Emergency brake="<<to_string(intFac.emBrake)<<endl;
        outLog<<string(12,' ')<<"Position data: X="<<to_string(intFac.car.X)<<", Y="<<to_string(intFac.car.Y)<<endl;
        outLog<<string(12,' ')<<"Direction="<<to_string(int(intFac.car.dir))<<endl;
        outLog<<string(12,' ')<<"Linear acceleration="<<to_string(intFac.car.linAcc)<<endl;
        outLog<<string(12,' ')<<"Linear speed="<<to_string(intFac.car.linVel)<<endl;
        outLog<<string(12,' ')<<"Angular acceleration="<<to_string(intFac.car.angAcc)<<endl;
        outLog<<string(12,' ')<<"Angular speed="<<to_string(intFac.car.angVel)<<endl;
        outLog<<string(12,' ')<<"Residual angle="<<to_string(intFac.car.angle)<<endl;
        outLog<<string(12,' ')<<"Residual trace="<<to_string(intFac.car.icDist)<<endl<<endl;
    }
    outLog.flush();
}
inline interface_d::blockType checkCollision(const size_t _rowC,
                           const size_t _colC,
                           vector<interface_d::mv_b>& guardWalls,
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
        iiWall=find_if(guardWalls.begin(), guardWalls.end(), isWallAt);
        if(iiWall!=guardWalls.end()){
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
    interface_d::mv_t logPos;
    bool ifLog=false;
    stringstream logStream(guardMsg);
    double brakeTime=0.0;
    double turnTime=0.0;
    unsigned char brakeRange=1;
    unsigned char turnRange=0;
    using hrClk=chrono::high_resolution_clock ;
    double logFreq=.2;//< log frequency if the car is not moving
    hrClk::time_point lastTime;
    bool emergencyBrakeOn=false;
    double brakeRangeDbl;
    guardWalls = _intFac.walls;
    const double sqrt2=sqrt(2.0);
    uint8_t emBrake;
    interface_d::blockType chkRes;
    /** Time slippage from command to intervention \snippet this tslip*/
//[tslip]
    const double timeSlip=safeC*double(interface_d::tSlot)*1e-3;
//[tslip]
    function<bool(const interface_d::mv_b&)> isWallAt = [&nextPos](auto iiW)->bool{return (iiW.X==nextPos.X)&&(iiW.Y==nextPos.Y);};
    function<bool(const interface_d::mv_f&)> isForceAt = [&nextPos](auto iiW)->bool{return (iiW.X==nextPos.X)&&(iiW.Y==nextPos.Y);};

    locDeathtraps(_intFac, guardWalls, _maxLinVel, _maxAcc, _rowC, _colC, isWallAt, isForceAt, _dtPlus);
//    for_each(guardWalls.begin(), guardWalls.end(), [&](interface_d::mv_b& iii){this->_outLog<<"x: "<<to_string(iii.X)<<", y:"<<to_string(iii.Y)<<endl;});
//        this->_outLog<<"Deathtrap size: "<<to_string(guardWalls.size())<<endl;
    lastTime=hrClk::now();

    while(!quitter){
        {// from here _intFac._mutex lock to the closing brace
            const std::lock_guard<std::mutex> lock(_intFac._mutex);

        emBrake=0;
        ifLog=false;
        testCar=_intFac.car;
        if((logPos.X!=testCar.X)||(logPos.Y!=testCar.Y)||(int(logPos.dir)!=int(testCar.dir))){
            lastTime=hrClk::now();
            logPos.X=testCar.X;
            logPos.Y=testCar.Y;
            logPos.dir=testCar.dir;
            if(_logOn){
                ifLog=true;
            }
        }
        if((chrono::duration_cast<chrono::duration<double>>( hrClk::now()-lastTime).count()>logFreq)){
            lastTime=hrClk::now();
            if(_logOn && _logTm){
                ifLog=true;
            }
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
                                  guardWalls,
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
                        logStream<<"Collision emergency brake"<<
                                   (_ctrCar?" has been":" should have been")<<
                                   " applied!\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl<<"emergency brake: "<<to_string(emBrake)<<endl;
                    }
                }else{
                    if(ifLog)
                        logStream<<"Warning!\nCollision emergency.\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                }
                break;
            case interface_d::blockType::mobile_t:
                if(ifLog)
                    logStream<<"Warning!\nObstacle on track.\nMovable block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                break;
            case interface_d::blockType::ffield_t:
                if(ifLog)
                    logStream<<"Warning!\nForce field danger.\nForce field position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
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
                                      guardWalls,
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
                            logStream<<"Collision emergency steering lock"<<
                                       (_ctrCar?" has been":" should have been")<<
                                       " applied!\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                        }
                    }else{
                        if(ifLog)
                            logStream<<"Warning!\nCollision emergency.\nWall block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                    }
                    break;
                case interface_d::blockType::mobile_t:
                    if(ifLog)
                        logStream<<"Warning!\nObstacle on track.\nMovable block position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                    break;
                case interface_d::blockType::ffield_t:
                    if(ifLog)
                        logStream<<"Warning!\nForce field danger.\nForce field position: X="<<to_string(nextPos.X)<< " Y="<<to_string(nextPos.Y)<<endl;
                    break;
                default:
                    break;
                }
            }

        }
        if(ifLog){
            carPosLog(_outLog, _intFac, logStream, _logVerb);
        }
        if(_ctrCar){
            _intFac.emBrake=emBrake;
        }
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
