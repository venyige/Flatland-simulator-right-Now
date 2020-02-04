#ifndef GUARD_AGENT_H
#define GUARD_AGENT_H
#include<interface.h>
#include<iostream>
#include<string>
#include<fstream>
#include<thread>
#include<cmath>
#include<di_math.h>



namespace now{
using namespace std;
/** \brief Guard agent
 * Car driver guard agent class.<br>
 * A collision protection with emergency brake function.*/
class guard_agent
{
public:
    /** \brief Init Constructor:
     * Initializer constructor with all the variables necessary to
     * autopilot and log*/
    guard_agent(const size_t, const size_t, const double, const double, const double,  interface_d&, ofstream&, bool controlCar, bool logOn, bool logVerb, bool logTimely);
    void guard(volatile bool&);
    thread guardThread(volatile bool& quitter){return thread([&quitter, this]{this->guard(quitter);});}
private:
    string guardMsg;///< Message to log
    ofstream& _outLog;///< Temp file stream to enter log into
    const size_t _rowC;///< Rowcount
    const size_t _colC;///< columncount
    const double _maxLinVel;///< Maximum linear velocity from main()
    const double _maxAngVel;///< Maximum angular velocity from main()
    const double _maxAcc;///< Maximum linear/angular acceleration from main()
    interface_d& _intFac;///< Interface with the car driver thread
    const double safeC=1.2;///< Safety constant. Multiplicator of maximum distance taken in the time-slice
    vector<interface_d::mv_b> guardWalls;///< Walls and deathtraps
    bool _logOn, _logVerb, _logTm, _ctrCar;
};


}
#endif // GUARD_AGENT_H
