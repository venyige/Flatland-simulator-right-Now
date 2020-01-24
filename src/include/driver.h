#ifndef DRIVER_H
#define DRIVER_H
#include<iostream>
#include<mutex>
#include<chrono>
#include<array>
#include<memory>
#include<thread>
#include<interface.h>
namespace now{
using namespace std;

template<const size_t rowC, const size_t colC>
class driver
{
public:

    driver(shared_ptr<array<array<char, colC>, rowC>>&,
           mutex&,mutex&,
           volatile const char&,
           volatile const bool&,
           const double&,
           const double&,
           const double&);
    void physics();
    thread driverThread(){return thread([this]{this->physics();});}
    const interface_d& getInterface() const{return intFac;}
private:
    interface_d intFac;
    const size_t _rowC=rowC;
    const size_t _colC=colC;
    size_t _carX, _carY;
    const shared_ptr<array<array<char, colC>, rowC>>& _sceneMat;
    mutex& _sceneMutex;
    mutex& _kbdMutex;
    char headReplacement;///< In case car head gets in a force block

    volatile const char& _kbStat;
    volatile const bool& _quitter;
    char kbStatBuf;
    enum tailChar{u='|', ul='\\', l='-',  dl='/',  d='|', dr='\\', r='-', ur='/'};
    typedef struct{
        char X;
        char Y;
        char tailC=tailChar::u;
        char replacement=' ';
    }tail;
    const double& _maxLinSpeed;
    const double& _maxAngSpeed;
    const double& _maxAcc;
};
}
#include "../driver.cpp"
#endif // DRIVER_H

