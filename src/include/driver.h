#ifndef DRIVER_H
#define DRIVER_H
#include<iostream>
#include<mutex>
#include<chrono>
#include<array>
#include<memory>
#include<thread>
using namespace std;

class driver
{
public:
    driver(shared_ptr<array<array<char, 80>, 25>>&,
           mutex&,mutex&,
           volatile const char&,
           volatile const bool&,
           size_t, size_t,
           size_t, size_t);
    void physics();
    thread driverThread(){return thread([this]{this->physics();});}
private:
    size_t _rowC, _colC, _carX, _carY;
    const shared_ptr<array<array<char, 80>, 25>>& _sceneMat;
    mutex& _sceneMutex;
    mutex& _kbdMutex;
    char headReplacement;
    char tailReplacement;
    volatile const char& _kbStat;
    volatile const bool& _quitter;
    char kbStatBuf;
    using hiResClock=chrono::high_resolution_clock ;
    double forwardOn,
            backwardOn,
            leftOn,
            rightOn;

    bool forwardProcessed,
            backwardProcessed,
            leftProcessed,
            rightProcessed;
};

#endif // DRIVER_H
