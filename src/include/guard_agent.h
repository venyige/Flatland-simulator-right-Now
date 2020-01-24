#ifndef GUARD_AGENT_H
#define GUARD_AGENT_H
#include<interface.h>
#include<iostream>
#include<string>
#include<fstream>



namespace now{
using namespace std;
class guard_agent
{
public:
    guard_agent(const char rowC, const char colC, const interface_d& intFac, ifstream& outLog);

private:
    string guardMsg;
    ifstream& _outLog;
    const char _rowC;
    const char _colC;
    const interface_d& _intFac;
};
}
#endif // GUARD_AGENT_H
