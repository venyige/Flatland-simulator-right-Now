#ifndef GUARD_AGENT_CPP
#define GUARD_AGENT_CPP
#include "guard_agent.h"
namespace now{
guard_agent::guard_agent(const char rowC, const char colC, const interface_d& intFac, ifstream& outLog):
    _outLog(outLog),
    _rowC(rowC),
    _colC(colC),
    _intFac(intFac){}

}
#endif // GUARD_AGENT_CPP
