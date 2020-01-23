#include "driver.h"

driver::driver(shared_ptr<array<array<char, 80>, 25>>& sceneMat,
               mutex& sceneMut,
               mutex& kbdMut,
               volatile const char& kbStat,
               volatile const bool& quitter,
               size_t rowC, size_t colC,
               size_t carX, size_t carY):
    _rowC(rowC),_colC(colC),
    _carX(carX), _carY(carY),
    _kbStat(kbStat),
    _quitter(quitter),
    _sceneMat(sceneMat),
    _sceneMutex(sceneMut),
    _kbdMutex(kbdMut)
{
    forwardOn=-1.0;
    backwardOn=-1.0;
    leftOn=-1.0;
    rightOn=-1.0;

    forwardProcessed=false;
    backwardProcessed=false;
    leftProcessed=false;
    rightProcessed=false;
}
void driver::physics(){

    while(!_quitter){
        while(_kbdMutex.try_lock());
        if(kbStatBuf!=_kbStat){


            kbStatBuf=_kbStat;
            _kbdMutex.unlock();
        }else {
            _kbdMutex.unlock();


        }


    }

}
