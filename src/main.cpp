#include<iostream>
#include<keyboard.h>
#include<gui_curses.h>
#include<driver.h>
#include<thread>
#include<chrono>
#include<fstream>
#include<mutex>
using namespace now;

void help(){
    cout<< "The program must be started with full path to the scene file as first argument:"<<endl;
    cout<< "./fsrn /full/path/to/scene/file"<<endl;
    cout<< "The file is to be read line by line, one line in file corresponds a line of"<<endl;
    cout<< "the 80X25 screen."<<endl;
    cout<< "A minimal scene file is a text file containing a sole \"x\" or \"X\" character."<<endl;
    cout<< "This minimal scene file is being considered as empty 80X25 scene, with car"<<endl;
    cout<< "position at the upper-left corner."<<endl;
    cout<< "The empty grid points can be denoted by spaces, or for better readability by dots"<<endl;
    cout<< "\".\" or plus signs \"+\". Any character other than \"x\", \"X\", \"<\", \"^\", \">\",  \"v\",  \"#\" or"<<endl;
    cout<< "\"~\" being evaluated as empty space. Empty line being considered as line full of spaces."<<endl;
    cout<< "Lines shorter than 80, being padded with spaces. Missing lines of the full set of 25"<<endl;
    cout<< "being considered as empty lines."<<endl;
    cout<< "Lines longer than 80 being truncated."<<endl;
    cout<< "In the 80X25 range the file must contain one and only one \"x\" or \"X\" character"<<endl;
    cout<< "denoting the car position."<<endl;
    cout<< "If car position is missing, the program quits with error message: \"The car position"<<endl;
    cout<< "is missing from scene file\"."<<endl;
    cout<< "In the 80X25 range of the scene file only one \"x\" or \"X\" allowed, if more than one"<<endl;
    cout<< "found, the program quits with error message: \"Only one car position is allowed\"."<<endl;
}

/*** main function accepts one argument, the scene descriptor
 * file name with full path.
 * */
int main(int argc, char** argv)
{
    const bool kbTestMode=false;
    const size_t scRowCnt=25;
    const size_t scColCnt=80;
    const double maxLinSpeed=3.0;
    const double maxAngSpeed=1.0;
    const double maxAcceleration=1.0;
    string sceneLine;
    string driverMsg;
    size_t charX, charY;
    shared_ptr<array<array<char, scColCnt>, scRowCnt>> sceneMat=std::make_shared<array<array<char, scColCnt>, scRowCnt>>();///< Scene matrix definition and initialization
    int retVal=0;
    if(argc>1){
        ifstream sceneF(string(argv[1]).c_str(), std::ifstream::in);
        if(sceneF.good()){
            bool getCarPos=false;
            for(auto& iii: (*sceneMat)){
                iii.fill(' ');///< fill with spaces (ascii 32)
            }
            for(size_t iii=0; iii<scRowCnt&&!retVal; iii++){
                if(getline(sceneF,sceneLine)){
                    for(size_t jjj=0; jjj<sceneLine.size()&&jjj<scColCnt&&!retVal;jjj++){
                        switch (sceneLine[jjj]) {
                        case 'x':
                        case 'X':
                            if(getCarPos){
                                help();
                                cout<<"Only one car position is allowed"<<endl;
                                retVal=1;
                            }else {
                                getCarPos=true;
                                (*sceneMat)[iii][jjj]='O';
                                charX=jjj;
                                charY=iii;
                            }
                            break;
                        case '<':
                        case '^':
                        case '>':
                        case 'v':
                        case '#':
                        case '~':
                            (*sceneMat)[iii][jjj]=sceneLine[jjj];
                            break;
                        default:
                            break;
                        }
                    }
                }else {
                    break;
                }
            }
            if(!getCarPos){
                help();
                cout<<"The car position is missing from scene file"<<endl;
                retVal=1;
            }
        }else{
            help();
            cout<<"File can not be opened for text reading"<<endl;
            retVal=1;
        }

    }else{
        help();
        cout<<"Missing scene file"<<endl;
        retVal=1;
    }
    if(retVal){
        sceneMat.reset();
        return 1;
    }else {

        volatile char kbState;///< Keyboard indicator - lsb:w,W, lsb+1:a,A lsb+2:s,S lsb+3:d,D
        volatile bool kbQuitter=false;///< Quitter switch for keyboard listener
        mutex sceneMutex, kbdMutex; ///< Scene mutex to avoid duplicated appearance of car and moving blocks

        unique_ptr<driver<scRowCnt, scColCnt>> driverInst=make_unique<driver<scRowCnt, scColCnt>>(sceneMat,
                                                                                                  sceneMutex,
                                                                                                  kbdMutex,
                                                                                                  kbState,
                                                                                                  kbQuitter,
                                                                                                  maxLinSpeed,
                                                                                                  maxAngSpeed,
                                                                                                  maxAcceleration,
                                                                                                  driverMsg);
        thread drvTr=driverInst->driverThread();///< Start driver::physics() in a separate thread. Calling driver::driverThread() does the work.

        unique_ptr<keyboard> kbInst=make_unique<keyboard>(kbQuitter, kbdMutex);
        thread kbTr=kbInst->stateThread(kbState);///< Start keyboard::state() in a separate thread. Calling keyboard::stateThread() does the work.

        unique_ptr<gui_curses<scRowCnt, scColCnt>> guiC=make_unique<gui_curses<scRowCnt, scColCnt>>(kbState, sceneMutex, sceneMat, kbQuitter);
        if(kbTestMode){
            guiC->test();
        }else{
            guiC->disp();///< gui::disp works in the main thread.
        }
        kbQuitter=true;///< Whith this easy step we stopped the main loop of the keyboard and driver thread
        drvTr.join();
        kbTr.join();
        driverInst.reset();
        sceneMat.reset();
        kbInst.reset(nullptr);
        guiC.reset(nullptr);
        if(driverMsg.length()>0){
            cout<<driverMsg<<endl;
            cout<<"Enter to close."<<endl;

        }else{
            cout<<"Normal end of program. Enter to close."<<endl;
        }
        getchar();
        return 0;
    }
}
