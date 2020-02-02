#include<iostream>
#include<keyboard.h>
#include<gui_curses.h>
#include<driver.h>
#include<thread>
#include<chrono>
#include<fstream>
#include<mutex>
#include<guard_agent.h>
using namespace now;
/** \brief <h3>Welcome screen</h3>
 * Puts out info about the only parameter, the scene descriptor file
 * and the scene file structure and evaluation.*/
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



/** <h3>main function</h3>
 * accepts one argument, the scene descriptor
 * file name with full path.*/
int main(int argc, char** argv)
{
    /**
* <h3> Parameters:</h3>
*  <b><i>Note: All local constants. Let's say "compile time parameters"</i></b><br>
* \b kbTestMode To test the keyboard in "joystick" mode.<br>
* \b scRowCnt Row count<br>
* \b scColCnt Column count<br>
* \b maxLinSpeed Maximum linear speed<br>
* \b maxAngSpeed Maximum angular speed<br>
* \b maxAcceleration Maximum linear/angular acceleration
\snippet this compile_time_pars*/
    //[compile_time_pars]
    const bool kbTestMode=false;
    const size_t scRowCnt=25;
    const size_t scColCnt=80;
    const double maxLinSpeed=3.0;
    const double maxAngSpeed=1.0;
    const double maxAcceleration=1.0;
    //[compile_time_pars]
    string sceneLine;
    string driverMsg;
    size_t charX, charY;
    /** \brief Scene matrix definition and initialization: \snippet this 11*/
    //[11]
    shared_ptr<array<array<char, scColCnt>, scRowCnt>> sceneMat=std::make_shared<array<array<char, scColCnt>, scRowCnt>>();//< Scene matrix definition and initialization
    //[11]
    int retVal=0;
    if(argc>1){
        ifstream sceneF(string(argv[1]).c_str(), std::ifstream::in);
        if(sceneF.good()){
            bool getCarPos=false;
            for(auto& iii: (*sceneMat)){
                iii.fill(' ');//< fill with spaces (ascii 32)
            }
            /** \brief Processing and storing scene data: \snippet this 2*/
            //[2]
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
            //[2]
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
        /** Bitwise keyboard indicator  - lsb:w,W, lsb+1:a,A lsb+2:s,S lsb+3:d,D \snippet this 3*/
        //[3]
        volatile char kbState;
        //[3]
        volatile bool kbQuitter=false;//< Quitter switch for thread loops
        mutex sceneMutex, kbdMutex;
        unique_ptr<keyboard> kbInst=make_unique<keyboard>(kbQuitter, kbdMutex);
        /** Start keyboard::state() in a separate thread.  \snippet this 5*/
        //[5]
        thread kbTr=kbInst->stateThread(kbState);
        //[5]
        ofstream logFile;
        unique_ptr<gui_curses<scRowCnt, scColCnt>> guiC=make_unique<gui_curses<scRowCnt, scColCnt>>(kbState, sceneMutex, sceneMat, kbQuitter);
        if(kbTestMode){
            guiC->test();
            kbQuitter=true;
        }else{

            unique_ptr<driver<scRowCnt, scColCnt>> driverInst=make_unique<driver<scRowCnt, scColCnt>>(sceneMat,
                                                                                                      sceneMutex,
                                                                                                      kbdMutex,
                                                                                                      kbState,
                                                                                                      kbQuitter,
                                                                                                      maxLinSpeed,
                                                                                                      maxAngSpeed,
                                                                                                      maxAcceleration,
                                                                                                      driverMsg);
            /** Start driver::physics() in a separate thread.  \snippet this 4*/
            //[4]
            thread drvTr=driverInst->driverThread();
            //[4]

            logFile.open("/tmp/car_log.txt", ios::out);
            unique_ptr<guard_agent> guardInst=make_unique<guard_agent>(scRowCnt,
                                                                       scColCnt,
                                                                       maxLinSpeed,
                                                                       maxAngSpeed,
                                                                       maxAcceleration,
                                                                        driverInst->getInterface(),
                                                                       logFile
                                                                       );
            thread gaTr=guardInst->guardThread(kbQuitter);
            guiC->disp();//< gui::disp works in the main thread.
            /** Whith this easy step we stopped the main loop of the keyboard and driver thread \snippet this 6*/
//[6]
            kbQuitter=true;
//[6]
            gaTr.join();
            guardInst.reset();
            drvTr.join();
            driverInst.reset();
            logFile.close();
        }


        kbTr.join();

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
