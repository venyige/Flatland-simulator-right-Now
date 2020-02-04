#ifndef DIRECTIONS_H
#define DIRECTIONS_H
#include<map>
#include<bitset>
#include<vector>
#include<cmath>

using namespace std;
/** d_t direction type, a smart byte to store and process direction data*/
class d_t{
public:
    d_t():_data(0){}
    d_t(int s){*this=s;}
//    d_t(char s){*this=char(s);}
    typedef enum {
        none=0u,
        u_l=1u<<0,
        d_r=1u<<1
    }dir_t;
    enum dir{u,ul,l,dl,d,dr,r,ur};

    static const map<array<dir_t,2>, dir> dirs;
    static const map<dir, array<dir_t,2>> dirsR;

    static const map<array<d_t::dir_t,2>, d_t::dir> returnDirMap(){
        return {{{u_l,none}   ,u  },
            {{u_l, u_l}   ,ul },
            {{none, u_l}  ,l  },
            {{d_r, u_l}   ,dl },
            {{d_r,none}   ,d  },
            {{d_r,d_r}    ,dr },
            {{none,d_r}   ,r  },
            {{u_l, d_r}   ,ur }};
    }

    static const map<d_t::dir, array<d_t::dir_t,2>> returnRevMap(){
        return {{ u  ,{u_l,none}  },
            { ul ,{u_l, u_l}  },
            { l  ,{none, u_l} },
            { dl ,{d_r, u_l}  },
            { d  ,{d_r,none}  },
            { dr ,{d_r,d_r}   },
            { r  ,{none,d_r}  },
            { ur ,{u_l, d_r}  }};
    }

    int operator +(int a){
        return (int(*this)+a)%8;
    }
    int operator -(int a){
        int retVal=(int(*this)-a)%8;
        if(retVal<0)
            retVal=(8+retVal);

        return retVal;
    }
    d_t& operator +=(int a){
       *this=(int(*this)+a)%8;
        return *this;
    }
    d_t& operator -=(int a){
       *this=(d_t(*this)-a);
        return *this;
    }
    d_t& operator ++(){
       *this=(int(*this)+1)%8;
        return *this;
    }
    d_t operator++(int)
    {
        d_t tmp(*this);
        operator++();
        return tmp;
    }
    d_t& operator --(){
       *this=(d_t(*this)-1);
        return *this;
    }
    d_t operator--(int)
    {
        d_t tmp(*this);
        operator--();
        return tmp;
    }
    d_t& operator =(const array<uint8_t,2>& a){
        _data=setArr(a);
        return *this;
    }
    d_t& operator =(const array<dir_t,2>& a){
        _data=setArr(a);
        return *this;
    }
    d_t& operator =(int iii){
        iii%=8;
        if(iii<0)
            iii=(8+iii);
        const map<dir, array<dir_t,2>>::const_iterator mii=dirsR.find(dir(iii));
        _data=setArr((*mii).second);
              return *this;
    }
    d_t& operator =(dir& dii){
        const map<dir, array<dir_t,2>>::const_iterator mii=dirsR.find(dii);
        _data=setArr((*mii).second);
              return *this;
    }
    d_t& operator =(char chr){
        switch (chr) {
        case '<':
            *this=2;
            break;
        case 'v':
            *this=4;
            break;
        case '>':
            *this=6;
            break;
        case '^':
        default:
            *this=0;
            break;

        }
        return *this;
    }
    operator char() const{
        char retVal;
        switch (int(*this)) {
        case 2:
            retVal='<';
            break;
        case 4:
            retVal='v';
            break;
        case 6:
            retVal='>';
            break;
        case 0:
        default:
            retVal='^';
            break;

        }
        return retVal;
    }
    bool operator ==(int a){
        return int(*this)==a;
    }
    operator int() const{

        const map<array<dir_t,2>,dir>::const_iterator mii=dirs.find(getArr());
        return static_cast<int>(mii->second);
    }
    operator array<bitset<2>, 2>() const{
        return array<bitset<2>, 2>{bitset<2>((_data>>2)&3), bitset<2>((_data)&3)};
    }
    operator vector<bitset<2>>() const{
        vector<bitset<2>> retVal{bitset<2>((_data>>2)&3), bitset<2>((_data)&3)};
        return retVal;
    }
    uint8_t& get(){return _data;}
    bool hasUp(){return (_data>>2)&u_l;}
    bool hasDown(){return (_data>>2)&d_r;}
    bool hasLeft(){return _data&u_l;}
    bool hasRight(){return _data&d_r;}
private:
    uint8_t setArr(const array<uint8_t,2>& a){
        return((a[0]&3)<<2)+(a[1]&3);}
    uint8_t setArr(const array<dir_t,2>& a){
        return((a[0]&3)<<2)+(a[1]&3);}
    const array<dir_t,2>& getArr()const{
        array<dir_t,2>* retVal=new array<dir_t,2>{static_cast<dir_t>((_data>>2)&3), static_cast<dir_t>(_data&3)};
        return *retVal;}
    uint8_t _data;
};

#endif // DIRECTIONS_H
