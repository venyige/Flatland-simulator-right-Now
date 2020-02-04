#include "include/directions.h"

const map<array<d_t::dir_t,2>, d_t::dir> d_t::dirs=d_t::returnDirMap();
const map<d_t::dir, array<d_t::dir_t,2>> d_t::dirsR=d_t::returnRevMap();
