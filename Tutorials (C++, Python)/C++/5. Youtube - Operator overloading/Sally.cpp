#include <iostream>
#include "Sally.h"
using namespace std;

Sally::Sally()
{
}

Sally::Sally(int a){
    num = a;
}

Sally Sally::operator+(Sally b_so){
    Sally c_so;
    c_so.num = num + b_so.num;
    return c_so;
}
