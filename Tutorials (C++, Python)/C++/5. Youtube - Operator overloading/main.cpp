#include <iostream>
#include "Sally.h"
using namespace std;

int main()
{
    Sally ao(12);
    Sally bo(14);
    Sally co;

    co = ao+bo;
    cout << co.num << endl;
}
