#include <iostream>
#include "Sally.h"
using namespace std;

int main()
{
    Sally sallyObj(3,5);
    sallyObj.print();
    sallyObj.printInt(13);

    const Sally constObj(6,7);
    constObj.printInt(17);
    // constObj.print(); // can not use since const object can only use the const functions

    Sally *sallyPointer = &sallyObj; // don't need to give the inital address
    sallyPointer->print();
    sallyPointer->printInt(16);

}
