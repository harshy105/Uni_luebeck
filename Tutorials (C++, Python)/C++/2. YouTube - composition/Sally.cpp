#include <iostream>
#include "Sally.h"
using namespace std;

Sally::Sally(int a, int b)
: regVar(a), constVar(b){} // member initializations for constVar

Sally::~Sally(){
    cout << "This is the destructor" << endl;
}

void Sally::print(){
cout << "Regular variable is: " << regVar << endl;
cout << "Constant variable is: "<< constVar << endl;
}


int Sally::printInt(int var) const{ // constant function initilization
    cout << "The integer is: " << var << endl;
}
