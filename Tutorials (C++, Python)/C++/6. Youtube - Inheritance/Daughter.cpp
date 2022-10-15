#include <iostream>
#include "Daughter.h"
using namespace std;


Daughter::Daughter(){
    cout << "Daughter Cons" << endl;
}

Daughter::~Daughter(){
    cout << "Daughter Decons" << endl;
}


void Daughter::doSomething(){
    publicVar = 1;
    cout << "Daughter can access public variable : " << publicVar << endl;
    protectedVar = 2;
    cout << "Daughter can access protected variable : " << protectedVar << endl;
    // privateVar = 3; // this will throw an error as it can not be accessed by derived class
    cout << "Daughter can NOT access private variable" << endl;
}
