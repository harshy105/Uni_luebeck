#include "People.h"
#include <iostream>
using namespace std;

People::People(string n, Birthday birthObject)
: name(n), dateOfBirth(birthObject)
{}

void People::printInfo(){
    cout << name << " was born on ";
    dateOfBirth.printDate();
}
