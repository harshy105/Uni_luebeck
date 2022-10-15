#ifndef PEOPLE_H
#define PEOPLE_H

#include <string>
#include "Birthday.h"
using namespace std;

class People
{
    public:
        People(string n, Birthday birthObject);
        void printInfo();

    private:
        string name;
        Birthday dateOfBirth;
};

#endif // PEOPLE_H
