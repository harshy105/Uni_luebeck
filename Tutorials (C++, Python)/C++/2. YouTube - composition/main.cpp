#include <iostream>
#include "People.h"
#include "Birthday.h"
using namespace std;

int main()
{
    Birthday birthObj(04, 11, 1995);

    People harsh("Harsh", birthObj);
    harsh.printInfo();
}
