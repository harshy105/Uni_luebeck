#ifndef DAUGHTER_H
#define DAUGHTER_H

#include "Mother.h"

class Daughter: public Mother
{
    public:
        Daughter();
        ~Daughter();
        void doSomething();
};

#endif // DAUGHTER_H
