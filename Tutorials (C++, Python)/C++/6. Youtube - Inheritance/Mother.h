#ifndef MOTHER_H
#define MOTHER_H


class Mother
{
    public:
        Mother();
        ~Mother();
        int publicVar;

    protected: // any dervied class has access to it, friends have access to it
        int protectedVar;

    private:
        int privateVar;  // friends have access to it
};

#endif // MOTHER_H
