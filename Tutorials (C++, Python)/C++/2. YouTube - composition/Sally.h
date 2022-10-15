#ifndef SALLY_H
#define SALLY_H

class Sally{
public:
    Sally(int a, int b);
    ~Sally();
    void print();
    int printInt(int var) const;
private:
    int regVar;
    const int constVar;
};

#endif // SALLY_H
