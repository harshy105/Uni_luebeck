#include <iostream>

using namespace std;

class Enemy{
public:
    virtual void attack(){
        cout << "enemy attack" << endl;
    };
    virtual void attack2()=0; // pure virtual function
};

class Ninja: public Enemy{
public:
    void attack(){
        cout << "Ninja not pure attack \n";
    }
    void attack2(){
        cout << "Ninja pure attack \n";
    }
};

class Monster: public Enemy{
public:
    void attack2(){
        cout << "Monster pure attack" << endl;
    }

};

int main()
{
    Ninja n;
    Monster m;
    Enemy *enemy_n = &n;
    Enemy *enemy_m = &m;
    enemy_n->attack();
    enemy_m->attack();
    enemy_n->attack2();
    enemy_m->attack2();
}
