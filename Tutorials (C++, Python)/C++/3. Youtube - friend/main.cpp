#include <iostream>
using namespace std;

class StankFist{
public:
    StankFist(){stinkyVar = 0;};
    int getVar(){return stinkyVar;}
private:
    int stinkyVar;
    friend void stinksFriend(StankFist sfo);
};

void stinksFriend(StankFist sfo){
    sfo.stinkyVar = 99; // Friend can access the private variable
    cout << sfo.stinkyVar << endl;
}

int main()
{
    StankFist bob;
    stinksFriend(bob);
    cout << "Original variable is now: " << bob.getVar() << endl;
}
