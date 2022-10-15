#include <iostream>
#include <string>
#include <fstream>
using namespace std;

int main()
{
    ifstream readFile("players.txt");

    int playerId;
    string name;
    float money;

    while(readFile >> playerId >> name >> money){
        cout << playerId << ',' << name << ',' << money << endl; // can use ' '
    }
    // no need to use readFile.close()
}
