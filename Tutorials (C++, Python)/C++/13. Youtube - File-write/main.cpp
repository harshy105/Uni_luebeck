#include <iostream>
#include <fstream>
using namespace std;

int main(){
    ofstream writeFile("players.txt");

    cout << "Enter the Player ID, Name and Power\n";
    cout << "Press ctrl+z to exit" << endl;

    int id;
    string name;
    double power;

    while(cin >> id >> name >> power){
        writeFile << id << ' ' << name << ' ' << power << endl;
    }
    writeFile.close();
}
