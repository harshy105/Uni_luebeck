#include <iostream>
//#include <cmath>
//#include <cstdlib>
//#include <string>
//#include "header.h"
#include <fstream>
using namespace std;

int main(){
    ofstream harshsFile("tuna.txt");
    // harshsFile.open("tuna.txt");

    if(harshsFile.is_open()){
        harshsFile << "I love tuna\n";
        harshsFile.close();
    }else{
        cout << "File is not open" << endl;
    }
}
