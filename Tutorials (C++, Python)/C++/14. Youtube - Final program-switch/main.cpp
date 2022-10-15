#include <iostream>
#include <fstream>
using namespace std;

int getWhatTheyWant();
void displayItems(int x);

// main function
int main()
{
    int whatTheyWant;

    whatTheyWant = getWhatTheyWant();

    while(whatTheyWant != 4){
        switch(whatTheyWant){
        case 1:
            displayItems(1);
            break;
        case 2:
            displayItems(2);
            break;
        case 3:
            displayItems(3);
            break;
        }

        whatTheyWant = getWhatTheyWant();
    }

}

// getWhatTheyWant function
int getWhatTheyWant(){
    int choice;
    cout << endl;
    cout << "1 - plain objects" << endl;
    cout << "2 - useful objects" << endl;
    cout << "3 - harmful objects" << endl;
    cout << "4 - end program" << endl;
    cout << endl;

    cin >> choice;

    return choice;
}


// displayItems function
void displayItems(int x){
    ifstream objectFile("objects.txt");
    string name;
    double power;

    if(x == 1){
        while(objectFile >> name >> power){
            if(power==0){
                cout << name << ' ' << power << endl;
            }
        }
    }
    if(x == 2){
        while(objectFile >> name >> power){
            if(power>0){
                cout << name << ' ' << power << endl;
            }
        }
    }
    if(x == 3){
        while(objectFile >> name >> power){
            if(power<0){
                cout << name << ' ' << power << endl;
            }
        }
    }
}
