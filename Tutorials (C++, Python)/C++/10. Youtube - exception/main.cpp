#include <iostream>
using namespace std;

int main(){
    try{
        float num1, num2;
        cout << "Enter the numerator:" << endl;
        cin >> num1;
        cout << "Enter the denominator:" << endl;
        cin >> num2;

        if(num2 == 0) {
            throw 7; // can be 7.2, 'd', etc.
        }

        cout << "ANSWER: " << num1/num2 << endl;

    }//catch(...){ // can accept any data type of throw
    //    cout << "Denominator can not be zero" << endl;
    //}
    catch(int x){ // can be catch(float x) etc.
        cout << "Denominator can not be zero, ERROR MESSAGE: " << x << endl;
    }
}
