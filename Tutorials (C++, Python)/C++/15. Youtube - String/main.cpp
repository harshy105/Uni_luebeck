#include <iostream>
#include <string>
using namespace std;

int main(){
    string s1("Hi my name is Harsh and I am awesome today!");
    cout << s1 << endl;
    s1.erase(20); // erase after 20 char
    cout << s1 << endl;
    s1.replace(14, 5, "Abhi");
    cout << s1 << endl;
    s1.insert(14, "lucky ");
    cout << s1 << endl;
}

void function1(){
    string x;
    getline(cin, x);
    cout << x << endl;

    string y;
    cin >> y;
    cout << y << endl;

    cout << y.at(3) << endl;
    cout << y.length() << endl;
}

void function2(){
    string one("apple");
    string two("beans");

    one.swap(two);
    cout << one << ' ' << two << endl;

    cout << one.substr(2,3) << endl; // starting from 2 and have 3 char

    string s1("ham is spam oh yes I am!");
    cout << s1.find("am") << endl; // gives the first occurrence of am
    cout << s1.rfind("am") << endl; // gives the last occurrence of am
}
