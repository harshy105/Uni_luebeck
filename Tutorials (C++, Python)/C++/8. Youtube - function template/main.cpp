#include <iostream>
using namespace std;

template <class T, class U>

U larger(T x, U y){   // try the return type T
    return (x>y?x:y);
}

int main(){
    int a = 4;
    double b = 4.3;
    cout << larger(a,b) << endl;
}
