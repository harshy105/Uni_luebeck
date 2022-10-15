#include <iostream>
using namespace std;

template <class T>
class Harsh{
public:
    Harsh(T a);
};

template <class T> // making a  function outside the header
Harsh<T>::Harsh(T a){
    cout << a << " is not a character" << endl;
}

template<> // template specialization
class Harsh<char>{
public:
    Harsh(char a){
    cout << a << " is indeed a character" << endl;
    }
};

int main(){
    Harsh <int> obj1(10);
    Harsh <float> obj2(21.1);
    Harsh <char> obj3('y');
}
