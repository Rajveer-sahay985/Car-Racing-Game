#include <iostream>
using namespace std;

int main() {

    int a = 10;
    int b = 20;

    int c = 0;

    c = a;
    a = b;
    b = c;

    cout << a << "\n" << b << "\n";
    return 0;
}
