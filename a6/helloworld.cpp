#include <iostream>
#include "PrimeChecker.hpp"
// This function defines the main program
int main(int argc, char** argv) {
    if (argc == 2) {
        int number = std::stoi(argv[1]);
        PrimeChecker pc;
        std::cout << "Svahn, Ling; " << number << " is a prime number? " << pc.isPrime(number) << std::endl; 
    }
    return 0;
}
