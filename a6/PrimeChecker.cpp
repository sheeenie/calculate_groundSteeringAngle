#include "PrimeChecker.hpp"
// function to decide if the number is a prime number or not
// if n is less than 2 or even , then it's not prime
// for odd numbers that are >=3, only odd divisors are checked
bool PrimeChecker::isPrime(uint16_t n) {
    bool retVal{true};
    if (n<2 || 0 == n%2) {
        retVal = false;
    }
    else {
        for(uint16_t i{3}; (i*i) <= n; i += 2) {
            if (0 == n%i) {
                return false;
                break;
            }
        }
    }
    return retVal;
}
