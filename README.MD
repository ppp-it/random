# GENERATION OF RANDOM NUMBERS

one instance of the class is able to generate integer and real random numbers;  
generates numbers uniformly distributed in the range;  
when generating, you can change the range of numbers generated.  
  
  
Example of use.
```
#include <iostream>
#include <thread>
#include "random_t.h"



int main()
{
    static thread_local CRandom_t<std::minstd_rand0> random_t;
//    /*static thread_local*/ CRandom_t<std::minstd_rand> random_t;
//    /*static thread_local*/ CRandom_t<std::mt19937> random_t;
//    /*static thread_local*/ CRandom_t<std::mt19937_64> random_t;
//    /*static thread_local*/ CRandom_t<std::ranlux24_base> random_t;
//    /*static thread_local*/ CRandom_t<std::ranlux48_base> random_t;
//    /*static thread_local*/ CRandom_t<std::knuth_b> random_t;
//    /*static thread_local*/ CRandom_t<std::default_random_engine> random_t;

    float               fres = 0;
    double              dres = 0;
    int                 ires = 0;
    unsigned int        uires = 0;
    unsigned long       ulres = 0;
    long long           llres = 0;
    unsigned long long  ullres = 0;

    for ( int x = 0 ; x < 10 ; ++x ) {
        for ( int y = 0 ; y < 1 ; ++y ) {
            fres = random_t.rand<float>( 0, 1000 );
            std::cout << "fres = " << fres << std::endl;
            dres = random_t.rand<double>( 0, 1000 );
            std::cout << "dres = " << dres << std::endl;
            ires = random_t.rand<int>( 0, 1000 );
            std::cout << "ires = " << ires << std::endl;
            uires = random_t.rand<unsigned int>( 0, 1000 );
            std::cout << "uires = " << uires << std::endl;
            ulres = random_t.rand<unsigned long>( 0, 1000 );
            std::cout << "ulres = " << ulres << std::endl;
            llres = random_t.rand<long long>( 0, 1000 );
            std::cout << "llres = " << llres << std::endl;
            ullres = random_t.rand<unsigned long long>( 0, 1000 );
            std::cout << "ullres = " << ullres << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}

 ```  
