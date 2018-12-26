/*!
 * \file random_t.h
 * \brief генератор случайных чисел
 *
 *  один экземпляр класса способен генерить как целые\n
 *  так и вещественные случайные числа;\n
 *  создает числа равномерно распределенные по диапазону;\n
 *  при каждой генерации можно заново устанавливать предел\n
 *  диапазона от первого до последнего числа.\n
 *
 * Copyright (C) 2018 Pochepko PP.
 * Contact: ppp.it@hotmail.com
 *
 * This file is part of software written by Pochepko PP.
 *
 * This software is provided 'as-is', without any express or implied\n
 * warranty. In no event will the authors be held liable for any damages\n
 * arising from the use of this software.\n
 *
 * Permission is granted to anyone to use this software for any purpose,\n
 * including commercial applications, and to alter it and redistribute it\n
 * freely, subject to the following restrictions:\n
 *
 *    1. The origin of this software must not be misrepresented; you must not\n
 *    claim that you wrote the original software. If you use this software\n
 *    in a product, an acknowledgment in the product documentation would be\n
 *    appreciated but is not required.\n
 *
 *    2. Altered source versions must be plainly marked as such, and must not be\n
 *    misrepresented as being the original software.\n
 *    3. This notice may not be removed or altered from any source\n
 *    distribution.\n
 *
 */
//-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!-!


#ifndef RANDOM_T_H_PochepkoPP
#define RANDOM_T_H_PochepkoPP


#if defined(__GNUC__)
#include <chrono>       //  std::time (MINGW compiller)
#endif
#if defined(Q_CC_MSVC) || defined(_MSC_VER)
#include <ctime>        //  std::time   (MSVC compiller)
#endif
#include <random>       //  std::random_device, std::seed_seq
#include <type_traits>  //  std::enable_if< >
#include <limits>       //  std::numeric_limits< >



//--------------------------------------------------------------------------------------------------//


////////////////////////////////////// start of class CRandom_t //////////////////////////////////////

/// \brief The CRandom_t class - генератор случайных чисел
///
template<typename Engine>
class CRandom_t {

public:
    ///////////////////////////////////////////////////////////////////////////////
    /// \brief констуктор
    /// @note MINGW не всегда выдает случайность при повторном создании объекта,\n
    /// поэтому добавил текущее время, а в MS Visual Studio этого не наблюдалось
    CRandom_t() :
        rd{},
        seed{ static_cast<std::random_device::result_type>(std::time(NULL)),
              rd(), rd(), rd(), rd(), rd(), rd() },
        engine{ seed }
    {}
    ///////////////////////////////////////////////////////////////////////////////
    /// \brief дестуктор
    ~CRandom_t()
    {}

public:
    ///////////////////////////////////////////////////////////////////////////////
    /// \brief rand  - генерирует случайные числа, равномерно распределенные по\n
    ///                диапазону от first до last
    ///
    /// \param first - минимальное значение для дипапзона
    /// \param last  - максимальное значение для дипапзона
    ///
    /// \return      - случайное число в диапазоне от first до last
    ///
    template<class T>
    T rand( T first = std::numeric_limits<T>::min( ),
            T last  = std::numeric_limits<T>::max( ) )
    {
        return uniform_distribution( first, last );
    }

protected:
    ///////////////////////////////////////////////////////////////////////////////
    /// \brief rand  - генерирует случайное число для целочисленных типов\n
    ///                в диапазоне от first до last
    ///
    /// \param first - минимальное значение для дипапзона
    /// \param last  - максимальное значение для дипапзона
    ///
    /// \return      - случайное целое число в диапазоне от first до last
    ///
    template<class T>
    typename std::enable_if<std::is_integral<T>::value, T>::type
    uniform_distribution( T first = std::numeric_limits<T>::min( ),
                          T last  = std::numeric_limits<T>::max( ) )
    {
        if( first < last )
            return std::uniform_int_distribution<T>{ first, last }( engine );
        return std::uniform_int_distribution<T>{ last, first }( engine );
    }

protected:
    ///////////////////////////////////////////////////////////////////////////////
    /// \brief rand  - генерирует случайное число для вещественных типов\n
    ///                в диапазоне от first до last
    ///
    /// \param first - минимальное значение для дипапзона
    /// \param last  - максимальное значение для дипапзона
    ///
    /// \return      - случайное вещественное число в диапазоне от first до last
    ///
    template<class T>
    typename std::enable_if<std::is_floating_point<T>::value, T>::type
    uniform_distribution( T first = std::numeric_limits<T>::min( ),
                          T last  = std::numeric_limits<T>::max( ) )
    {
        if( first < last )
            return std::uniform_real_distribution<T>{ first, last }( engine );
        return std::uniform_real_distribution<T>{ last, first }( engine );
    }


//--------------------------------------------------------------------------------------------------//


private:
    ///////////////////////////////////////////////////////////////////////////////
    /// Seed with a real random value, if available
    std::random_device  rd;
private:
    ///////////////////////////////////////////////////////////////////////////////
    /// general-purpose bias-eliminating scrambled seed sequence generator
    std::seed_seq       seed;
private:
    ///////////////////////////////////////////////////////////////////////////////
    /// random number generators
    Engine              engine;
};
/////////////////////////////////////// end of class CRandom_t ///////////////////////////////////////

#endif // RANDOM_T_H_PochepkoPP
