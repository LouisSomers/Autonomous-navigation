#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>



template< typename T > inline std::string tostring( T t )
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}