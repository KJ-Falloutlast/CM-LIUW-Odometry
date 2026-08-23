#ifndef _BASIC_FUNCTION_H_
#define _BASIC_FUNCTION_H_

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>
#include <boost/array.hpp>
#include <vector>
template<typename T>
inline std::string num2str(const T src){
    std::stringstream ss;
    ss << src;
    return ss.str();
}

// vector arithematic
template <typename T, const size_t N>
inline boost::array<T,N> vectorSubtraction(const boost::array<T,N> &a, const boost::array<T,N> &b){
    boost::array<T,N> dst;
    for(int i=0;i<a.size();++i){
        dst.at(i) = ( a.at(i)-b.at(i) );
    }
    return dst;
}
template <typename T, const size_t N>
inline boost::array<T,N> vectorAddition(const boost::array<T,N> &a, const boost::array<T,N> &b){
    boost::array<T,N> dst;
    for(int i=0;i<a.size();++i){
        dst.at(i) = ( a.at(i)+b.at(i) );
    }
    return dst;
}
template <typename T, size_t N>
inline void setZeroVector(boost::array<T,N> &a){
    for(int i=0;i<a.size();++i){
        a.at(i) = 0;
    }
}


template <typename T>
inline std::vector<T> vectorSubtraction(const std::vector<T> &a, const std::vector<T> &b){
    std::vector<T> dst;
    for(int i=0;i<a.size();++i){
        dst.push_back( a.at(i)-b.at(i) );
    }
    return dst;
}
template <typename T>
inline std::vector<T> vectorAddition(const std::vector<T> &a, const std::vector<T> &b){
    std::vector<T> dst;
    for(int i=0;i<a.size();++i){
        dst.push_back( a.at(i)+b.at(i) );
    }
    return dst;
}
template <typename T>
inline void setZeroVector(std::vector<T> &a){
    for(int i=0;i<a.size();++i){
        a.at(i) = 0;
    }
}



#endif