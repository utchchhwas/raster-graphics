#ifndef POINT_HPP
#define POINT_HPP

#include <iostream>
#include <iomanip>
#include "Vec.hpp"
#include "Matrix.hpp"

template <typename T>
class Point {
public:
    T x, y, z;

    Point() : x(0), y(0), z(0) {}

    Point(T xVal, T yVal, T zVal) : x(xVal), y(yVal), z(zVal) {}

    Point(const Vec<T, 3>& vec) : x(vec[0]), y(vec[1]), z(vec[2]) {}

    Point operator+(const Vec<T, 3>& vec) const {
        return Point(x + vec[0], y + vec[1], z + vec[2]);
    }

    Point operator-(const Vec<T, 3>& vec) const {
        return Point(x - vec[0], y - vec[1], z - vec[2]);
    }

    Vec<T, 3> operator-(const Point& other) const {
        return Vec<T, 3>{x - other.x, y - other.y, z - other.z};
    }

    Vec<T, 3> toVector() const {
        return Vec<T, 3>(x, y, z);
    }

    Matrix<T, 4, 1> toMatrix() const{
        Matrix<T, 4, 1> matrix;
        matrix[0][0] = x;
        matrix[1][0] = y;
        matrix[2][0] = z;
        matrix[3][0] = 1;
        return matrix;
    }
};

// Input operator for Point
template <typename T>
std::istream& operator>>(std::istream& is, Point<T>& point) {
    is >> point.x >> point.y >> point.z;
    return is;
}

// Output operator for Point
template <typename T>
std::ostream& operator<<(std::ostream& os, const Point<T>& point) {
    os << std::fixed << std::setprecision(7) << point.x << " " << point.y << " " << point.z;
    return os;
}

#endif // POINT_HPP
