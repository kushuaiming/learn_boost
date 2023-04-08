#include <boost/geometry.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <iostream>

using namespace boost::geometry;

void UseOnlyPartOfTheLibrary() {
  model::d2::point_xy<int> p1(1, 1), p2(2, 2);
  std::cout << "Distance p1-p2 is: " << distance(p1, p2) << std::endl;
}

BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)
void RegisteringAndUsingCArray() {
  int a[2] = {1, 1};
  int b[2] = {2, 3};
  double d = distance(a, b);
  std::cout << "Distance a-b is: " << d << std::endl;
}

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)
void RegisteringAndUsingTuple() {
  double points[][2] = {
      {2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6}, {2.9, 0.7}, {2.0, 1.3}};
  model::polygon<model::d2::point_xy<double> > poly;
  append(poly, points);
  boost::tuple<double, double> p = boost::make_tuple(3.7, 2.0);
  std::cout << "Point p is in polygon? " << std::boolalpha << within(p, poly)
            << std::endl;
  std::cout << "Area: " << area(poly) << std::endl;
  int a[2] = {1, 1};
  double d2 = distance(a, p);
  std::cout << "Distance a-p is: " << d2 << std::endl;
}

int main(int argc, char* argv[]) {
  UseOnlyPartOfTheLibrary();
  RegisteringAndUsingCArray();
  RegisteringAndUsingTuple();
  return 0;
}
