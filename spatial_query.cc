#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

// to store queries results
#include <vector>

// just for output
#include <boost/foreach.hpp>
#include <iostream>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

int main(int argc, char* argv[]) {
  typedef bg::model::point<float, 2, bg::cs::cartesian> point;
  typedef bg::model::box<point> box;
  typedef std::pair<box, unsigned> value;

  // create the rtree using default constructor
  bgi::rtree<value, bgi::quadratic<16>> rtree;

  // create some values
  for (unsigned i = 0; i < 10; ++i) {
    // create a box
    box b(point(i + 0.0f, i + 0.0f), point(i + 0.5f, i + 0.5f));
    // insert new value
    rtree.insert(std::make_pair(b, i));
  }

  // find values intersecting some area defined by a box
  box query_box(point(0, 0), point(2, 2));
  std::vector<value> result_s;
  rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));

  // display results
  std::cout << "spatial query box:" << std::endl;
  std::cout << bg::wkt<box>(query_box) << std::endl;
  std::cout << "spatial query result:" << std::endl;
  BOOST_FOREACH (value const& v, result_s)
    std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

  std::cout << "knn query point:" << std::endl;
  std::cout << bg::wkt<point>(point(0, 0)) << std::endl;
  std::cout << "knn query result:" << std::endl;
  BOOST_FOREACH (value const& v, result_s)
    std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

  return 0;
}
