#ifndef DESIGN_RATIONALE_H_
#define DESIGN_RATIONALE_H_

#include <cmath>

#include "boost/mpl/int.hpp"

struct mypoint {
  double x, y;
};

double distance(mypoint const& a, mypoint const& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

template <typename P1, typename P2>
double distance(P1 const& a, P2 const& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

template <typename P1, typename P2>
double distance(P1 const& a, P2 const& b) {
  double dx = get<0>(a) - get<0>(b);
  double dy = get<1>(a) - get<1>(b);
  return std::sqrt(dx * dx + dy * dy);
}

namespace traits {

template <typename P, int D>
struct access {};

template <>
struct access<mypoint, 0> {
  static double get(mypoint const& p) { return p.x; }
};

template <>
struct access<mypoint, 1> {
  static double get(mypoint const& p) { return p.y; }
};

template <typename P>
struct dimension {};

template <>
struct dimension<mypoint> : boost::mpl::int_<2> {};

template <typename P>
struct coordinate_type {};

// specialization for our mypoint
template <>
struct coordinate_type<mypoint> {
  typedef double type;
};

template <typename G>
struct tag {};

// specialization
template <>
struct tag<mypoint> {
  typedef point_tag type;
};

}  // namespace traits

namespace dispatch {
template <typename Tag1, typename Tag2, typename G1, typename G2>
struct distance {};

template <typename P1, typename P2>
struct distance<point_tag, point_tag, P1, P2> {
  static double apply(P1 const& a, P2 const& b) {
    // here we call pythagoras
    // exactly like we did before
    ...
  }
};

template <typename P, typename S>
struct distance<point_tag, segment_tag, P, S> {
  static double apply(P const& p, S const& s) {
    // here we refer to another function
    // implementing point-segment
    // calculations in 2 or 3
    // dimensions...
    ...
  }
};

// here we might have many more
// specializations,
// for point-polygon, box-circle, etc.

}  // namespace dispatch

template <int D, typename P>
inline double get(P const& p) {
  return traits::access<P, D>::get(p);
}

template <typename P>
struct dimension : traits::dimension<P> {};

template <typename P>
struct coordinate_type : traits::coordinate_type<P> {};

template <typename P1, typename P2, int D>
struct pythagoras {
  static double apply(P1 const& a, P2 const& b) {
    double d = get<D - 1>(a) - get<D - 1>(b);
    return d * d + pythagoras<P1, P2, D - 1>::apply(a, b);
  }

  typedef typename select_most_precise<typename coordinate_type<P1>::type,
                                       typename coordinate_type<P2>::type>::type
      computation_type;

  static computation_type apply(P1 const& a, P2 const& b) {
    computation_type d = get<D - 1>(a) - get<D - 1>(b);
    return d * d + pythagoras<P1, P2, D - 1>::apply(a, b);
  }
};

template <typename P1, typename P2>
struct pythagoras<P1, P2, 0> {
  static double apply(P1 const&, P2 const&) { return 0; }
};

template <typename P1, typename P2>
double distance(P1 const& a, P2 const& b) {
  BOOST_STATIC_ASSERT((dimension<P1>::value == dimension<P2>::value));

  return std::sqrt(pythagoras<P1, P2, dimension<P1>::value>::apply(a, b));
}

template <typename P, typename S>
double distance_point_segment(P const& p, S const& s) {}

template <typename G1, typename G2>
double distance(G1 const& g1, G2 const& g2) {
  return dispatch::distance<typename tag<G1>::type, typename tag<G2>::type, G1,
                            G2>::apply(g1, g2);
}

template <typename G>
struct tag : traits::tag<G> {};

template <typename G>
struct coordinate_type {
  typedef
      typename dispatch::coordinate_type<typename tag<G>::type, G>::type type;
};

struct cartesian {};

template <typename DegreeOrRadian>
struct geographic {
  typedef DegreeOrRadian units;
};

template <typename G1, typename G2>
double distance(G1 const& g1, G2 const& g2) {
  typedef typename strategy_distance<
      typename coordinate_system<G1>::type,
      typename coordinate_system<G2>::type, typename point_type<G1>::type,
      typename point_type<G2>::type, dimension<G1>::value>::type strategy;

  return dispatch::distance<typename tag<G1>::type, typename tag<G2>::type, G1,
                            G2, strategy>::apply(g1, g2, strategy());
}

template <typename T1, typename T2, typename P1, typename P2, int D>
struct strategy_distance {
  typedef void type;
};

template <typename P1, typename P2, int D>
struct strategy_distance<cartesian, cartesian, P1, P2, D> {
  typedef pythagoras<P1, P2, D> type;
};

template <typename P1, typename P2, int D = 2>
struct strategy_distance<spherical, spherical, P1, P2, D> {
  typedef haversine<P1, P2> type;
};

template <typename G1, typename G2, typename S>
double distance(G1 const& g1, G2 const& g2, S const& strategy) {
  return dispatch::distance<typename tag<G1>::type, typename tag<G2>::type, G1,
                            G2, S>::apply(g1, g2, strategy);
}

template <typename T = double>
struct cartesian_distance {
  T sq;
  explicit cartesian_distance(T const& v) : sq(v) {}

  inline operator T() const { return std::sqrt(sq); }
};

template <typename G1, typename G2 = G1>
struct distance_result {
  typedef typename point_type<G1>::type P1;
  typedef typename point_type<G2>::type P2;
  typedef typename strategy_distance<typename cs_tag<P1>::type,
                                     typename cs_tag<P2>::type, P1, P2>::type S;

  typedef typename S::return_type type;
};

// struct haversine with apply function
// is omitted here

#endif
