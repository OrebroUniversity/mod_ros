//
// Created by ksatyaki on 8/13/19.
//

#include <boost/chrono.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::d2::point_xy<double> BgPoint2D;
typedef bg::model::box<BgPoint2D> BgBox;

int main() {

  bgi::rtree<std::pair<BgPoint2D, int>, bgi::quadratic<16>> tree;
  std::array<BgPoint2D, 10> points = {
      BgPoint2D({2.0, 5.0}), BgPoint2D({2.5, 4.6}), BgPoint2D({2.4, 2.4}),
      BgPoint2D({2.8, 5.0}), BgPoint2D({2.2, 8.2}), BgPoint2D({2.7, 5.3}),
      BgPoint2D({2.3, 6.8}), BgPoint2D({2.6, 7.2}), BgPoint2D({2.2, 3.3}),
      BgPoint2D({2.1, 1.4})};

  for (auto i = 0; i < 10; i++)
    tree.insert(std::make_pair(points[i], i));

  boost::chrono::high_resolution_clock cl;

  std::vector<std::pair<BgPoint2D, int>> returned;
  BgPoint2D query_pt(3, 3);
  BgBox query_box(BgPoint2D(query_pt.get<0>() - 2, query_pt.get<1>() - 2),
                  BgPoint2D(query_pt.get<0>() + 2, query_pt.get<1>() + 2));

  printf("\nQuery box is: (%lf, %lf), (%lf, %lf)\n",
         query_box.max_corner().get<0>(), query_box.max_corner().get<1>(),
         query_box.min_corner().get<0>(), query_box.min_corner().get<1>());

  auto start = cl.now();
  tree.query(bgi::satisfies([&](std::pair<BgPoint2D, int> V) {
               return bg::distance(V.first, query_pt) < 2;
             }),
             std::back_inserter(returned));
  auto end = cl.now();

  for (const auto &val : returned) {
    std::cout << "Returned Values:" << std::endl;
    printf("(%lf, %lf),\n", val.first.get<0>(), val.first.get<1>());
  }

  std::cout << "Time: "
            << boost::chrono::duration_cast<boost::chrono::nanoseconds>(end -
                                                                        start)
                   .count()
            << std::endl;

  start = cl.now();

  returned.clear();
  tree.query(bgi::satisfies([&](std::pair<BgPoint2D, int> V) {
               return bg::within(V.first, query_box);
             }),
             std::back_inserter(returned));
  end = cl.now();

  for (const auto &val : returned) {
    std::cout << "Returned Values:" << std::endl;
    printf("(%lf, %lf),\n", val.first.get<0>(), val.first.get<1>());
  }

  std::cout << "Time (with ring): "
            << boost::chrono::duration_cast<boost::chrono::nanoseconds>(end -
                                                                        start)
                   .count()
            << std::endl;

  return 0;
}
