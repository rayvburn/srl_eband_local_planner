#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <set>

// geometry_msg
#include <geometry_msgs/PoseStamped.h>


typedef boost::array<double, 2> Array2D;

typedef boost::array<double, 4> Array4D;

typedef boost::array<int, 2> Array2I;

namespace check_points_on_path{

  class CheckPointsOnPath{

  public:

      CheckPointsOnPath();

      ~CheckPointsOnPath();

      Array4D intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4);

      double edist(Array4D v1, Array4D v2);

      std::pair<double, bool> distance2Segment(Array4D x, Array4D xs,
                                        Array4D xe,  Array4D &proj, double &dout);

      bool checkLaserPointInside(double x, double y);

      bool setPathDistance(double p);

      bool setPath(std::vector<geometry_msgs::PoseStamped> local_plan);

      double path_distance_;

      Eigen::MatrixXd *path_;

  };

}
