#include <srl_eband_local_planner/check_points_on_path.h>
#define INFINITO 1000000000.0


using namespace check_points_on_path;
using namespace std;

/// -----------------------------------------------------------
/// CheckPointsOnPath()
/// -----------------------------------------------------------
CheckPointsOnPath::CheckPointsOnPath(){

  path_distance_ = 0.750;
  path_ = NULL;
  path_= new Eigen::MatrixXd(0,2);
}


/// -----------------------------------------------------------
/// ~CheckPointsOnPath()
/// -----------------------------------------------------------
CheckPointsOnPath::~CheckPointsOnPath(){

  if( path_ != NULL ){
    delete path_;
  }

}



/// ---------------------------------------------------------------------------
/// intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4)
/// Computes the intersection point of two inifinite lines given by two
/// 2x1 support points x1,x2 and x3,x4. Returns Inf if lines are parallel
/// ---------------------------------------------------------------------------
Array4D  CheckPointsOnPath
::intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4){

  Array4D p = {{ INFINITO, INFINITO, INFINITO, INFINITO }};

  double denom = 0;
  denom = (x1[0]-x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]);

    if (fabs(denom) > 0.000000001){

        p[0] = ( (x1[0]*x2[1]-x1[1]*x2[0])*(x3[0]-x4[0])
                  - (x1[0]-x2[0])*(x3[0]*x4[1] - x3[1]*x4[0]) ) / denom;
        p[1] = ( (x1[0]*x2[1]-x1[1]*x2[0])*(x3[1]-x4[1])
                  - (x1[1]-x2[1])*(x3[0]*x4[1] - x3[1]*x4[0]) ) / denom;
        p[2] = 0;
        p[3] = 0;
    }

  return p;
}


/// ----------------------------------------------------------------------------
/// \brief Compute Euclidean Distance
/// ----------------------------------------------------------------------------
double  CheckPointsOnPath
::edist(Array4D v1, Array4D v2)
{
    return sqrt((v1[0]-v2[0])*(v1[0]-v2[0]) +
                              (v1[1]-v2[1])*(v1[1]-v2[1]));
}




/// ----------------------------------------------------------------------------
/// \brief Compute distance from a point to a line segment
/// also returns a flag indicating if the point is within
/// the line limits (perperdicularly)
/// ----------------------------------------------------------------------------
std::pair<double, bool> CheckPointsOnPath
::distance2Segment(Array4D x, Array4D xs, Array4D xe,  Array4D &proj,
    double &dout)
{
    double xa = xs[0]; double ya = xs[1];
    double xb = xe[0]; double yb = xe[1];
    double xp = x[0];  double yp = x[1];

    // % x-coordinates
    double A = xb-xa;
    double B = yb-ya;
    double C = yp*B+xp*A;
    double a =  2*((B*B)+(A*A));
    double b = -4*A*C+(2*yp+ya+yb)*A*B-(2*xp+xa+xb)*(B*B);
    double c =  2*(C*C)-(2*yp+ya+yb)*C*B+(yp*(ya+yb)+xp*(xa+xb))*(B*B);
    double x1 = (-b + sqrt((b*b)-4*a*c))/(2*a);
    double x2 = (-b - sqrt((b*b)-4*a*c))/(2*a);

    // % y-coordinates
    A = yb-ya;
    B = xb-xa;
    C = xp*B+yp*A;
    a =  2*((B*B)+(A*A));
    b = -4*A*C+(2*xp+xa+xb)*A*B-(2*yp+ya+yb)*(B*B);
    c =  2*(C*C)-(2*xp+xa+xb)*C*B+(xp*(xa+xb)+yp*(ya+yb))*(B*B);
    double y1 = (-b + sqrt((b*b)-4*a*c))/(2*a);
    double y2 = (-b - sqrt((b*b)-4*a*c))/(2*a);

    // % Put point candidates together
    std::vector<double> dvec; dvec.reserve(4);
    Array4D xfm1 = {{x1,y1,0,0}};
    Array4D xfm2 = {{x2,y2,0,0}};
    Array4D xfm3 = {{x1,y2,0,0}};
    Array4D xfm4 = {{x2,y1,0,0}};

    dvec.push_back(edist(xfm1, x));
    dvec.push_back(edist(xfm2, x));
    dvec.push_back(edist(xfm3, x));
    dvec.push_back(edist(xfm4, x));

    double dmax = -1;
    double imax = -1;
    for (int i = 0; i < 4; i++)
    {
        if (dvec[i] > dmax)
        {
            dmax = dvec[i];
            imax = i;
        }
    }

    Array4D xf;
    if (imax == 0)
        xf = xfm1;
    else if (imax == 1)
        xf = xfm2;
    else if (imax == 2)
        xf = xfm3;
    else if (imax == 3)
        xf = xfm4;

    Array4D xs_xf = {{xs[0]-xf[0], xs[1]-xf[1], 0, 0}};
    Array4D xe_xf = {{xe[0]-xf[0], xe[1]-xf[1], 0, 0}};
    double dotp = (xs_xf[0] * xe_xf[0]) + (xs_xf[1] * xe_xf[1]);

    Array4D temp = {{ xf[0], xf[1], 0, 0}};
    proj = temp;

    double din;
    din = dmax;

    bool inside = false;

    if (dotp <= 0.0){

        inside = true;
        dout = din;
    }
    else{
        double de = sqrt((x[0]-xe[0])*(x[0]-xe[0])+(x[1]-xe[1])*(x[1]-xe[1]));
        double ds = sqrt((x[0]-xs[0])*(x[0]-xs[0])+(x[1]-xs[1])*(x[1]-xs[1]));

        if(ds<de){
          dout = ds;
        }
        else{
          dout = de;
        }


      }


    return std::make_pair(din, inside);
}

/// ---------------------------------------------------------------------------
/// \brief calcbiasorientation(double x, double y, Eigen::MatrixXd path)
/// ---------------------------------------------------------------------------
bool CheckPointsOnPath
::checkLaserPointInside(double x, double y){
  if( path_ == NULL ){
    return false;
  }
  int N = (*path_).rows();

  if(N==0){

    return false;

  }

  int coord = (*path_).cols();
  int nsegs = N-1;
  bool isinmin = false;
  double dmin = INFINITO;
  int imin = 0;
  int iref = 0;
  Array4D xfmin = {{0,0,0,0}};
  vector<double> alphavec;
  alphavec.resize(N);
  Array4D sample = {{x, y, 0, 0}};
  // Given a sample x,y, loop over path segments and determine closest
  // segment, respective projection point, and if it is inside its nearest segment
  for (int n=0; n<(N-1); n++){

        Array4D xfrom = {{ (*path_)(n,0), (*path_)(n,1), 0, 0}};
        Array4D xto = {{ (*path_)(n+1,0), (*path_)(n+1,1), 0, 0}};
        Array4D xf = {{0,0,0,0}};
        double dout=0;
        alphavec[n]=atan2(((*path_)(n+1,1)-(*path_)(n,1)),((*path_)(n+1,0)-(*path_)(n,0)));
        std::pair<double, bool> result = distance2Segment(sample, xfrom, xto, xf, dout);
        double din = result.first;
        bool isin = result.second;

        if(isin){
            if(din<dmin){
              dmin = din;
              imin = n;
              xfmin = xf;
              isinmin = isin;
            }
        }else{
            if(dout<dmin && (fabs(dout-dmin) > 0.0000000001 ))
            {
              dmin = dout;
              imin = n;
              xfmin = xf;
              isinmin = isin;
            }
        }
  }
  if( (dmin > path_distance_) ){
    isinmin = false;
  }

  return isinmin;

}



/// ---------------------------------------------------------------------------
/// \brief setPath(std::vector<geometry_msgs::PoseStamped> local_plan)
/// ---------------------------------------------------------------------------
bool CheckPointsOnPath
::setPath(std::vector<geometry_msgs::PoseStamped> local_plan){

  // Get the number of points in the path support
  int N = (int)local_plan.size();
  double xi,yi;

  // path_= new Eigen::MatrixXd(N,2);;
  (*path_).resize(N,2);
  for(int i=0;i<N;i++){
    xi = local_plan[i].pose.position.x;
    yi = local_plan[i].pose.position.y;
    (*path_)(i,0)=xi;
    (*path_)(i,1)=yi;
  }

  return true;

}



/// ---------------------------------------------------------------------------
/// \brief setPath(std::vector<geometry_msgs::PoseStamped> local_plan)
/// ---------------------------------------------------------------------------
bool CheckPointsOnPath
::setPathDistance(double p){

  path_distance_ = p;
  return true;
}
