#include "obstacle.h"
#include <math.h>

static double vector_mag(const double r[3]);
static double unit_vector(const double x[3], double xhat[3]);
static double side_point(double ti, double d);

obstacle::obstacle(double dx, double dy)
{ // This defines a rectangular object.
    size[0] = dx;
    size[1] = dy;
    tag.pose.setIdentity();
    tag.id = 0;
}

obstacle::obstacle(double r)
{ // This defines a circular object
    size[0] = r;
    size[1] = 0.;
    tag.pose.setIdentity();
    tag.id = 0;
}


double obstacle::closest_point(const Pose3D& rover, double rho[3])
{
    auto T_kB = tag.pose.inverse()*rover; // pose to object, from rover
    double tmp_rho_k[3];
    if(size[1] == 0.){
        //Circular 
        
        unit_vector(T_kB.t, tmp_rho_k); // This t is the rover relative to the obstacle.
        for (size_t i = 0; i < 3; i++)
        {
            tmp_rho_k[i] *=size[0]; // This is in the Frame-K
        }
    } else {
        //Rectangle
        tmp_rho_k[0] = side_point(T_kB.t[0], size[0]);
        tmp_rho_k[1] = side_point(T_kB.t[1], size[1]);
        tmp_rho_k[2] = 0.; //Assume the z-value is zero.      
    }
    //Calculate the equivalent point, relative to Frame-B
    T_kB.inverse().transformPoint(tmp_rho_k,rho);
    return vector_mag(rho);
}

static double side_point(double ti, double d){
    double tmp_abs_d = fabs(d);
    return (ti< -tmp_abs_d ? -tmp_abs_d: (ti > tmp_abs_d ? tmp_abs_d:ti));
}

static double vector_mag(const double r[3]){
   return sqrt(r[0]*r[0] + r[1]*r[1]+ r[2]*r[2]); 
}

//This is inplace-safe
static double unit_vector(const double x[3], double xhat[3]){
    double mag = vector_mag(x);
    xhat[0] = x[0]/mag;
    xhat[1] = x[1]/mag;
    xhat[2] = x[2]/mag;
    return mag;
}