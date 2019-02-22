#include "depth_filter.h"
#include <boost/math/distributions/normal.hpp>

void Depth_Filter::computeTau(const cv::Mat& T12, const cv::Point3d& P, const double& px_error_angle, double& tau)
{

    cv::Point3d t = cv::Point3d(T12.at<double>(0,3), T12.at<double>(1,3), T12.at<double>(2,3));
    cv::Point3d a = P-t;

    double apha = acos( (P.x*t.x + P.y*t.y + P.z*t.z) / (cv::norm(P)*cv::norm(t)) ); 
    double beta = acos( -(a.x*t.x + a.y*t.y + a.z*t.z) / (cv::norm(a)*cv::norm(t)) );
    double beta_plus = beta + px_error_angle;
    double gamma = CV_PI - apha - beta_plus;
    double P_plus_norm = cv::norm(t)*sin(beta_plus)/sin(gamma);

    tau = P_plus_norm - cv::norm(P);

}

void Depth_Filter::updateFilter(const double& x, const double& tau2, Seed* seed)
{

    double norm_scale = sqrt(seed->sigma2 + tau2);
    if(std::isnan(norm_scale))
        return;
    boost::math::normal_distribution<double> nd(seed->mu, norm_scale);
    double s2 = 1./(1./seed->sigma2 + 1./tau2);
    double m = s2*(seed->mu/seed->sigma2 + x/tau2);
    double C1 = seed->a/(seed->a+seed->b) * boost::math::pdf(nd, x);
    double C2 = seed->b/(seed->a+seed->b) * 1./seed->z_range;
    double normalization_constant = C1 + C2;
    C1 /= normalization_constant;
    C2 /= normalization_constant;
    double f = C1*(seed->a+1.)/(seed->a+seed->b+1.) + C2*seed->a/(seed->a+seed->b+1.);
    double e = C1*(seed->a+1.)*(seed->a+2.)/((seed->a+seed->b+1.)*(seed->a+seed->b+2.))
             + C2*seed->a*(seed->a+1.0f)/((seed->a+seed->b+1.0f)*(seed->a+seed->b+2.0f));
   
    // update parameters
    double mu_new = C1*m+C2*seed->mu;
    seed->sigma2 = C1*(s2 + m*m) + C2*(seed->sigma2 + seed->mu*seed->mu) - mu_new*mu_new;
    seed->mu = mu_new;
    seed->a = (e-f)/(f-e/f);
    seed->b = seed->a*(1.0f-f)/f;

}
