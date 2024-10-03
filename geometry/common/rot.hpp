//
//  rot.hpp
//  gicp3
//
//  Created by eythymisk on 31/7/24.
//

#ifndef rot_hpp
#define rot_hpp

#include <Eigen/Dense>
#include <cmath>

class Rot {
public:
    // Rotation matrix from Euler angles
    static Eigen::Matrix3f eul2rotm(const Eigen::Vector3f& a) {
        
        Eigen::Matrix3f R;
        
        float ax = a(0);
        float ay = a(1);
        float az = a(2);

        float cax = std::cos(ax);
        float sax = std::sin(ax);
        float cay = std::cos(ay);
        float say = std::sin(ay);
        float caz = std::cos(az);
        float saz = std::sin(az);
        
        R(0, 0) = cay * caz;
        R(0, 1) = sax * say * caz - cax * saz;
        R(0, 2) = cax * say * caz + sax * saz;

        R(1, 0) = cay * saz;
        R(1, 1) = sax * say * saz + cax * caz;
        R(1, 2) = cax * say * saz - sax * caz;

        R(2, 0) = -say;
        R(2, 1) = sax * cay;
        R(2, 2) = cax * cay;
        
        return R;
    }

    // Rotation matrix derivative with respect to Euler angles
    static void eul2rotm(const Eigen::Vector3f& a, Eigen::Matrix3f& R, Eigen::Matrix3f& dR_dax, Eigen::Matrix3f& dR_day, Eigen::Matrix3f& dR_daz) {
        
        float ax = a(0);
        float ay = a(1);
        float az = a(2);

        float cax = std::cos(ax);
        float sax = std::sin(ax);
        float cay = std::cos(ay);
        float say = std::sin(ay);
        float caz = std::cos(az);
        float saz = std::sin(az);
        
        R(0, 0) = cay * caz;
        R(0, 1) = sax * say * caz - cax * saz;
        R(0, 2) = cax * say * caz + sax * saz;
        R(1, 0) = cay * saz;
        R(1, 1) = sax * say * saz + cax * caz;
        R(1, 2) = cax * say * saz - sax * caz;
        R(2, 0) = -say;
        R(2, 1) = sax * cay;
        R(2, 2) = cax * cay;
        
        dR_dax(0, 0) = 0.0f;
        dR_dax(0, 1) = cax * say * caz + sax * saz;
        dR_dax(0, 2) = -sax * say * caz + cax * saz;
        dR_dax(1, 0) = 0.0f;
        dR_dax(1, 1) = cax * say * saz - sax * caz;
        dR_dax(1, 2) = -sax * say * saz - cax * caz;
        dR_dax(2, 0) = 0.0f;
        dR_dax(2, 1) = cax * cay;
        dR_dax(2, 2) = -sax * cay;
        
        dR_day(0, 0) = -say * caz;
        dR_day(0, 1) = sax * cay * caz;
        dR_day(0, 2) = cax * cay * caz;
        dR_day(1, 0) = -say * saz;
        dR_day(1, 1) = sax * cay * saz;
        dR_day(1, 2) = cax * cay * saz;
        dR_day(2, 0) = -cay;
        dR_day(2, 1) = -sax * say;
        dR_day(2, 2) = -cax * say;

        dR_daz(0, 0) = -cay * saz;
        dR_daz(0, 1) = -sax * say * saz - cax * caz;
        dR_daz(0, 2) = -cax * say * saz + sax * caz;
        dR_daz(1, 0) = cay * caz;
        dR_daz(1, 1) = sax * say * caz - cax * saz;
        dR_daz(1, 2) = cax * say * caz + sax * saz;
        dR_daz(2, 0) = 0.0f;
        dR_daz(2, 1) = 0.0f;
        dR_daz(2, 2) = 0.0f;
    }

    // Euler angles from rotation matrix
    static Eigen::Vector3f rotm2eul(const Eigen::Matrix3f& R) {
        
        Eigen::Vector3f a;
        
        if (R(0, 2) == 1) {
            a(0) = std::atan2(-R(1, 2), R(1, 1));
            a(1) = M_PI / 2;
            a(2) = 0;
        } else if (R(0, 2) == -1) {
            a(0) = std::atan2(-R(1, 2), R(1, 1));
            a(1) = -M_PI / 2;
            a(2) = 0;
        } else {
            a(0) = std::atan2(R(2, 1), R(2, 2));
            a(1) = std::asin(-R(2, 0));
            a(2) = std::atan2(R(1, 0), R(0, 0));
        }
        return a;
    }
};

#endif /* rot_hpp */
