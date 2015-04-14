/*
 * Object.h
 *
 *  Created on: Jan 30, 2015
 *      Author: lukehsiao
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include "ComputerVision.h"


class Object {
  public:
    Object();
    virtual ~Object();

    // Setters and Getters
    void set_x_pos(int x);
    int get_x_pos();

    void set_y_pos(int y);
    int get_y_pos();

    void set_img_x(int x);
    int get_img_x();

    void set_img_y(int y);
    int get_img_y();

    int get_old_x();
    int get_old_y();

    cv::Scalar getHSVmin();
    void setHSVmin(cv::Scalar min);

    cv::Scalar getHSVmax();
    void setHSVmax(cv::Scalar max);



  protected:
    int x_pos, y_pos;
    int img_x, img_y;

    // Used to smooth data
    int old_x_pos, old_y_pos;

    cv::Scalar HSVmin, HSVmax;

};

#endif /* OBJECT_H_ */
