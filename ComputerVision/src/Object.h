/*
 * Object.h
 *
 *  Created on: Jan 30, 2015
 *      Author: lukehsiao
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include <opencv/cv.h>

class Object {
  public:
    Object();
    virtual ~Object();

    // Setters and Getters
    void set_x_pos(int x);
    int get_x_pos();

    void set_y_pos(int y);
    int get_y_pos();

    cv::Scalar getHSVmin();
    void setHSVmin(cv::Scalar min);

    cv::Scalar getHSVmax();
    void setHSVmax(cv::Scalar max);

  protected:
    int x_pos, y_pos;
    cv::Scalar HSVmin, HSVmax;
};

#endif /* OBJECT_H_ */
