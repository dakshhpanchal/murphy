#pragma once

#include <cmath>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define MURPHY_PI 3.141592653589793

double radians(double degrees);

cv::Scalar lerp(cv::Scalar src, cv::Scalar dest, double percent);

class Feeler {
public:
    Feeler(int x, int y);

    int getX() const;
    int getY() const;
    double getLength() const;

    int getOriginalX() const;
    int getOriginalY() const;
    double getOriginalLength() const;

    double getBiasAmount() const;

    void setXY(int x, int y);
    void setLength(double length);
    void bias(double amount);

    void update(const cv::Mat &mask);
    void draw(cv::Mat &image) const;

    Feeler operator+(const Feeler &other) const;
    Feeler operator-(const Feeler &other) const;
    Feeler operator*(int scalar) const;
    double operator*(const Feeler &other) const;

private:
    double dist(int x, int y) const;
    std::vector<int> centerCoords(int x, int y, int w, int h) const;

    int x_, y_;
    double length_;

    int ox_, oy_;
    double original_length_;

    bool biased_{false};
    int bx_{0}, by_{0};
    double bias_amount_{0.0};

    cv::Scalar color_{0, 200, 0};
};

