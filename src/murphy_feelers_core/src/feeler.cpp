#include "murphy_feelers_core/feeler.hpp"
#include <algorithm>

double radians(double degrees) {
    return degrees * (MURPHY_PI / 180.0);
}

cv::Scalar lerp(cv::Scalar a, cv::Scalar b, double t) {
    return cv::Scalar(
        a[0]*(1-t) + b[0]*t,
        a[1]*(1-t) + b[1]*t,
        a[2]*(1-t) + b[2]*t
    );
}

Feeler::Feeler(int x, int y)
    : x_(x), y_(y),
      ox_(x), oy_(y) {
    length_ = dist(x, y);
    original_length_ = length_;
}

double Feeler::dist(int x, int y) const {
    return std::sqrt(x*x + y*y);
}

int Feeler::getX() const { return x_; }
int Feeler::getY() const { return y_; }
double Feeler::getLength() const { return length_; }

int Feeler::getOriginalX() const { return ox_; }
int Feeler::getOriginalY() const { return oy_; }
double Feeler::getOriginalLength() const { return original_length_; }

double Feeler::getBiasAmount() const { return bias_amount_; }

void Feeler::setXY(int x, int y) {
    x_ = x;
    y_ = y;
    length_ = dist(x, y);
}

void Feeler::setLength(double l) {
    double s = l / length_;
    x_ *= s;
    y_ *= s;
    length_ = dist(x_, y_);
}

void Feeler::bias(double amount) {
    if (amount <= 0.0) {
        biased_ = false;
        bias_amount_ = 0.0;
        return;
    }
    biased_ = true;
    bias_amount_ = amount;
    double s = (original_length_ + amount) / original_length_;
    bx_ = ox_ * s;
    by_ = oy_ * s;
}

std::vector<int> Feeler::centerCoords(int x, int y, int w, int h) const {
    return {x + w/2, -y + h/2};
}

void Feeler::update(const cv::Mat &mask) {
    int channels = mask.channels();
    const uint8_t *px = mask.data;

    int xd = ox_ < 0 ? -1 : 1;
    int yd = oy_ < 0 ? -1 : 1;

    double slope = (oy_ != 0) ? double(oy_) / double(ox_) : 0;
    bool inf = (ox_ == 0);

    int x = 0, y = 0, pxp = 0, pyp = 0;

    while (true) {
        if (inf) y++;
        else if (slope == 0) x++;
        else if (std::abs(slope) <= 1) {
            double ny = std::abs(slope) * x;
            if (ny - pyp > 0) y++;
            x++; pyp = y;
        } else {
            double nx = std::abs(1/slope) * y;
            if (nx - pxp > 0) x++;
            y++; pxp = x;
        }

        auto c = centerCoords(x*xd, y*yd, mask.cols, mask.rows);
        int idx = (c[1]*mask.cols + c[0]) * channels;

        for (int i = 0; i < 3; ++i) {
            if (px[idx + i] > 0) {
                setXY(x*xd, y*yd);
                return;
            }
        }

        if (std::abs(x) > std::abs(ox_) || std::abs(y) > std::abs(oy_)) {
            if (biased_) {
                if (std::abs(x) > std::abs(bx_) || std::abs(y) > std::abs(by_)) {
                    setXY(bx_, by_);
                    return;
                }
            } else {
                setXY(ox_, oy_);
                return;
            }
        }
    }
}

void Feeler::draw(cv::Mat &img) const {
    auto s = centerCoords(0, 0, img.cols, img.rows);
    auto e = centerCoords(x_, y_, img.cols, img.rows);
    cv::line(
        img,
        {s[0], s[1]},
        {std::clamp(e[0], 0, img.cols-1), std::clamp(e[1], 0, img.rows-1)},
        color_,
        4
    );
}

Feeler Feeler::operator+(const Feeler &o) const {
    return Feeler(x_ + o.x_, y_ + o.y_);
}

Feeler Feeler::operator-(const Feeler &o) const {
    return Feeler(x_ - o.x_, y_ - o.y_);
}

Feeler Feeler::operator*(int s) const {
    return Feeler(x_ * s, y_ * s);
}

double Feeler::operator*(const Feeler &o) const {
    double nx = x_ / length_;
    double ny = y_ / length_;
    double ox = o.x_ / o.length_;
    double oy = o.y_ / o.length_;
    return nx*ox + ny*oy;
}

