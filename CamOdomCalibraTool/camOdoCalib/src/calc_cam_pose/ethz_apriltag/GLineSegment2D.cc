#include "GLineSegment2D.h"
#include <limits>

namespace AprilTags {

GLineSegment2D::GLineSegment2D(const std::pair<float, float> &p0Arg,
                               const std::pair<float, float> &p1Arg)
    : line(p0Arg, p1Arg), p0(p0Arg), p1(p1Arg), weight() {}

GLineSegment2D GLineSegment2D::lsqFitXYW(
    const std::vector<XYWeight> &xyweight) {
  GLine2D gline = GLine2D::lsqFitXYW(xyweight);  //得到聚类点的主方向，dx（-sin(theta)），dy（cos(theta)），pts（图像灰度质心点）
  float maxcoord = -std::numeric_limits<float>::infinity();
  float mincoord = std::numeric_limits<float>::infinity();
  ;

  for (unsigned int i = 0; i < xyweight.size(); i++) {
    std::pair<float, float> p(xyweight[i].x, xyweight[i].y);
    float coord = gline.getLineCoordinate(p);      //根据简单绘图可知，x'+y'的值（归一化p点到直线的法向量的距离（有正负）），记录其最大和最小值，其分别是法线两边最远的点
    maxcoord = std::max(maxcoord, coord);
    mincoord = std::min(mincoord, coord);
  }

  std::pair<float, float> minValue = gline.getPointOfCoordinate(mincoord);//将法线两边分别离法线最远的点投影到 过聚类中点且与法线垂直的直线上（拟合直线） 从而得到这两个点的坐标，分别是直线的两端
  std::pair<float, float> maxValue = gline.getPointOfCoordinate(maxcoord);
  return GLineSegment2D(minValue, maxValue);
}

}  // namespace AprilTags
