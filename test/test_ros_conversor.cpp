//=============================================================================
//    Copyright (C) 2022 Firefly Automatix Inc., - All Rights Reserved
//                     Author: Vinny Ruia
//                        BSD-3 License
//=============================================================================


#include <gtest/gtest.h>

#include <fields2cover.h>
#include "ros/conversor.h"

#include <tuple>

constexpr static float_t FLOAT_TOL = 1e-6;

class ToPointTest
  : public ::testing::TestWithParam<std::tuple<float_t, float_t, float_t>> {
};

TEST_P(ToPointTest, testConvertPoint) {
  auto params = GetParam();
  auto x = std::get<0>(params);
  auto y = std::get<1>(params);
  auto z = std::get<2>(params);
  conversor::GeometryMsgs::Point point;
  F2CPoint p1(x, y, z);
  conversor::ROS::to(p1, point);

  EXPECT_NEAR(x, point.x, FLOAT_TOL);
  EXPECT_NEAR(y, point.y, FLOAT_TOL);
  EXPECT_NEAR(z, point.z, FLOAT_TOL);
}

TEST_P(ToPointTest, testConvertPoint32) {
  auto params = GetParam();
  auto x = std::get<0>(params);
  auto y = std::get<1>(params);
  auto z = std::get<2>(params);
  conversor::GeometryMsgs::Point32 point;
  F2CPoint p1(x, y, z);
  conversor::ROS::to(p1, point);

  EXPECT_NEAR(x, point.x, FLOAT_TOL);
  EXPECT_NEAR(y, point.y, FLOAT_TOL);
  EXPECT_NEAR(z, point.z, FLOAT_TOL);
}

INSTANTIATE_TEST_SUITE_P(
  ParametrizedToPointTest,
  ToPointTest,
  ::testing::Values(
    std::tuple<float_t, float_t, float_t>{0.0, 0.0, 0.0},
    std::tuple<float_t, float_t, float_t>{1.2, 3.456, -7.9},
    std::tuple<float_t, float_t, float_t>{20001.2787878, -3.456, -0.0}
  )
);

class ToPolyTest
  : public ::testing::Test {
};

TEST_F(ToPolyTest, testToPoly) {
  F2CLinearRing outer_ring{
    F2CPoint(0, 0), F2CPoint(2, 0), F2CPoint(2, 2), F2CPoint(0, 2), F2CPoint(0, 0)};
  F2CLinearRing inner_ring{
    F2CPoint(0.5, 0.5), F2CPoint(1.5, 0.5), F2CPoint(1.5, 1.5),
    F2CPoint(0.5, 1.5), F2CPoint(0.5, 0.5)};
  F2CCell cell;
  cell.addRing(outer_ring);
  cell.addRing(inner_ring);

  std::vector<conversor::GeometryMsgs::Polygon> polygons;

  conversor::ROS::to(cell, polygons);

  EXPECT_NEAR(polygons[0].points[0].x, 0, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[0].y, 0, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[1].x, 2, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[1].y, 0, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[2].x, 2, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[2].y, 2, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[3].x, 0, FLOAT_TOL);
  EXPECT_NEAR(polygons[0].points[3].y, 2, FLOAT_TOL);

  EXPECT_NEAR(polygons[1].points[0].x, 0.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[0].y, 0.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[1].x, 1.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[1].y, 0.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[2].x, 1.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[2].y, 1.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[3].x, 0.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[3].y, 1.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[4].x, 0.5, FLOAT_TOL);
  EXPECT_NEAR(polygons[1].points[4].y, 0.5, FLOAT_TOL);
};

