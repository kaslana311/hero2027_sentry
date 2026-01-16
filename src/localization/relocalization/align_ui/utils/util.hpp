#pragma once
#include <stdbool.h>

struct align_res
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  int flag;
};

struct reloc_res
{
  double fitness;
  double dtrans;
  double droll;
  double dpitch;
  double dyaw;
  bool isOk;
};