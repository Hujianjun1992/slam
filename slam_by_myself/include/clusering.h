#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <ctime>

using namespace std;
typedef unsigned int unit;

struct Cluster
{
  vector <double> centroid;
  vector<unit> samples;
};

double cal_distance(vector<double> a, vector<double> b);

vector<Cluster> k_means(vector<vector<double>> trainX, uint k, uint maxepoches);
