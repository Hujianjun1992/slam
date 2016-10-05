#include "clusering.h"

double cal_distance(vector<double> a, vector<double> b)
{
  uint da = a.size();
  uint db = b.size();
  if (da != db) {
    cerr << "Dimensions of two vectors must be same !!" << endl;
  }
  double val = 0.0;
  for (uint i = 0; i < da; ++i) {
    val += pow( (a[i] - b[i]), 2);
  }
  return pow(val, 0.5);
}

vector<Cluster> k_means(vector<vector<double>> trainX, uint k, uint maxepoches)
{
  const uint row_num = trainX.size();
  const uint col_num = trainX[0].size();

  vector<Cluster> clusters(k);
  uint seed = (uint)time(NULL);
  for (uint i = 0; i < k; ++i) {
    srand(seed);
    int c = rand() % row_num;
    clusters[i].centroid = trainX[c];
    seed = rand();
  }

  for (uint it = 0; it < maxepoches; ++it) {
    for (uint i = 0; i < k; ++i) {
      clusters[i].samples.clear();
    }
    for (uint j = 0; j < row_num; ++j) {
      uint c = 0;
      double min_distance = cal_distance(trainX[j], clusters[c].centroid);
      for (uint i = 1; i < k; ++i) {
        double distance = cal_distance(trainX[j],clusters[i].centroid);
        if(distance < min_distance){
          min_distance = distance;
          c = i;
        }
      }
      clusters[c].samples.push_back(j);
    }

    for (uint i = 0; i < k; ++i) {
      vector<double> val(col_num, 0.0);
      for (uint j = 0; j < clusters[i].samples.size(); ++j) {
        uint samples = clusters[i].samples[j];
        for (uint d = 0; d < col_num; ++d) {
          val[d] += trainX[samples][d];
          if (j == clusters[i].samples.size() - 1) {
            clusters[i].centroid[d] = val[d] / clusters[i].samples.size();
          }
        }
      }
    }
  }
  return clusters;
}
