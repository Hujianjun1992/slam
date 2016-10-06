# pragma once

#include <iostream>
#include <string>
#include <map>
#include <fstream>
using namespace std;



//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

class ParameterReader
{
 public:
  ParameterReader(string filename = "./cfg/parameters.txt")
    {
      ifstream fin(filename.c_str());
      if (!fin ) {
        cerr << "parameter file does not exist." << endl;
        return;
      }
      while (!fin.eof()) {
        string str;
        getline(fin,str);
        if (str[0] == '#') {
          continue;
        }
        int pos = str.find("=");
        if (pos == -1) {
          continue;
        }
        string key = str.substr(0, pos);
        string value = str.substr(pos+1, str.length());
        data[key] = value ;
 //       cout << data[key] << endl;

        if (!fin.good()) {
          break;
        }
      }
    }
  string getData(string key)
  {
//	cout << " xuhong " << endl;
    map<string,string>::iterator iter = data.find(key);
    if (iter == data.end()) {
      cerr << "Parameter name" << key <<" not found!" << endl;
      return string("NOT_FOUND");
    }
    return iter->second;
  }
 public:
  map<string,string>data;
};

template<class T>
void info_show(string str,T arg)
{
  //  string str = atos(arg);
  cout << YELLOW"\t" << str << ":" << arg << RESET << endl;
}
