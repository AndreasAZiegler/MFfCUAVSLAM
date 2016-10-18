#ifndef ESTD_H_
#define ESTD_H_

#include <vector>
#include <thread>
#include <mutex>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>

extern int extVerboseModeClient;
extern int extVerboseModeServer;

extern int extVerboseMode;
extern bool showparams;

using namespace std;

//std::bad_alloc::bad_alloc();

namespace estd{

#define defpair make_pair(-1,-1) //default pair
#define defid -1 //default id

//#define STATS
#undef STATS
#define VISUALIZATION
//#undef VISUALIZATION
//#define PERFORMANCE
#undef PERFORMANCE
//#define DEBUG
#undef DEBUG
//#define HACKZ
#undef HACKZ
#define MAPFUSION
//#undef MAPFUSION

typedef pair<size_t,size_t> idpair;
typedef boost::shared_ptr<std::thread> threadptr;
class UniqueIdDispenser;
typedef boost::shared_ptr<UniqueIdDispenser> uidptr;

class infrastructure_ex : public std::exception
{
public:
    virtual const char* what() const throw()
      {
        return "EXCEPTION: Bad Infrastructure";
      }
};

class inheritance_ex : public std::exception
{
public:
    virtual const char* what() const throw()
      {
        return "EXCEPTION: Bad Inheritance";
      }
};

template<typename T> class vec : public std::vector<T>
{
public:
    using std::vector<T>::vector; //use constructor from std::vector

    T& operator[](int i) { return std::vector<T>::at(i); } //range check - throws out_of_range if violated

    const T& operator[](int i) const { return std::vector<T>::at(i); } //range check const objects
};

class UniqueIdDispenser
{
public:
    UniqueIdDispenser() : mLastId(0) {}

    size_t GetId()
    {
        unique_lock<mutex> lock(mMutexId);
        ++mLastId;
        return mLastId;
    }

    size_t GetLastId()
    {
        unique_lock<mutex> lock(mMutexId);
        return mLastId;
    }

private:
    size_t mLastId;
    mutex mMutexId;
};

static void UpdateStats(double& mean, double& ssd, size_t& n, double x)
{
    ++n;
    double delta = x - mean;
    mean += delta/static_cast<double>(n);
    ssd += delta*(x-mean);
}

static void WriteTrafficToFile(vector<double> vTime,vector<double> vIn, vector<double> vOut,string id)
{
    std::stringstream* pFile;

    pFile = new stringstream;
    *pFile << "slamstats_" << id << ".csv";

    ofstream myfile (pFile->str());
    if (myfile.is_open())
    {
        myfile << "Time,DataIn,DataOut"<< "\n";

        for(int it=0;it<vTime.size();++it)
        {
            myfile << std::fixed << std::setprecision(3) << vTime[it] << "," << vIn[it] << "," << vOut[it] << "\n";
        }

        myfile.close();
    }
    else cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m WriteTrafficToFile: Unable to open file" << endl;

    delete pFile;
}

} //end namespace estd

#endif // ESTD_H_
