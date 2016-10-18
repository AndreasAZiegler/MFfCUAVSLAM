#ifndef MACSLAM_DATABASE_H_
#define MACSLAM_DATABASE_H_

//C++
#include <boost/shared_ptr.hpp>
#include <vector>
#include <list>
#include <set>
#include <mutex>

//MCP SLAM
#include <helper/estd.h>
#include <macslam/Datatypes.h>
#include <macslam/ORBVocabulary.h>
#include <macslam/KeyFrame.h>
#include <macslam/Map.h>
#include <macslam/Frame.h>

#include <thirdparty/DBoW2/DBoW2/BowVector.h>

using namespace std;
using namespace estd;

namespace macslam {

//forward decs
class Map;
//class Frame;
//------------

class KeyFrameDatabase
{
public:
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<Map> mapptr;
public:

   KeyFrameDatabase(const vocptr pVoc);

    void add(kfptr pKF);

    void erase(kfptr pKF);

    void clear();

   // Loop Detection
    vector<kfptr> DetectLoopCandidates(kfptr pKF, float minScore);
    vector<kfptr> DetectMapMatchCandidates(kfptr pKF, float minScore, mapptr pMap);

   // Relocalization
   std::vector<kfptr> DetectRelocalizationCandidates(Frame& F);

    #ifdef STATS
    //use database to stroe data traffic stats, because its centralized
    struct timeval mtStartTotal;
    struct timeval mtLastStamp;
    double mdTimeThres;
    vector<double> mvtTimeStamp;
    vector<double> mvdLoadIn;
    vector<double> mvdLoadOut;
    size_t mdSizeOfMsgIn; //in Byte
    size_t mdSizeOfMsgOut;

    void InsertDataMeasurements(size_t in, size_t out);
    #endif

protected:

  // Associated vocabulary
  const vocptr mpVoc;

  // Inverted file
  std::vector<list<kfptr> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};



} //end namespace

#endif
