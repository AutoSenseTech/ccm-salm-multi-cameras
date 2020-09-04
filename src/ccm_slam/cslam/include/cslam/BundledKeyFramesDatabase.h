#ifndef CSLAM_BUNDLEDKEYFRAMESDATABASE_H_
#define CSLAM_BUNDLEDKEYFRAMESDATABASE_H_

//C++
#include <boost/shared_ptr.hpp>
#include <vector>
#include <list>
#include <set>
#include <mutex>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/BundledKeyFrames.h>
#include <cslam/BundledMap.h>
#include <cslam/Frame.h>

#include <cslam/MapPoint.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>

using namespace std;
using namespace estd;


namespace cslam{
    class BundledKeyFrames;
    class Frame;
    class BundledMap;

class BundledKeyFramesDatabase{
public:
    typedef boost::shared_ptr<BundledKeyFramesDatabase> bdbptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
    typedef boost::shared_ptr<BundledMap> bmapptr;
public:
    BundledKeyFramesDatabase(const vocptr pVoc);
    
    void add(bkfptr pBKF);

    void erase(bkfptr pBKF);

    void clear();

    // Loop Detection
    //todo
    //std::vector<bkfptr> DetectLoopCandidates(bkfptr pBKF, float minScore);
    //vector<bkfptr> DetectMapMatchCandidates(bkfptr pBKF, float minScore, bmapptr pBMap);
    // Relocalization
    //std::vector<bkfptr> DetectRelocalizationCandidates(Frame& F);

protected:

    // Associated vocabulary
    const vocptr mpVoc;

    // Inverted file
    std::vector<list<bkfptr> > mvInvertedFile;

    // Mutex
    std::mutex mMutex;
};

}

#endif