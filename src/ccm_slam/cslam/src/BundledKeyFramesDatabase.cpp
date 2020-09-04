#include <cslam/BundledKeyFramesDatabase.h>

namespace cslam{
BundledKeyFramesDatabase::BundledKeyFramesDatabase (const vocptr pVoc):
        mpVoc(pVoc)
{
    mvInvertedFile.resize((*pVoc).size());
    cout << "+++++ BundledKeyFrame Database Initialized +++++" << endl;
}

void BundledKeyFramesDatabase::add(bkfptr pBKFs)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pBKFs->mBowVec.begin(), vend=pBKFs->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pBKFs);
}

void BundledKeyFramesDatabase::erase(bkfptr pBKFs)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pBKFs->mBowVec.begin(), vend=pBKFs->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<bkfptr> &lBKFs =   mvInvertedFile[vit->first];

        for(list<bkfptr>::iterator lit=lBKFs.begin(), lend= lBKFs.end(); lit!=lend; lit++)
        {
            if(pBKFs==*lit)
            {
                lBKFs.erase(lit);
                break;
            }
        }
    }
}

void BundledKeyFramesDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


}