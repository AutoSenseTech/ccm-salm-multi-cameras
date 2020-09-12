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

vector<BundledKeyFramesDatabase::bkfptr> BundledKeyFramesDatabase::DetectMapMatchCandidates(bkfptr pBKF, float minScore, bmapptr pBMap)
{
    list<bkfptr> lBKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes that belong to KF's map
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pBKF->mBowVec.begin(), vend=pBKF->mBowVec.end(); vit != vend; vit++)
        {
            list<bkfptr> &lBKFs =  mvInvertedFile[vit->first];

            for(list<bkfptr>::iterator lit=lBKFs.begin(), lend= lBKFs.end(); lit!=lend; lit++)
            {
                bkfptr pBKFi=*lit;

                if(!(pBKFi->mMatchQuery==pBKF->mId)) // pBKFi还没有标记为pBKF的候选帧
                {
                    pBKFi->mnLoopWords=0;

                    if(!pBMap->msuAssClients.count(pBKFi->mId.second))  // 与pBKF不同的client的关键帧不进入闭环候选帧
                    {
                        pBKFi->mMatchQuery=pBKF->mId;
                        lBKFsSharingWords.push_back(pBKFi);
                    }
                }
                pBKFi->mnLoopWords++; // 记录pBKFi与pBKF具有相同word的个数
            }
        }
    }

    if(lBKFsSharingWords.empty())
        return vector<bkfptr>();

    list<pair<float,bkfptr> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // 步骤2：统计所有闭环候选帧中与pKF具有共同单词最多的单词数
    int maxCommonWords=0;
    for(list<bkfptr>::iterator lit=lBKFsSharingWords.begin(), lend= lBKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<bkfptr>::iterator lit=lBKFsSharingWords.begin(), lend= lBKFsSharingWords.end(); lit!=lend; lit++)
    {
        bkfptr pBKFi = *lit;

        if(pBKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pBKF->mBowVec,pBKFi->mBowVec);

            pBKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pBKFi)); //候选关键帧以及它和当前帧的相似度的score
        }
    }

    if(lScoreAndMatch.empty())
        return vector<bkfptr>();

    list<pair<float,bkfptr> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 步骤4：具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,bkfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        bkfptr pBKFi = it->second;
        vector<bkfptr> vpNeighs = pBKFi->GetBestCovisibilityBundledKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        bkfptr pBestBKF = pBKFi;
        for(vector<bkfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            bkfptr pBKF2 = *vit;

            if(pBKF2->mMatchQuery==pBKF->mId && pBKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pBKF2->mLoopScore;
                if(pBKF2->mLoopScore>bestScore)
                {
                    pBestBKF=pBKF2;
                    bestScore = pBKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestBKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<bkfptr> spAlreadyAddedBKF;
    vector<bkfptr> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,bkfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            bkfptr pBKFi = it->second;
            if(!spAlreadyAddedBKF.count(pBKFi))
            {
                vpLoopCandidates.push_back(pBKFi);
                spAlreadyAddedBKF.insert(pBKFi);
            }
        }
    }


    return vpLoopCandidates;
}

void BundledKeyFramesDatabase::ResetMPs()
{
    unique_lock<mutex> lock(mMutexMPs);

    mmbDirectBad.clear();
    mmpMPs.clear();
}


}