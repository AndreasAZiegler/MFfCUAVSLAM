#include <macslam/Database.h>

namespace macslam {

KeyFrameDatabase::KeyFrameDatabase(const vocptr pVoc):
    mpVoc(pVoc)
{
    mvInvertedFile.resize((*pVoc).size());

    #ifdef STATS
    gettimeofday(&mtStartTotal,NULL);
    gettimeofday(&mtLastStamp,NULL);

    mdTimeThres=1.0;
    mvtTimeStamp.clear();
    mvdLoadIn.clear();
    mvdLoadOut.clear();
    mdSizeOfMsgIn=0;
    mdSizeOfMsgOut=0;
    #endif
}


void KeyFrameDatabase::add(kfptr pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(kfptr pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<kfptr> &lKFs =   mvInvertedFile[vit->first];

        for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

vector<KeyFrameDatabase::kfptr> KeyFrameDatabase::DetectLoopCandidates(kfptr pKF, float minScore)
{
    set<kfptr> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<kfptr> lKFsSharingWords;

    set<kfptr> spAllKfsInMap = pKF->GetMapptr()->GetMspKeyFrames();

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<kfptr> &lKFs =   mvInvertedFile[vit->first];

            for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                kfptr pKFi=*lit;

//                if(pKFi->mnId == pKF->mnId && pKFi->mClientId == pKF->mClientId) continue;
                if(pKFi->mId == pKF->mId) continue;

                if(!spAllKfsInMap.count(pKFi)) continue; //Only consider KFs that belong to the same map

//                if(!(pKFi->mnLoopQuery==pKF->mnId && pKFi->mnLoopQueryClientId==pKF->mClientId)) //ID-Tag
                if(!(pKFi->mLoopQuery==pKF->mId)) //ID-Tag
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
//                        pKFi->mnLoopQuery=pKF->mnId;
//                        pKFi->mnLoopQueryClientId=pKF->mClientId; //ID-Tag
                        pKFi->mLoopQuery=pKF->mId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,kfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        kfptr pKFi = it->second;
        vector<kfptr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        kfptr pBestKF = pKFi;
        for(vector<kfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            kfptr pKF2 = *vit;
//            if(pKF2->mnLoopQuery==pKF->mnId && pKFi->mnLoopQueryClientId==pKF->mClientId && pKF2->mnLoopWords>minCommonWords) //ID-Tag
            if(pKF2->mLoopQuery==pKF->mId && pKF2->mnLoopWords>minCommonWords) //ID-Tag
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<kfptr> spAlreadyAddedKF;
    vector<kfptr> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,kfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            kfptr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

vector<KeyFrameDatabase::kfptr> KeyFrameDatabase::DetectMapMatchCandidates(kfptr pKF, float minScore, mapptr pMap)
{
//    set<kfptr> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<kfptr> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes that belong to KF's map
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<kfptr> &lKFs =   mvInvertedFile[vit->first];

            for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                kfptr pKFi=*lit;
//                if(!(pKFi->mnLoopQuery==pKF->mnId && pKFi->mnLoopQueryClientId==pKF->mClientId)) //ID-Tag
                if(!(pKFi->mMatchQuery==pKF->mId)) //ID-Tag
                {
                    pKFi->mnLoopWords=0;
                    //if(!spConnectedKeyFrames.count(pKFi))
//                    if(!pMap->msuAssClients.count(pKFi->mClientId))
                    if(!pMap->msuAssClients.count(pKFi->mId.second))
                    {
//                        pKFi->mnLoopQuery=pKF->mnId;
//                        pKFi->mnLoopQueryClientId=pKF->mClientId; //ID-Tag
                        pKFi->mMatchQuery=pKF->mId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,kfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        kfptr pKFi = it->second;
        vector<kfptr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        kfptr pBestKF = pKFi;
        for(vector<kfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            kfptr pKF2 = *vit;
//            if(pKF2->mnLoopQuery==pKF->mnId && pKFi->mnLoopQueryClientId==pKF->mClientId && pKF2->mnLoopWords>minCommonWords) //ID-Tag
            if(pKF2->mMatchQuery==pKF->mId && pKF2->mnLoopWords>minCommonWords) //ID-Tag
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<kfptr> spAlreadyAddedKF;
    vector<kfptr> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,kfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            kfptr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

vector<KeyFrameDatabase::kfptr> KeyFrameDatabase::DetectRelocalizationCandidates(Frame &F)
{
    list<kfptr> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F.mBowVec.begin(), vend=F.mBowVec.end(); vit != vend; vit++)
        {
            list<kfptr> &lKFs =   mvInvertedFile[vit->first];

            for(list<kfptr>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                kfptr pKFi=*lit;
                if(pKFi->mRelocQuery!=F.mId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mRelocQuery=F.mId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<kfptr>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,kfptr> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<kfptr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F.mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<kfptr>();

    list<pair<float,kfptr> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,kfptr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        kfptr pKFi = it->second;
        vector<kfptr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        kfptr pBestKF = pKFi;
        for(vector<kfptr>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            kfptr pKF2 = *vit;
            if(pKF2->mRelocQuery!=F.mId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<kfptr> spAlreadyAddedKF;
    vector<kfptr> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,kfptr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            kfptr pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

#ifdef STATS
void KeyFrameDatabase::InsertDataMeasurements(size_t in, size_t out)
{
    struct timeval tNow;
    gettimeofday(&tNow,NULL);
    double dEl = (tNow.tv_sec - mtLastStamp.tv_sec) + (tNow.tv_usec - mtLastStamp.tv_usec) / 1000000.0;

    if(dEl >=mdTimeThres)
    {
        double dElTotal = (tNow.tv_sec - mtStartTotal.tv_sec) + (tNow.tv_usec - mtStartTotal.tv_usec)  / 1000000.0;

        mvtTimeStamp.push_back(dElTotal);
        mvdLoadIn.push_back(static_cast<double>(mdSizeOfMsgIn)/1024.0);
        mvdLoadOut.push_back(static_cast<double>(mdSizeOfMsgOut)/1024.0);

        gettimeofday(&mtLastStamp,NULL);

        mdSizeOfMsgIn = 0;
        mdSizeOfMsgOut = 0;

//        cout << "Vector Size: " << mvtTimeStamp.size() << endl;
//        cout << "mvdLoadIn Size: " << mvdLoadIn.size() << endl;
//        cout << "mvdLoadOut Size: " << mvdLoadOut.size() << endl;

        estd::WriteTrafficToFile(mvtTimeStamp,mvdLoadIn,mvdLoadOut,"S");
    }

    mdSizeOfMsgIn += in;
    mdSizeOfMsgOut += out;
}
#endif

} //end ns
