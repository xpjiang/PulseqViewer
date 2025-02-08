#include "pulseq_loader.h"

PulseqLoader::PulseqLoader(QObject *parent)
    : QObject{parent}
{
    m_stSeqInfo = SeqInfo();
}


void PulseqLoader::process()
{
    emit processingStarted();

    if (!m_spPulseqSeq->load(m_sFilePath.toStdString())) {
        emit errorOccurred("Load " + m_sFilePath + " failed!");
        emit finished();
        return;
    }

    const int shVersion = m_spPulseqSeq->GetVersion();
    emit versionLoaded(shVersion);

    const int lSeqBlockNum = m_spPulseqSeq->GetNumberOfBlocks();
    m_vecSeqBlock.resize(lSeqBlockNum);

    uint64_t progress(0.);
    for (int ushBlockIndex = 0; ushBlockIndex < lSeqBlockNum; ushBlockIndex++) {
        m_vecSeqBlock[ushBlockIndex] = m_spPulseqSeq->GetBlock(ushBlockIndex);
        if (!m_spPulseqSeq->decodeBlock(m_vecSeqBlock[ushBlockIndex])) {
            emit errorOccurred(QString("Decode SeqBlock failed, block index: %1").arg(ushBlockIndex));
            emit finished();
            return;
        }
        if (m_vecSeqBlock[ushBlockIndex]->isRF()) {
            m_lRfNum += 1;
        }
        progress = ushBlockIndex * 100 / lSeqBlockNum;
        emit progressUpdated(progress);
    }

    m_vecRfLib.reserve(m_lRfNum);
    if (!LoadPulseqEvents())
    {
        emit errorOccurred("LoadPulseqEvents failed!");
        emit finished();
        return;
    }

    emit loadingCompleted(m_stSeqInfo, m_vecSeqBlock, m_mapShapeLib, m_vecRfLib);
    emit finished();
}

bool PulseqLoader::LoadPulseqEvents()
{

    if (m_vecSeqBlock.size() == 0) return true;
    double dCurrentStartTime_us(0.);
    for (const auto& pSeqBlock : m_vecSeqBlock)
    {
        if (pSeqBlock->isRF())
        {
            const RFEvent& rfEvent = pSeqBlock->GetRFEvent();
            const int& ushSamples = pSeqBlock->GetRFLength();
            const float& fDwell = pSeqBlock->GetRFDwellTime();
            const double& dDuration_us = ushSamples * fDwell;
            RfInfo rfInfo(dCurrentStartTime_us+rfEvent.delay, dDuration_us, ushSamples, fDwell, &rfEvent);
            m_vecRfLib.push_back(rfInfo);

            const int& magShapeID = rfEvent.magShape;
            if (!m_mapShapeLib.contains(magShapeID))
            {
                QVector<float> vecAmp(ushSamples, 0.f);
                const float* fAmp = pSeqBlock->GetRFAmplitudePtr();
                std::memcpy(vecAmp.data(), fAmp, ushSamples * sizeof(float));
                m_mapShapeLib.insert(magShapeID, vecAmp);
            }

            const int& phaseShapeID = rfEvent.phaseShape;
            if (!m_mapShapeLib.contains(phaseShapeID))
            {
                QVector<float> vecPhase(ushSamples, 0.f);
                const float* fPhase = pSeqBlock->GetRFPhasePtr();
                std::memcpy(vecPhase.data(), fPhase, ushSamples * sizeof(float));
                m_mapShapeLib.insert(phaseShapeID, vecPhase);
            }

        }
        dCurrentStartTime_us += pSeqBlock->GetDuration();
        m_stSeqInfo.totalDuration_us += pSeqBlock->GetDuration();
    }

    std::cout << m_vecRfLib.size() << " RF events detetced!\n";
    return true;
}
