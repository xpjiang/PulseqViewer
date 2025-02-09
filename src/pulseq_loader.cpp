#include "pulseq_loader.h"

#include <complex>

PulseqLoader::PulseqLoader(QObject *parent)
    : QObject{parent}
    , m_stSeqInfo(SeqInfo())
{
}


void PulseqLoader::process()
{
    emit processingStarted();

    if (!m_spPulseqSeq->load(m_sFilePath.toStdString())) {
        emit errorOccurred("Load " + m_sFilePath + " failed!");
        emit finished();
        return;
    }
    int64_t rfNum(0);
    const int shVersion = m_spPulseqSeq->GetVersion();
    emit versionLoaded(shVersion);

    const int lSeqBlockNum = m_spPulseqSeq->GetNumberOfBlocks();
    m_vecSeqBlock.resize(lSeqBlockNum);

    uint64_t progress(0.);
    for (int ushBlockIndex = 0; ushBlockIndex < lSeqBlockNum; ushBlockIndex++)
    {
        m_vecSeqBlock[ushBlockIndex] = m_spPulseqSeq->GetBlock(ushBlockIndex);
        if (!m_spPulseqSeq->decodeBlock(m_vecSeqBlock[ushBlockIndex]))
        {
            emit errorOccurred(QString("Decode SeqBlock failed, block index: %1").arg(ushBlockIndex));
            emit finished();
            return;
        }
        if (m_vecSeqBlock[ushBlockIndex]->isRF())
        {
            rfNum += 1;
        }
        progress = ushBlockIndex * 100 / lSeqBlockNum;
        emit progressUpdated(progress);
    }

    m_stSeqInfo.rfNum = rfNum;
    m_vecRfLib.reserve(rfNum);
    if (!LoadPulseqEvents())
    {
        emit errorOccurred("LoadPulseqEvents failed!");
        emit finished();
        return;
    }

    emit loadingCompleted(m_stSeqInfo, m_vecSeqBlock, m_mapShapeLib, m_vecRfLib, m_mapRfMagShapeLib);
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

            QPair<int, int> magAbsShapeID(magShapeID, phaseShapeID);
            if (!m_mapRfMagShapeLib.contains(magAbsShapeID))
            {
                double sampleTime = rfInfo.startAbsTime_us;
                const QVector<float>& vecAmp = m_mapShapeLib[rfEvent.magShape];
                const QVector<float>& vecPhase = m_mapShapeLib[rfEvent.phaseShape];
                QVector<double> vecMagnitudes(ushSamples+2, 0.);

                vecMagnitudes[0] = 0;
                double signal(0.);
                for(uint32_t index = 0; index < ushSamples; index++)
                {
                    const float& amp = vecAmp[index];
                    const float& phase = vecPhase[index];
                    signal = std::abs(std::polar(amp, phase));
                    vecMagnitudes[index+1] = signal;
                    sampleTime += rfInfo.dwell;
                }
                vecMagnitudes[ushSamples+1] = 0;
                m_mapRfMagShapeLib.insert(magAbsShapeID, vecMagnitudes);
            }
        }
        dCurrentStartTime_us += pSeqBlock->GetDuration();
        m_stSeqInfo.totalDuration_us += pSeqBlock->GetDuration();
    }
    std::cout << "rf mag lib size: " << m_mapRfMagShapeLib.size() << "\n";

    std::cout << m_vecRfLib.size() << " RF events detetced!\n";
    return true;
}
