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

    emit loadingCompleted(m_stSeqInfo,
                          m_vecSeqBlock,
                          m_mapShapeLib,
                          m_vecRfLib,
                          m_mapRfMagShapeLib,
                          m_vecGzLib,
                          m_vecGyLib,
                          m_vecGxLib,
                          m_vecAdcLib
                          );
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

        GradAxis gradAxis;

        if (pSeqBlock->isTrapGradient(gradAxis = kGZ))
        {
            const GradEvent& gradEvent = pSeqBlock->GetGradEvent(gradAxis);
            const float& amp = gradEvent.amplitude;
            m_stSeqInfo.gzMaxAmp_Hz_m = std::max(m_stSeqInfo.gzMaxAmp_Hz_m, (double)amp);
            m_stSeqInfo.gzMinAmp_Hz_m = std::min(m_stSeqInfo.gzMinAmp_Hz_m, (double)amp);
            int duration_us = gradEvent.rampUpTime + gradEvent.flatTime + gradEvent.rampDownTime;
            double absStartTime_us = dCurrentStartTime_us + gradEvent.delay;
            QVector<double> time{absStartTime_us, absStartTime_us+gradEvent.rampUpTime, absStartTime_us+gradEvent.rampUpTime+gradEvent.flatTime, absStartTime_us+duration_us};
            QVector<double> amplitudes{0, amp, amp, 0};
            GradTrapInfo gradTrapInfo(absStartTime_us, duration_us, time, amplitudes, &gradEvent);
            m_vecGzLib.push_back(gradTrapInfo);
        }

        if (pSeqBlock->isTrapGradient(gradAxis = kGY))
        {
            const GradEvent& gradEvent = pSeqBlock->GetGradEvent(gradAxis);
            const float& amp = gradEvent.amplitude;
            m_stSeqInfo.gyMaxAmp_Hz_m = std::max(m_stSeqInfo.gyMaxAmp_Hz_m, (double)amp);
            m_stSeqInfo.gyMinAmp_Hz_m = std::min(m_stSeqInfo.gyMinAmp_Hz_m, (double)amp);
            int duration_us = gradEvent.rampUpTime + gradEvent.flatTime + gradEvent.rampDownTime;
            double absStartTime_us = dCurrentStartTime_us + gradEvent.delay;
            QVector<double> time{absStartTime_us, absStartTime_us+gradEvent.rampUpTime, absStartTime_us+gradEvent.rampUpTime+gradEvent.flatTime, absStartTime_us+duration_us};
            QVector<double> amplitudes{0, amp, amp, 0};
            GradTrapInfo gradTrapInfo(absStartTime_us, duration_us, time, amplitudes, &gradEvent);
            m_vecGyLib.push_back(gradTrapInfo);
        }

        if (pSeqBlock->isTrapGradient(gradAxis = kGX))
        {
            const GradEvent& gradEvent = pSeqBlock->GetGradEvent(gradAxis);
            const float& amp = gradEvent.amplitude;
            m_stSeqInfo.gxMaxAmp_Hz_m = std::max(m_stSeqInfo.gxMaxAmp_Hz_m, (double)amp);
            m_stSeqInfo.gxMinAmp_Hz_m = std::min(m_stSeqInfo.gxMinAmp_Hz_m, (double)amp);
            int duration_us = gradEvent.rampUpTime + gradEvent.flatTime + gradEvent.rampDownTime;
            double absStartTime_us = dCurrentStartTime_us + gradEvent.delay;
            QVector<double> time{absStartTime_us, absStartTime_us+gradEvent.rampUpTime, absStartTime_us+gradEvent.rampUpTime+gradEvent.flatTime, absStartTime_us+duration_us};
            QVector<double> amplitudes{0, amp, amp, 0};
            GradTrapInfo gradTrapInfo(absStartTime_us, duration_us, time, amplitudes, &gradEvent);
            m_vecGxLib.push_back(gradTrapInfo);
        }

        if (pSeqBlock->isADC())
        {
            const ADCEvent& adcEvent = pSeqBlock->GetADCEvent();
            const double duration_us  = adcEvent.dwellTime * adcEvent.numSamples * 1e-3;
            double absStartTime_us = dCurrentStartTime_us + adcEvent.delay;
            QVector<double> time{absStartTime_us, absStartTime_us, absStartTime_us+duration_us, absStartTime_us+duration_us};
            QVector<double> amplitudes{0, 1, 1, 0};
            AdcInfo adcInfo(absStartTime_us, duration_us, adcEvent.numSamples, adcEvent.dwellTime, time, amplitudes, &adcEvent);
            m_vecAdcLib.push_back(adcInfo);
        }

        dCurrentStartTime_us += pSeqBlock->GetDuration();
        m_stSeqInfo.totalDuration_us += pSeqBlock->GetDuration();
    }
    std::cout << m_vecRfLib.size() << " RF events detetced!\n";
    std::cout << m_vecGzLib.size() << " GZ events detetced!\n";
    std::cout << m_vecGyLib.size() << " GY events detetced!\n";
    std::cout << m_vecGxLib.size() << " GX events detetced!\n";
    return true;
}
