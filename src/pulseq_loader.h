#ifndef PULSEQ_LOADER_H
#define PULSEQ_LOADER_H

#include <QObject>
#include <QMap>
#include <ExternalSequence.h>

typedef QMap<QPair<int, int>, QVector<double>> RfTimeWaveShapeMap;

enum GradAxis
{
    kGX = 0,
    kGY = 1,
    kGZ = 2
};

struct SeqInfo
{
    double totalDuration_us;
    // RF
    uint64_t rfNum;
    double rfMaxAmp_Hz;
    double rfMinAmp_Hz;

    // GZ
    uint64_t gzNum;
    double gzMaxAmp_Hz_m;
    double gzMinAmp_Hz_m;

    SeqInfo()
        : totalDuration_us(0.)
        , rfNum(0)
        , rfMaxAmp_Hz(0.)
        , rfMinAmp_Hz(0.)
        , gzNum(0)
        , gzMaxAmp_Hz_m(0.)
        , gzMinAmp_Hz_m(0.)
    {}

    void reset()
    {
        totalDuration_us = 0;

        // RF
        rfNum = 0;
        rfMaxAmp_Hz = 0.;
        rfMinAmp_Hz = 0.;
        // GZ
        gzNum = 0;
        gzMaxAmp_Hz_m = 0.;
        gzMinAmp_Hz_m = 0.;
    }
};

struct RfInfo
{
    double startAbsTime_us;
    double duration_us;
    uint16_t samples;
    float dwell;
    const RFEvent* event;

    RfInfo(
        const double& dStartAbsTime_us,
        const double& dDuration_us,
        const uint16_t ushSamples,
        const float& fDwell,
        const RFEvent* rfEvent)
        : startAbsTime_us(dStartAbsTime_us)
        , duration_us(dDuration_us)
        , samples(ushSamples)
        , dwell(fDwell)
        , event(rfEvent)
    {}
};

struct GradTrapInfo
{
    double startAbsTime_us;
    long duration_us;
    QVector<double> time;
    QVector<double> amplitude;
    const GradEvent* event;

    GradTrapInfo(
        const double& dStartAbsTime_us,
        const double& dDuration_us,
        const QVector<double>& vecTime,
        const QVector<double>& vecAmplitude,
        const GradEvent* gradEvent)
        : startAbsTime_us(dStartAbsTime_us)
        , duration_us(dDuration_us)
        , time(vecTime)
        , amplitude(vecAmplitude)
        , event(gradEvent)
    {}
};

class PulseqLoader : public QObject
{
    Q_OBJECT
public:
    explicit PulseqLoader(QObject *parent = nullptr);
    inline void SetPulseqFile(const QString& filePath) { m_sFilePath = filePath; }
    inline void SetSequence(std::shared_ptr<ExternalSequence>& seq) { m_spPulseqSeq = seq; }

public slots:
    void process();

signals:
    void processingStarted();
    void errorOccurred(const QString& error);
    void progressUpdated(uint64_t progress);
    void versionLoaded(int version);
    void loadingCompleted(const SeqInfo& seqInfo,
                          const QVector<SeqBlock*>& blocks,
                          const QMap<int, QVector<float>>& shapeLib,
                          const QVector<RfInfo>& rfLib,
                          const RfTimeWaveShapeMap& rfMagShapeLib,
                          QVector<GradTrapInfo> gzLib
                          );
    void finished();

private:
    QString                                     m_sFilePath;
    std::shared_ptr<ExternalSequence>           m_spPulseqSeq;
    QVector<SeqBlock*>                          m_vecSeqBlock;
    SeqInfo                                     m_stSeqInfo;
    QMap<int, QVector<float>>                   m_mapShapeLib;
    RfTimeWaveShapeMap                          m_mapRfMagShapeLib;
    QVector<RfInfo>                             m_vecRfLib;
    QVector<GradTrapInfo>                       m_vecGzLib;

private:
    bool LoadPulseqEvents();
};

#endif // PULSEQ_LOADER_H
