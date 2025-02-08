#ifndef PULSEQ_LOADER_H
#define PULSEQ_LOADER_H

#include <QObject>
#include <QMap>
#include <ExternalSequence.h>

typedef QMap<QPair<int, int>, QVector<double>> RfTimeWaveShapeMap;

struct SeqInfo
{
    double totalDuration_us;
    double rfMaxAmp_Hz;
    double rfMinAmp_Hz;

    SeqInfo()
        : totalDuration_us(0.)
        , rfMaxAmp_Hz(0.)
        , rfMinAmp_Hz(0.)
    {}
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
        const RFEvent* pRfEvent)
        : startAbsTime_us(dStartAbsTime_us)
        , duration_us(dDuration_us)
        , samples(ushSamples)
        , dwell(fDwell)
        , event(pRfEvent)
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
                          const RfTimeWaveShapeMap& rfMagShapeLib
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
    int64_t                                     m_lRfNum;

private:
    bool LoadPulseqEvents();
};

#endif // PULSEQ_LOADER_H
