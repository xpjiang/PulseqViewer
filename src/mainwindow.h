#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProgressBar>
#include <QLabel>
#include <qcustomplot.h>

#include <ExternalSequence.h>


#define SAFE_DELETE(p) { if(p) { delete p; p = nullptr; } }

namespace Ui {
class MainWindow;
}

struct RfInfo
{
    double startAbsTime_us;
    double duration_us;
    uint16_t samples;
    float dwell;
    const RFEvent* event;

    RfInfo(const double& dStartAbsTime_us, const double& dDuration_us, const uint16_t ushSamples, const float& fDwell, const RFEvent* pRfEvent)
        : startAbsTime_us(dStartAbsTime_us)
        , duration_us(dDuration_us)
        , samples(ushSamples)
        , dwell(fDwell)
        , event(pRfEvent)
    {}
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

    static const int MAX_RECENT_FILES = 10;

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void Init();
    void InitSlots();
    void InitStatusBar();
    void InitSequenceFigure();
    void UpdatePlotRange(const double& x1, const double& x2);

    // Slots-File
    void OpenPulseqFile();
    void ReOpenPulseqFile();

    // Slots-View
    void ResetView();

    // Slots-Interaction
    void onMousePress(QMouseEvent* event);
    void onMouseMove(QMouseEvent* event);
    void onMouseRelease(QMouseEvent* event);
    void DrawRFWaveform(const double& dStartTime = 0, double dEndTime = -1);
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

    // Pulseq
    void ClearPulseqCache();
    bool LoadPulseqFile(const QString& sPulseqFilePath);
    bool ClosePulseqFile();
    bool LoadPulseqEvents();
    //bool IsBlockRf(const float* fAmp, const float* fPhase, const int& iSamples);

private:
    Ui::MainWindow                       *ui;

    QLabel                               *m_pVersionLabel;
    QProgressBar                         *m_pProgressBar;

    // Pulseq
    QString                              m_sPulseqFilePath;
    QString                              m_sPulseqFilePathCache;
    QStringList                          m_listRecentPulseqFilePaths;
    std::shared_ptr<ExternalSequence>    m_spPulseqSeq;
    std::vector<SeqBlock*>               m_vecSeqBlocks;
    double                               m_dTotalDuration_us;

    QMap<int, std::vector<float>>        m_mapShapeLib;
    // RF
    uint64_t                             m_lRfNum;
    QVector<RfInfo>                      m_vecRfLib;

    // GZ
    uint64_t                             m_lGzNum;
    QVector<GradEvent>                   m_vecGzLib;

    // GY
    uint64_t                             m_lGyNum;
    QVector<GradEvent>                   m_vecGyLib;

    // GX
    uint64_t                             m_lGxNum;
    QVector<GradEvent>                   m_vecGxLib;

    // ADC
    uint64_t                             m_lAdcNum;
    QVector<ADCEvent>                    m_vecAdcLib;

    // Plot
    QVector<QCPGraph*>                   m_vecRfGraphs;

    QCPAxisRect*                         m_pRfRect;
    QCPAxisRect*                         m_pGzRect;
    QCPAxisRect*                         m_pGyRect;
    QCPAxisRect*                         m_pGxRect;
    QCPAxisRect*                         m_pAdcRect;

    // Interaction
    bool                                 m_bIsSelecting;
    bool                                 m_bIsDragging;
    QPoint                               m_objSelectStartPos;
    QCPItemRect*                         m_pSelectionRect;
    QPoint                               m_objDragStartPos;       // 记录拖拽起始位置
    double                               m_dDragStartRange;
};

#endif // MAINWINDOW_H
