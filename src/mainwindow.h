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

    // Slots-File
    void OpenPulseqFile();
    void ReOpenPulseqFile();

    // Slots-View
    void ResetView();

    // Slots-Interaction
    void DrawRFWaveform(const double& dStartTime = 0, double dEndTime = -1);
    void DrawADCWaveform(const double& dStartTime = 0, double dEndTime = -1);
    void DrawGWaveform(const double& dStartTime = 0, double dEndTime = -1);
	void DrawBlockEdges();

    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;

    // Pulseq
    void ClearPulseqCache();
    bool LoadPulseqFile(const QString& sPulseqFilePath);
    bool ClosePulseqFile();
    bool IsBlockRf(const float* fAmp, const float* fPhase, const int& iSamples);

    std::tuple<double, size_t> calcRfCenter(const std::vector<double>& signal, const std::vector<double>& t);

private:
    Ui::MainWindow                       *ui;

    QLabel                               *m_pVersionLabel;
    QProgressBar                         *m_pProgressBar;

    QVector<QColor> colors; // same as matlab for plot

    // Pulseq
    QString                              m_sPulseqFilePath;
    QString                              m_sPulseqFilePathCache;
    QStringList                          m_listRecentPulseqFilePaths;
    std::shared_ptr<ExternalSequence>    m_spPulseqSeq;
    std::vector<SeqBlock*>               m_vecDecodeSeqBlocks;
    double                               m_dTotalDuration_us;

    // Plot rect
	QVector<QCPAxisRect*>                m_vecRects; // 0: ADC label, 1: RF Mag, 2: RF ADC phase, 3: Gx, 4: Gy, 5: Gz

    QCPAxisRect*                         m_pADCLabelsRect;
    QCPAxisRect*                         m_pRfMagRect;
	QCPAxisRect*                         m_pRfADCPhaseRect;
    QCPAxisRect*                         m_pGxRect;
    QCPAxisRect*                         m_pGyRect;
    QCPAxisRect*                         m_pGzRect;

    //
    int nBlockRangeStart; // ui
	int nBlockRangeEnd; // ui
    bool bShowBlocksEdges; // ui
	QVector<double> vecBlockEdges; // ui
	QVector<QString> validTimeUnits = { "s","ms","us" }; // ui
	QVector<double> tFactorList = { 1e-6, 1e-3, 1 }; // ui
	QString TimeUnits; // ui
	double tFactor; // ui


    // Interaction
    bool                                 m_bIsSelecting;
    QPoint                               m_objSelectStartPos;
    QCPItemRect*                         m_pSelectionRect;
    QPoint                               m_objDragStartPos;       // 记录拖拽起始位置
    double                               m_dDragStartRange;

private slots:
    void synchronizeXAxes(const QCPRange& newRange);
};

#endif // MAINWINDOW_H
