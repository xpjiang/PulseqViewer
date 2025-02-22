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

class EventBlockInfoDialog : public QDialog 
{
public:
    explicit EventBlockInfoDialog(QWidget* parent = nullptr) : QDialog(parent)
    {
        // 设置窗口属性
        setWindowTitle("Event Block");
        setWindowModality(Qt::NonModal); // 设置为非模态对话框
        resize(600, 200);                // 设置初始尺寸

        // 创建文本框控件
        textEdit = new QPlainTextEdit(this);
        textEdit->setReadOnly(true);     // 设置为只读模式
        textEdit->setWordWrapMode(QTextOption::NoWrap); // 禁用自动换行

        // 创建布局并添加控件
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setContentsMargins(2, 2, 2, 2); // 紧凑边距
        layout->addWidget(textEdit);

        // 构造显示内容（示例数据）
        //QString content =
        //    "Event Block No.: 0\n"
        //    "Event Block Start Time: 1000 [us]\n"
        //    "Event Block Duration: 20000 [us]\n"
        //    "Y-Value: 10.9\n"
        //    "Y-Unit: Volt\n"
        //    "Max/Min value: 10.996/0\n"
        //    "X Gradient: 0 mT/m\n"
        //    "Y Gradient: 0 mT/m\n"
        //    "Z Gradient: -4.95443 mT/m";

        //textEdit->setPlainText(content);
    }
	void setInfoContent(QString content)
	{
		textEdit->setPlainText(content);
	}
private:
	QPlainTextEdit* textEdit;
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

    void onMouseMove(QMouseEvent* event);

    // Pulseq
    void ClearPulseqCache();
    bool LoadPulseqFile(const QString& sPulseqFilePath);
    bool ClosePulseqFile();
    bool IsBlockRf(const float* fAmp, const float* fPhase, const int& iSamples);

    std::tuple<double, size_t> calcRfCenter(const std::vector<double>& signal, const std::vector<double>& t);

private:
    Ui::MainWindow                       *ui;
    QPoint m_rightClickPos;
    EventBlockInfoDialog* m_pBlockInfoDialog;
    

    QLabel                               *m_pVersionLabel;
    QProgressBar                         *m_pProgressBar;
    QLabel* m_pCoordLabel; // 添加用于显示坐标的 QLabel

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
    QVector<QCPItemTracer*> m_vecTracers; // 添加数据光标成员变量
    QVector <QCPItemStraightLine*> m_vecVerticalLine; // 添加垂直线成员变量
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
    void showContextMenu(const QPoint& pos);
    void showBlockInformation();
};

#endif // MAINWINDOW_H
