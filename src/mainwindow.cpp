#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <complex>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_sPulseqFilePath("")
    , m_sPulseqFilePathCache("")
    , m_spPulseqSeq(std::make_shared<ExternalSequence>())
    , m_lRfNum(0)
    , m_bIsSelecting(false)
    , m_bIsDragging(false)
    , m_dTotalDuration_us(0.)
    , m_dDragStartRange(0.)
{
    ui->setupUi(this);
    setAcceptDrops(true);
    Init();

    m_listRecentPulseqFilePaths.resize(10);

    m_pSelectionRect = new QCPItemRect(ui->customPlot);
    m_pSelectionRect->setVisible(false);
    m_pSelectionRect->setBrush(QBrush(QColor(128, 128, 128, 128)));
    m_pSelectionRect->setPen(QPen(Qt::black, 1));
}

MainWindow::~MainWindow()
{
    ClearPulseqCache();
    delete ui;
    SAFE_DELETE(m_pVersionLabel);
    SAFE_DELETE(m_pProgressBar);
}

void MainWindow::Init()
{
    InitSlots();
    InitStatusBar();
    InitSequenceFigure();
}

void MainWindow::InitSlots()
{
    // File
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::OpenPulseqFile);
    connect(ui->actionReopen, &QAction::triggered, this, &MainWindow::ReOpenPulseqFile);
    connect(ui->actionCloseFile, &QAction::triggered, this, &MainWindow::ClosePulseqFile);

    // View
    connect(ui->actionResetView, &QAction::triggered, this, &MainWindow::ResetView);

    // Interaction
    connect(ui->customPlot, &QCustomPlot::mousePress, this, &MainWindow::onMousePress);
    connect(ui->customPlot, &QCustomPlot::mouseMove, this, &MainWindow::onMouseMove);
    connect(ui->customPlot, &QCustomPlot::mouseRelease, this, &MainWindow::onMouseRelease);
}

void MainWindow::InitStatusBar()
{
    m_pVersionLabel= new QLabel(this);
    ui->statusbar->addWidget(m_pVersionLabel);

    m_pProgressBar = new QProgressBar(this);
    m_pProgressBar->setMaximumWidth(200);
    m_pProgressBar->setMinimumWidth(200);
    m_pProgressBar->hide();
    m_pProgressBar->setRange(0, 100);
    m_pProgressBar->setValue(0);
    ui->statusbar->addWidget(m_pProgressBar);
}

void MainWindow::InitSequenceFigure()
{
    ui->customPlot->plotLayout()->clear();

    ui->customPlot->setAntialiasedElements(QCP::aeAll);
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    m_pRfRect = new QCPAxisRect(ui->customPlot);
    m_pGzRect = new QCPAxisRect(ui->customPlot);
    m_pGyRect = new QCPAxisRect(ui->customPlot);
    m_pGxRect = new QCPAxisRect(ui->customPlot);
    m_pAdcRect = new QCPAxisRect(ui->customPlot);

    ui->customPlot->plotLayout()->addElement(0, 0, m_pRfRect);
    ui->customPlot->plotLayout()->addElement(1, 0, m_pGzRect);
    ui->customPlot->plotLayout()->addElement(2, 0, m_pGyRect);
    ui->customPlot->plotLayout()->addElement(3, 0, m_pGxRect);
    ui->customPlot->plotLayout()->addElement(4, 0, m_pAdcRect);

    m_pRfRect->setupFullAxesBox(true);
    m_pRfRect->axis(QCPAxis::atLeft)->setLabel("RF (Hz)");
    m_pGzRect->setupFullAxesBox(true);
    m_pGzRect->axis(QCPAxis::atLeft)->setLabel("GZ (Hz/m)");
    m_pGyRect->setupFullAxesBox(true);
    m_pGyRect->axis(QCPAxis::atLeft)->setLabel("GY (Hz/m)");
    m_pGxRect->setupFullAxesBox(true);
    m_pGxRect->axis(QCPAxis::atLeft)->setLabel("GX (Hz/m)");
    m_pAdcRect->setupFullAxesBox(true);
    m_pAdcRect->axis(QCPAxis::atLeft)->setLabel("ADC");

    // share the same time axis
    m_pRfRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
    m_pGzRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
    m_pGyRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
    m_pGxRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
    m_pAdcRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");

    // Hide all time axis but the last one
    m_pRfRect->axis(QCPAxis::atBottom)->setVisible(false);
    m_pGzRect->axis(QCPAxis::atBottom)->setVisible(false);
    m_pGyRect->axis(QCPAxis::atBottom)->setVisible(false);
    m_pGxRect->axis(QCPAxis::atBottom)->setVisible(false);
}

void MainWindow::OpenPulseqFile()
{
    m_sPulseqFilePath = QFileDialog::getOpenFileName(
        this,
        "Selec a Pulseq File",                           // Dialog title
        QDir::currentPath(),                    // Default open folder
        "Text Files (*.seq);;All Files (*)"  // File filter
        );

    if (!m_sPulseqFilePath.isEmpty())
    {
        if (!LoadPulseqFile(m_sPulseqFilePath))
        {
            m_sPulseqFilePath.clear();
            std::cout << "LoadPulseqFile failed!\n";
        }
        m_sPulseqFilePathCache = m_sPulseqFilePath;
    }
}

void MainWindow::ReOpenPulseqFile()
{
    if (m_sPulseqFilePathCache.size() > 0)
    {
        ClearPulseqCache();
        LoadPulseqFile(m_sPulseqFilePathCache);
    }
}

void MainWindow::ResetView()
{
    if (m_sPulseqFilePathCache.isEmpty()) return;
    DrawRFWaveform(0, -1);
}

void MainWindow::ClearPulseqCache()
{
    m_pVersionLabel->setText("");
    m_pVersionLabel->setVisible(false);
    m_pProgressBar->hide();

    if (NULL != ui->customPlot)  // 先检查 customPlot 是否有效
    {
        ui->customPlot->clearGraphs();
        m_vecRfGraphs.clear();
        ui->customPlot->replot();
    }

    m_dTotalDuration_us = 0.;
    m_lRfNum = 0;

    m_mapShapeLib.clear();
    m_vecRfLib.clear();
    if (nullptr != m_spPulseqSeq.get())
    {
        m_spPulseqSeq->reset();
        m_spPulseqSeq.reset(new ExternalSequence);
        for (uint16_t ushBlockIndex=0; ushBlockIndex < m_vecSeqBlocks.size(); ushBlockIndex++)
        {
            SAFE_DELETE(m_vecSeqBlocks[ushBlockIndex]);
        }
        m_vecSeqBlocks.clear();
        std::cout << m_sPulseqFilePath.toStdString() << " Closed\n";
    }
    this->setWindowFilePath("");
}

bool MainWindow::LoadPulseqFile(const QString& sPulseqFilePath)
{
    this->setEnabled(false);
    ClearPulseqCache();
    if (!m_spPulseqSeq->load(sPulseqFilePath.toStdString()))
    {
        this->setEnabled(false);
        std::stringstream sLog;
        sLog << "Load " << sPulseqFilePath.toStdString() << " failed!";
        std::cout << sLog.str() << "\n";
        QMessageBox::critical(this, "File Error", sLog.str().c_str());
        this->setEnabled(true);
        return false;
    }
    this->setWindowFilePath(sPulseqFilePath);

    const int& shVersion = m_spPulseqSeq->GetVersion();
    m_pVersionLabel->setVisible(true);
    const int& shVersionMajor = shVersion / 1000000L;
    const int& shVersionMinor = (shVersion / 1000L) % 1000L;
    const int& shVersionRevision = shVersion % 1000L;
    QString sVersion = QString::number(shVersionMajor) + "." + QString::number(shVersionMinor) + "." + QString::number(shVersionRevision);
    m_pVersionLabel->setText("Pulseq Version: v" + sVersion);

    const int64_t& lSeqBlockNum = m_spPulseqSeq->GetNumberOfBlocks();
    std::cout << lSeqBlockNum << " blocks detected!\n";
    m_vecSeqBlocks.resize(lSeqBlockNum);
    m_pProgressBar->show();
    uint8_t progress(0);
    for (uint16_t ushBlockIndex=0; ushBlockIndex < lSeqBlockNum; ushBlockIndex++)
    {
        m_vecSeqBlocks[ushBlockIndex] = m_spPulseqSeq->GetBlock(ushBlockIndex);
        if (!m_spPulseqSeq->decodeBlock(m_vecSeqBlocks[ushBlockIndex]))
        {
            std::stringstream sLog;
            sLog << "Decode SeqBlock failed, block index: " << ushBlockIndex;
            QMessageBox::critical(this, "File Error", sLog.str().c_str());
            ClearPulseqCache();
            return false;
        }
        if (m_vecSeqBlocks[ushBlockIndex]->isRF()) { m_lRfNum += 1; }

        progress = ushBlockIndex * 100 / lSeqBlockNum;
        m_pProgressBar->setValue(progress);
    }


    m_vecRfLib.reserve(m_lRfNum);

    if (!LoadPulseqEvents())
    {
        std::cout << "LoadPulseqEvents failed!\n";
        QMessageBox::critical(this, "Pulseq Events Error", "LoadPulseqEvents failed!");
        return false;
    }

    m_pProgressBar->setValue(100);
    this->setEnabled(true);
    return true;
}

bool MainWindow::ClosePulseqFile()
{
    ClearPulseqCache();

    return true;
}

bool MainWindow::LoadPulseqEvents()
{
    if (m_vecSeqBlocks.size() == 0) return true;
    m_dTotalDuration_us = 0.;
    double dCurrentStartTime_us(0.);
    for (const auto& pSeqBlock : m_vecSeqBlocks)
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
                std::vector<float> vecAmp(ushSamples+2, 0.f);
                vecAmp[0] =0;
                vecAmp[ushSamples+1] = std::numeric_limits<double>::quiet_NaN();
                const float* fAmp = pSeqBlock->GetRFAmplitudePtr();
                for (int index = 0; index < ushSamples; index++)
                {
                    vecAmp[index] = fAmp[index];
                }
                m_mapShapeLib.insert(magShapeID, vecAmp);
            }

            const int& phaseShapeID = rfEvent.phaseShape;
            if (!m_mapShapeLib.contains(phaseShapeID))
            {
                std::vector<float> vecPhase(ushSamples+2, 0.f);
                vecPhase[0] = std::numeric_limits<double>::quiet_NaN();
                vecPhase[ushSamples+1] = std::numeric_limits<double>::quiet_NaN();

                const float* fPhase = pSeqBlock->GetRFPhasePtr();
                for (int index = 0; index < ushSamples; index++)
                {
                    vecPhase[index] = fPhase[index];
                }
                m_mapShapeLib.insert(phaseShapeID, vecPhase);
            }

        }
        dCurrentStartTime_us += pSeqBlock->GetDuration();
        m_dTotalDuration_us += pSeqBlock->GetDuration();
    }

    std::cout << m_vecRfLib.size() << " RF events detetced!\n";
    DrawRFWaveform(0, -1);

    return true;
}

void MainWindow::onMousePress(QMouseEvent *event)
{
    if (m_vecSeqBlocks.size() == 0) return;
    if (event->button() == Qt::LeftButton)
    {
        ui->customPlot->setInteractions(QCP::Interactions());
        m_bIsSelecting = true;
        m_objSelectStartPos = event->pos();

        // 初始化选择框
        double x = ui->customPlot->xAxis->pixelToCoord(m_objSelectStartPos.x());
        double y = ui->customPlot->yAxis->pixelToCoord(m_objSelectStartPos.y());
        m_pSelectionRect->topLeft->setCoords(x, y);
        m_pSelectionRect->bottomRight->setCoords(x, y);
        m_pSelectionRect->setVisible(true);

        ui->customPlot->replot();
    }
    else if (event->button() == Qt::RightButton)
    {
        m_bIsDragging = true;
        setCursor(Qt::ClosedHandCursor);
        m_objDragStartPos = event->pos();
        m_dDragStartRange = ui->customPlot->xAxis->range().lower;
    }
}

void MainWindow::onMouseMove(QMouseEvent *event)
{
    if (m_vecSeqBlocks.size() == 0) return;
    if(m_bIsSelecting)
    {
        double yMin = ui->customPlot->yAxis->range().lower;
        double yMax = ui->customPlot->yAxis->range().upper;
        // 更新选择框
        double x1 = ui->customPlot->xAxis->pixelToCoord(m_objSelectStartPos.x());
        double y1 = ui->customPlot->yAxis->pixelToCoord(m_objSelectStartPos.y());
        double x2 = ui->customPlot->xAxis->pixelToCoord(event->pos().x());
        double y2 = ui->customPlot->yAxis->pixelToCoord(event->pos().y());

        // 限制 y 值在有效范围内
        y1 = qBound(yMin, y1, yMax);
        y2 = qBound(yMin, y2, yMax);

        m_pSelectionRect->topLeft->setCoords(qMin(x1, x2), qMax(y1, y2));
        m_pSelectionRect->bottomRight->setCoords(qMax(x1, x2), qMin(y1, y2));

        ui->customPlot->replot();
    }
    else if (m_bIsDragging)
    {
        int pixelDx = event->pos().x() - m_objDragStartPos.x();
        double dx = ui->customPlot->xAxis->pixelToCoord(pixelDx) - ui->customPlot->xAxis->pixelToCoord(0);

        const QCPRange& xRange = ui->customPlot->xAxis->range();
        double xSpan = xRange.size();

        double newX1 = m_dDragStartRange - dx;
        double newX2 = newX1 + xSpan;

        newX1 = newX1 < 0 ? 0 : newX1;
        newX2 = newX2 > m_dTotalDuration_us ? m_dTotalDuration_us : newX2;
        DrawRFWaveform(newX1, newX2);
    }
}

void MainWindow::onMouseRelease(QMouseEvent *event)
{
    if (m_vecSeqBlocks.size() == 0) return;
    if (m_vecRfLib.size() == 0) return;

    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    if(event->button() == Qt::LeftButton && m_bIsSelecting)
    {
        m_bIsSelecting = false;
        m_pSelectionRect->setVisible(false);

        // 获取选择的时间范围
        double x1 = ui->customPlot->xAxis->pixelToCoord(m_objSelectStartPos.x());
        double x2 = ui->customPlot->xAxis->pixelToCoord(event->pos().x());

        // 如果选择范围太小，认为是点击事件，不进行缩放
        if(qAbs(x2 - x1) > 5.0) 
        {  
            if (x2 > x1)
            {
                ui->customPlot->xAxis->setRange(x1, x2);
                DrawRFWaveform(x1, x2);
            }
            else
            {
                QCPRange currentRange = ui->customPlot->xAxis->range();
                double center = (currentRange.lower + currentRange.upper) / 2;
                double newSpan = currentRange.size() * 3;  // 可以调整这个倍数

                double x1New = center - newSpan / 2;
                double x2New = center + newSpan / 2;

                x1New = x1New < 0 ? 0 : x1New;
                x2New = x2New > m_dTotalDuration_us ? m_dTotalDuration_us : x2New;
                DrawRFWaveform(x1New, x2New);
            }
        }
    }
    else if (event->button() == Qt::RightButton && m_bIsDragging)
    {
        m_bIsDragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void MainWindow::DrawRFWaveform(const double& dStartTime, double dEndTime)
{
    // @TODO: fix time range, add reset. fix right click
    if (m_vecSeqBlocks.size() == 0) return;
    if (m_vecRfLib.size() == 0) return;
    if(dEndTime < 0) dEndTime = m_dTotalDuration_us;

    double dMaxAmp(0.);
    double dMinAmp(0.);
    for(const auto& rfInfo : m_vecRfLib)
    {
        const auto& rf = rfInfo.event;
        const std::vector<float>& vecAmp = m_mapShapeLib[rf->magShape];
        const std::vector<float>& vecPhase = m_mapShapeLib[rf->phaseShape];

        // 添加波形数据点
        double signal(0.);
        for(uint32_t index = 0; index < vecAmp.size(); index++)
        {
            const float& amp = vecAmp[index];
            const float& phase = vecPhase[index];
            signal = std::abs(std::polar(amp, phase)) * rfInfo.event->amplitude;
            dMaxAmp = std::max(dMaxAmp, signal);
            dMinAmp = std::min(dMinAmp, signal);
        }
    }
    double margin = (dMaxAmp - dMinAmp) * 0.1;
    m_pRfRect->axis(QCPAxis::atLeft)->setRange(dMinAmp - margin, dMaxAmp+ margin);
    QCPRange newRange(dStartTime, dEndTime);
    m_pRfRect->axis(QCPAxis::atBottom)->setRange(newRange);
    m_pAdcRect->axis(QCPAxis::atBottom)->setRange(newRange);

    // 只处理时间范围内的RF
    for(const auto& rfInfo : m_vecRfLib)
    {
        // 跳过范围外的RF
        if(rfInfo.startAbsTime_us + rfInfo.duration_us < dStartTime) continue;
        if(rfInfo.startAbsTime_us > dEndTime) break;

        QCPGraph* rfGraph = ui->customPlot->addGraph(m_pRfRect->axis(QCPAxis::atBottom),
                                                     m_pRfRect->axis(QCPAxis::atLeft));
        m_vecRfGraphs.append(rfGraph);

        const auto& rf = rfInfo.event;
        double sampleTime = rfInfo.startAbsTime_us;
        const std::vector<float>& vecAmp = m_mapShapeLib[rf->magShape];
        const std::vector<float>& vecPhase = m_mapShapeLib[rf->phaseShape];

        QVector<double> timePoints(vecAmp.size(), 0.);
        QVector<double> amplitudes(vecAmp.size(), 0.);

        // 添加波形数据点
        double signal(0.);
        for(uint32_t index = 0; index < vecAmp.size(); index++)
        {
            const float& amp = vecAmp[index];
            const float& phase = vecPhase[index];
            signal = std::abs(std::polar(amp, phase)) * rfInfo.event->amplitude;
            timePoints[index] = sampleTime;
            amplitudes[index] = signal;
            sampleTime += rfInfo.dwell;
        }
        rfGraph->setData(timePoints, amplitudes);
        rfGraph->setPen(QPen(Qt::blue));
        rfGraph->setSelectable(QCP::stWhole);
    }

    ui->customPlot->xAxis->setRange(dStartTime, dEndTime);
    ui->customPlot->replot();
}

// 拖拽进入事件
void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
}

// 拖拽放下事件
void MainWindow::dropEvent(QDropEvent *event)
{
    const QMimeData* mimeData = event->mimeData();
    // 获取拖拽的文件URL列表
    if (mimeData->hasUrls()) {
        QList<QUrl> urlList = mimeData->urls();
        // 获取第一个文件的本地路径
        m_sPulseqFilePath = urlList.at(0).toLocalFile();
        // 调用你的文件加载函数
        if (!LoadPulseqFile(m_sPulseqFilePath))
        {
            std::stringstream sLog;
            sLog << "Load " << m_sPulseqFilePath.toStdString() << " failed!";
            QMessageBox::critical(this, "File Error", sLog.str().c_str());
            return;
        }
        m_sPulseqFilePathCache = m_sPulseqFilePath;
    }
}


