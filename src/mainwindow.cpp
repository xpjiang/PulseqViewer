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
    , m_listAxis({"RF", "GZ", "GY", "GX", "ADC"})
    , m_pSelectedGraph(nullptr)
{
    ui->setupUi(this);
    setAcceptDrops(true);
    this->setWindowTitle(BASIC_WIN_TITLE);
    Init();

    m_mapAxisAction = {
        {"RF", ui->actionRF},
        {"GZ", ui->actionGZ},
        {"GY", ui->actionGY},
        {"GX", ui->actionGX},
        {"ADC", ui->actionADC},
        };

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
    InitStatusBar();
    InitSequenceFigure();
    InitSlots();
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

    uint8_t index(0);
    for (auto& axis : m_listAxis)
    {
        index = m_listAxis.indexOf(axis);
        m_mapRect[axis] = new QCPAxisRect(ui->customPlot);
        m_mapRect[axis]->axis(QCPAxis::atBottom)->setLabel("Time (us)");
        ui->customPlot->plotLayout()->addElement(index, 0, m_mapRect[axis]);
    }

    m_mapRect["RF"]->axis(QCPAxis::atLeft)->setLabel("RF (Hz)");
    m_mapRect["GZ"]->axis(QCPAxis::atLeft)->setLabel("GZ (Hz/m)");
    m_mapRect["GY"]->axis(QCPAxis::atLeft)->setLabel("GY (Hz/m)");
    m_mapRect["GX"]->axis(QCPAxis::atLeft)->setLabel("GX (Hz/m)");
    m_mapRect["ADC"]->axis(QCPAxis::atLeft)->setLabel("ADC");

    QMargins margins(70, 10, 10, 10);
    QFont labelFont;
    labelFont.setWeight(QFont::DemiBold);
    foreach (auto& rect, m_mapRect)
    {
        rect->setMinimumMargins(margins);
        rect->setRangeDrag(Qt::Horizontal);
        rect->setRangeZoom(Qt::Horizontal);
        rect->setupFullAxesBox(true);

        rect->axis(QCPAxis::atLeft)->setLabelPadding(10);
        rect->axis(QCPAxis::atLeft)->setLabelFont(labelFont);
    }

    // Hide all time axis but the last one
    UpdateAxisVisibility();

    m_mapRect["ADC"]->axis(QCPAxis::atLeft)->setRange(0, 1.3);
}

void MainWindow::InitSlots()
{
    // File
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::SlotOpenPulseqFile);
    connect(ui->actionReopen, &QAction::triggered, this, &MainWindow::SlotReOpenPulseqFile);
    connect(ui->actionCloseFile, &QAction::triggered, this, &MainWindow::ClosePulseqFile);

    // View
    connect(ui->actionEnableAxisToolbar, &QAction::triggered, this, &MainWindow::SlotEnableAxisToolbar);
    connect(ui->actionRF, &QAction::triggered, this, &MainWindow::SlotEnableRFAxis);
    connect(ui->actionGZ, &QAction::triggered, this, &MainWindow::SlotEnableGZAxis);
    connect(ui->actionGY, &QAction::triggered, this, &MainWindow::SlotEnableGYAxis);
    connect(ui->actionGX, &QAction::triggered, this, &MainWindow::SlotEnableGXAxis);
    connect(ui->actionADC, &QAction::triggered, this, &MainWindow::SlotEnableADCAxis);
    connect(ui->actionTrigger, &QAction::triggered, this, &MainWindow::SlotEnableTriggerAxis);

    connect(ui->actionResetView, &QAction::triggered, this, &MainWindow::SlotResetView);

    // Analysis
    connect(ui->actionExportData, &QAction::triggered, this, &MainWindow::SlotExportData);

    // Interaction
    connect(ui->customPlot, &QCustomPlot::mousePress, this, &MainWindow::onMousePress);
    connect(ui->customPlot, &QCustomPlot::mouseMove, this, &MainWindow::onMouseMove);
    connect(ui->customPlot, &QCustomPlot::mouseRelease, this, &MainWindow::onMouseRelease);
    connect(ui->customPlot, &QCustomPlot::plottableClick, this, &MainWindow::onPlottableClick);


    foreach (auto& rect1, m_mapRect)
    {
        if (rect1)
        {
            connect(rect1->axis(QCPAxis::atBottom),
                    QOverload<const QCPRange&>::of(&QCPAxis::rangeChanged),
                    this, &MainWindow::onAxisRangeChanged);
        }

        foreach (auto& rect2, m_mapRect)
        {
            if (rect1 != rect2)
            {
                connect(rect1->axis(QCPAxis::atBottom), QOverload<const QCPRange&>::of(&QCPAxis::rangeChanged),
                        rect2->axis(QCPAxis::atBottom), QOverload<const QCPRange&>::of(&QCPAxis::setRange));
            }
        }
    }
}

void MainWindow::UpdatePlotRange(const double& x1, const double& x2)
{
    ui->customPlot->xAxis->setRange(x1, x2);
    ui->customPlot->replot();
}

void MainWindow::RestoreViewLayout()
{
    foreach (auto& rect, m_mapRect)
    {
        ui->customPlot->plotLayout()->take(rect);
    }

    uint16_t index(0);
    for (auto& axis : m_listAxis)
    {
        if (m_mapAxisAction[axis]->isChecked())
        {
            ui->customPlot->plotLayout()->addElement(index, 0, m_mapRect[axis]);
            index += 1;
        }
    }
}

void MainWindow::UpdateAxisVisibility()
{
    bool bAxisVisible(false);
    for(auto it = m_listAxis.rbegin(); it != m_listAxis.rend(); ++it)
    {
        QString& axis = *it;
        bool bIsRectVis(m_mapRect[axis]->visible());
        if (bIsRectVis && !bAxisVisible)
        {
            bAxisVisible = true;
            m_mapRect[axis]->axis(QCPAxis::atBottom)->setVisible(bAxisVisible);
        }
        else
        {
            m_mapRect[axis]->axis(QCPAxis::atBottom)->setVisible(false);
        }
    }
}

void MainWindow::SlotOpenPulseqFile()
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

void MainWindow::SlotReOpenPulseqFile()
{
    if (m_sPulseqFilePathCache.size() > 0)
    {
        ClearPulseqCache();
        LoadPulseqFile(m_sPulseqFilePathCache);
    }
}

void MainWindow::SlotEnableAxisToolbar()
{
    const bool& isChecked = ui->actionEnableAxisToolbar->isChecked();
    
}

void MainWindow::SlotEnableRFAxis()
{
    const bool& isChecked = ui->actionRF->isChecked();
    m_mapRect["RF"]->setVisible(isChecked);
    if (isChecked)
    {
        RestoreViewLayout();
    }
    else
    {
        ui->customPlot->plotLayout()->take(m_mapRect["RF"]);
    }
    UpdateAxisVisibility();
    ui->customPlot->plotLayout()->simplify();
    ui->customPlot->replot();
}

void MainWindow::SlotEnableGZAxis()
{
    const bool& isChecked = ui->actionGZ->isChecked();
    m_mapRect["GZ"]->setVisible(isChecked);
    if (isChecked)
    {
        RestoreViewLayout();
    }
    else
    {
        ui->customPlot->plotLayout()->take(m_mapRect["GZ"]);
    }
    UpdateAxisVisibility();
    ui->customPlot->plotLayout()->simplify();
    ui->customPlot->replot();
}

void MainWindow::SlotEnableGYAxis()
{
    const bool& isChecked = ui->actionGY->isChecked();
    m_mapRect["GY"]->setVisible(isChecked);
    if (isChecked)
    {
        RestoreViewLayout();
    }
    else
    {
        ui->customPlot->plotLayout()->take(m_mapRect["GY"]);
    }
    UpdateAxisVisibility();
    ui->customPlot->plotLayout()->simplify();
    ui->customPlot->replot();
}

void MainWindow::SlotEnableGXAxis()
{
    const bool& isChecked = ui->actionGX->isChecked();
    m_mapRect["GX"]->setVisible(isChecked);
    if (isChecked)
    {
        RestoreViewLayout();
    }
    else
    {
        ui->customPlot->plotLayout()->take(m_mapRect["GX"]);
    }
    UpdateAxisVisibility();
    ui->customPlot->plotLayout()->simplify();
    ui->customPlot->replot();
}

void MainWindow::SlotEnableADCAxis()
{
    const bool& isChecked = ui->actionADC->isChecked();
    m_mapRect["ADC"]->setVisible(isChecked);
    if (isChecked)
    {
        RestoreViewLayout();
    }
    else
    {
        ui->customPlot->plotLayout()->take(m_mapRect["ADC"]);
    }
    UpdateAxisVisibility();
    ui->customPlot->plotLayout()->simplify();
    ui->customPlot->replot();
}

void MainWindow::SlotEnableTriggerAxis()
{
}

void MainWindow::SlotResetView()
{
    if (m_sPulseqFilePathCache.isEmpty()) return;
    UpdatePlotRange(0, m_dTotalDuration_us);
}

void MainWindow::ClearPulseqCache()
{
    m_pVersionLabel->setText("");
    m_pVersionLabel->setVisible(false);
    m_pProgressBar->hide();
    m_pSelectedGraph = nullptr;
    if (NULL != ui->customPlot)
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
    this->setWindowTitle(QString(BASIC_WIN_TITLE));
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
    uint32_t progress(0);
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
    this->setWindowTitle(QString(BASIC_WIN_TITLE) + QString(": ") + sPulseqFilePath);
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
            const bool& bIsBlock = IsBlockRf(pSeqBlock->GetRFAmplitudePtr(), pSeqBlock->GetRFPhasePtr(), ushSamples);
            RfInfo rfInfo(dCurrentStartTime_us+rfEvent.delay, dDuration_us, ushSamples, fDwell, bIsBlock, &rfEvent);
            m_vecRfLib.push_back(rfInfo);

            const int& magShapeID = rfEvent.magShape;
            if (!m_mapShapeLib.contains(magShapeID))
            {
                std::vector<float> vecAmp(ushSamples, 0.f);
                const float* fAmp = pSeqBlock->GetRFAmplitudePtr();
                std::memcpy(vecAmp.data(), fAmp, ushSamples * sizeof(float));
                m_mapShapeLib.insert(magShapeID, vecAmp);
            }

            const int& phaseShapeID = rfEvent.phaseShape;
            if (!m_mapShapeLib.contains(phaseShapeID))
            {
                std::vector<float> vecPhase(ushSamples, 0.f);
                const float* fPhase = pSeqBlock->GetRFPhasePtr();
                std::memcpy(vecPhase.data(), fPhase, ushSamples * sizeof(float));
                m_mapShapeLib.insert(phaseShapeID, vecPhase);
            }

        }
        dCurrentStartTime_us += pSeqBlock->GetDuration();
        m_dTotalDuration_us += pSeqBlock->GetDuration();
    }

    std::cout << m_vecRfLib.size() << " RF events detetced!\n";
    DrawWaveform(0, -1);

    return true;
}

bool MainWindow::IsBlockRf(const float* fAmp, const float* fPhase, const int& iSamples)
{
    for (int i = 0; i < iSamples - 1; ++i)
    {
        if (!(std::fabs(fAmp[i+1] - fAmp[i]) < 1e-6))
        {
            return false;
        }
        if (!(std::fabs(fPhase[i+1] - fPhase[i]) < 1e-6))
        {
            return false;
        }
    }
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

        double x1New = m_dDragStartRange - dx;
        double x2New = x1New + xSpan;

        x1New = x1New < 0 ? 0 : x1New;
        x2New = x2New > m_dTotalDuration_us ? m_dTotalDuration_us : x2New;
        UpdatePlotRange(x1New, x2New);
    }
}

void MainWindow::onMouseRelease(QMouseEvent *event)
{
    if (m_vecSeqBlocks.size() == 0) return;

    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    if(event->button() == Qt::LeftButton && m_bIsSelecting)
    {
        m_bIsSelecting = false;
        m_pSelectionRect->setVisible(false);

        // 获取选择的时间范围
        double x1 = ui->customPlot->xAxis->pixelToCoord(m_objSelectStartPos.x());
        double x2 = ui->customPlot->xAxis->pixelToCoord(event->pos().x());

        // 如果选择范围太小，认为是点击事件，不进行缩放
        if(qAbs(x2 - x1) > 5)
        {  
            if (x2 > x1)
            {
                UpdatePlotRange(x1, x2);
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
                UpdatePlotRange(x1New, x2New);
            }
        }
    }
    else if (event->button() == Qt::RightButton && m_bIsDragging)
    {
        m_bIsDragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void MainWindow::DrawWaveform(const double& dStartTime, double dEndTime)
{
    if (m_vecSeqBlocks.size() == 0) return;
    if (m_vecRfLib.size() == 0) return;
    if(dEndTime < 0) dEndTime = m_dTotalDuration_us;

    QPen pen;
    pen.setColor(Qt::blue);
    // pen.setWidth(2);
    pen.setJoinStyle(Qt::MiterJoin);
    pen.setCapStyle(Qt::FlatCap);

    double dRfMaxAmp(0.);
    double dRfMinAmp(0.);
    for(const auto& rfInfo : m_vecRfLib)
    {
        const auto& rf = rfInfo.event;
        const std::vector<float>& vecAmp = m_mapShapeLib[rf->magShape];
        const std::vector<float>& vecPhase = m_mapShapeLib[rf->phaseShape];

        double signal(0.);
        for(uint32_t index = 0; index < vecAmp.size(); index++)
        {
            const float& amp = vecAmp[index];
            const float& phase = vecPhase[index];
            signal = std::abs(std::polar(amp, phase)) * rfInfo.event->amplitude;
            dRfMaxAmp = std::max(dRfMaxAmp, signal);
            dRfMinAmp = std::min(dRfMinAmp, signal);
        }
    }
    double margin = (dRfMaxAmp - dRfMinAmp) * 0.1;
    m_mapRect["RF"]->axis(QCPAxis::atLeft)->setRange(dRfMinAmp - margin, dRfMaxAmp+ margin);
    QCPRange newRange(dStartTime, dEndTime);
    m_mapRect["RF"]->axis(QCPAxis::atBottom)->setRange(newRange);
    m_mapRect["RF"]->axis(QCPAxis::atBottom)->setRange(newRange);

    // 只处理时间范围内的RF
    for(const auto& rfInfo : m_vecRfLib)
    {
        // 跳过范围外的RF
        if(rfInfo.startAbsTime_us + rfInfo.duration_us < dStartTime) continue;
        if(rfInfo.startAbsTime_us > dEndTime) break;

        QCPGraph* rfGraph = ui->customPlot->addGraph(m_mapRect["RF"]->axis(QCPAxis::atBottom),
                                                     m_mapRect["RF"]->axis(QCPAxis::atLeft));
        rfGraph->setLineStyle(QCPGraph::lsStepLeft);
        m_vecRfGraphs.append(rfGraph);

        const auto& rf = rfInfo.event;
        double sampleTime = rfInfo.startAbsTime_us;
        const std::vector<float>& vecAmp = m_mapShapeLib[rf->magShape];
        const std::vector<float>& vecPhase = m_mapShapeLib[rf->phaseShape];

        const uint32_t& ushSamples = vecAmp.size();
        QVector<double> timePoints(ushSamples+3, 0.);
        QVector<double> amplitudes(ushSamples+3, 0.);

        timePoints[0] = sampleTime;
        amplitudes[0] = 0;
        double signal(0.);
        for(uint32_t index = 0; index < ushSamples; index++)
        {
            const float& amp = vecAmp[index];
            const float& phase = vecPhase[index];
            signal = std::abs(std::polar(amp, phase)) * rfInfo.event->amplitude;
            timePoints[index+1] = sampleTime;
            amplitudes[index+1] = signal;
            sampleTime += rfInfo.dwell;
        }

        timePoints[ushSamples+1] = sampleTime;
        amplitudes[ushSamples+1] = signal;

        timePoints[ushSamples+2] = sampleTime;
        amplitudes[ushSamples+2] = 0;
        rfGraph->setData(timePoints, amplitudes);

        rfGraph->setPen(pen);
        rfGraph->setSelectable(QCP::stWhole);
    }

    UpdatePlotRange(dStartTime, dEndTime);
}

void MainWindow::SlotExportData()
{
    if(!m_pSelectedGraph)
    {
        QMessageBox::information(this, "Hint", "Please select a graph!");
        return;
    }

    // 找到选中的graph在vector中的索引
    int graphIndex = m_vecRfGraphs.indexOf(m_pSelectedGraph);
    if(graphIndex == -1) return;
    // QString fileName = QFileDialog::getSaveFileName(this,
    //                                                 tr("保存波形数据"), "",
    //                                                 tr("文本文件 (*.txt);;所有文件 (*)"));

    // if(fileName.isEmpty()) return;

    QString fileName = "D:\\data.txt";

    QFile file(fileName);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream stream(&file);

        QSharedPointer<QCPGraphDataContainer> data = m_pSelectedGraph->data();
        double time(0.);
        double point(0.);
        for(int i = 0; i < data->size(); ++i)
        {
            time = data->at(i)->key;
            point = data->at(i)->value;
            stream << time << "\t" << point << "\n";
        }

        file.close();
        QMessageBox::information(this, "成功", "数据已成功导出");
    }
    else
    {
        QMessageBox::warning(this, "错误", "无法创建文件");
    }
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls())
    {
        event->acceptProposedAction();
    }
}

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

void MainWindow::onAxisRangeChanged(const QCPRange& newRange)
{
    QCPAxis* axis = qobject_cast<QCPAxis*>(sender());
    if (!axis) return;

    // check if exceeding range
    if (newRange.lower < 0 || newRange.upper > m_dTotalDuration_us) {
        QCPRange boundedRange = newRange;
        if (boundedRange.lower < 0)
        {
            boundedRange.lower = 0;
            boundedRange.upper = qMin(0 + newRange.size(), m_dTotalDuration_us);
        }
        if (boundedRange.upper > m_dTotalDuration_us)
        {
            boundedRange.upper = m_dTotalDuration_us;
            boundedRange.lower = qMax(m_dTotalDuration_us - newRange.size(), 0.);
        }
        axis->setRange(boundedRange);
    }
}

void MainWindow::onPlottableClick(QCPAbstractPlottable *plottable, int dataIndex, QMouseEvent *event)
{
    QCPGraph* graph = qobject_cast<QCPGraph*>(plottable);
    if(!graph) return;

    // 只需更新选中的 graph
    m_pSelectedGraph = graph;
}


