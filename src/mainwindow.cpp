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
	, m_bIsSelecting(false)
	, m_dTotalDuration_us(0.)
	, m_dDragStartRange(0.)
	, m_pBlockInfoDialog(nullptr)
{
	ui->setupUi(this);
	setAcceptDrops(true);

	m_vecVerticalLine = QVector<QCPItemStraightLine*>(6);
	for (int i = 0; i < 6; i++)
	{

		auto VerticalLine = new QCPItemStraightLine(ui->customPlot);
		VerticalLine->setVisible(false);
		QPen pen(Qt::red);
		pen.setStyle(Qt::DashLine); // 设置为虚线样式
		pen.setWidthF(0.5); // 设置线宽
		VerticalLine->setPen(pen);
		VerticalLine->point1->setType(QCPItemPosition::ptPlotCoords);
		VerticalLine->point2->setType(QCPItemPosition::ptPlotCoords);
		VerticalLine->setClipToAxisRect(true); // 使垂直线受单个坐标轴的限制
		m_vecVerticalLine[i] = (VerticalLine);
	}


	Init();

	m_listRecentPulseqFilePaths.resize(10);

	m_pSelectionRect = new QCPItemRect(ui->customPlot);
	m_pSelectionRect->setVisible(false);

	m_pCoordLabel = new QLabel(this); // 初始化 m_pCoordLabel
	ui->statusbar->addWidget(m_pCoordLabel); // 将 m_pCoordLabel 添加到状态栏

	// 初始化垂直线
	//m_pVerticalLine = new QCPItemStraightLine(ui->customPlot);
	//m_pVerticalLine->setVisible(false);
	//QPen pen(Qt::red);
	//pen.setStyle(Qt::DashLine); // 设置为虚线样式
	//pen.setWidthF(0.5); // 设置线宽
	//m_pVerticalLine->setPen(pen);
	//m_pVerticalLine->point1->setCoords(0, 0);
	//m_pVerticalLine->point2->setCoords(0, 1);
	//m_pVerticalLine->setClipToAxisRect(false); // 使垂直线不受单个坐标轴的限制
	// 连接右键菜单信号
	ui->customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(ui->customPlot, &QCustomPlot::customContextMenuRequested, this, &MainWindow::showContextMenu);

}

MainWindow::~MainWindow()
{
	ClearPulseqCache();
	delete ui;
	SAFE_DELETE(m_pVersionLabel);
	SAFE_DELETE(m_pProgressBar);
	SAFE_DELETE(m_pCoordLabel); // 删除 m_pCoordLabel
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
	//connect(ui->customPlot, &QCustomPlot::mousePress, this, &MainWindow::onMousePress);
	//connect(ui->customPlot, &QCustomPlot::mouseMove, this, &MainWindow::onMouseMove);
	//connect(ui->customPlot, &QCustomPlot::mouseRelease, this, &MainWindow::onMouseRelease);

	// Interaction
	connect(ui->customPlot, &QCustomPlot::mouseMove, this, &MainWindow::onMouseMove);
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
	ui->customPlot->clearGraphs();
	ui->customPlot->plotLayout()->clear();

	ui->customPlot->setAntialiasedElements(QCP::aeAll);
	ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


	m_pADCLabelsRect = new QCPAxisRect(ui->customPlot);
	m_pRfMagRect = new QCPAxisRect(ui->customPlot);
	m_pRfADCPhaseRect = new QCPAxisRect(ui->customPlot);
	m_pGxRect = new QCPAxisRect(ui->customPlot);
	m_pGyRect = new QCPAxisRect(ui->customPlot);
	m_pGzRect = new QCPAxisRect(ui->customPlot);

	m_vecRects.append(m_pADCLabelsRect);
	m_vecRects.append(m_pRfMagRect);
	m_vecRects.append(m_pRfADCPhaseRect);
	m_vecRects.append(m_pGxRect);
	m_vecRects.append(m_pGyRect);
	m_vecRects.append(m_pGzRect);

	// Create a margin group for aligning the left margins of all axis rects
	auto m_pMarginGroup = new QCPMarginGroup(ui->customPlot);
	for(int i = 0; i < m_vecRects.count(); i++)
	{
		ui->customPlot->plotLayout()->addElement(i, 0, m_vecRects[i]);
		m_vecRects[i]->axis(QCPAxis::atLeft)->setRange(-1, 1);
		m_vecRects[i]->axis(QCPAxis::atBottom)->setRange(0, 100);

		// Set the left margin of each axis rect to the margin group
		m_vecRects[i]->setMarginGroup(QCP::msLeft, m_pMarginGroup);

		// Set margins and spacing to reduce the gap between graphs
		m_vecRects[i]->setMargins(QMargins(0, 0, 0, 2));
		m_vecRects[i]->setMinimumMargins(QMargins(0, 0, 0, 0));
		m_vecRects[i]->setAutoMargins(QCP::msLeft);
		
	}

	m_vecRects[5]->setAutoMargins(QCP::msBottom| QCP::msLeft);
	
	// Hide x-axis tick labels for m_vecRects[0] to m_vecRects[4]
	for (int i = 0; i < 5; i++)
	{
		m_vecRects[i]->axis(QCPAxis::atBottom)->setTickLabels(false);
	}

	// // 只允许水平方向拖拽
	// m_pRfRect->setRangeDrag(Qt::Horizontal);    // 设置RF区域只能水平拖拽
	// m_pGzRect->setRangeDrag(Qt::Horizontal);    // 对每个区域都设置
	// m_pGyRect->setRangeDrag(Qt::Horizontal);
	// m_pGxRect->setRangeDrag(Qt::Horizontal);
	// m_pAdcRect->setRangeDrag(Qt::Horizontal);

	// // 同样，对于缩放也可以只允许水平方向
	// m_pRfRect->setRangeZoom(Qt::Horizontal);    // 设置只能水平缩放
	// m_pGzRect->setRangeZoom(Qt::Horizontal);
	// m_pGyRect->setRangeZoom(Qt::Horizontal);
	// m_pGxRect->setRangeZoom(Qt::Horizontal);
	// m_pAdcRect->setRangeZoom(Qt::Horizontal);

	m_pADCLabelsRect->setRangeZoom(Qt::Horizontal);
	m_pRfMagRect->setRangeZoom(Qt::Horizontal);
	m_pRfADCPhaseRect->setRangeZoom(Qt::Horizontal);

	//m_pRfRect->setupFullAxesBox(true);
	m_pADCLabelsRect->axis(QCPAxis::atLeft)->setLabel("ADC/labels");
	m_pRfMagRect->axis(QCPAxis::atLeft)->setLabel("RF mag(Hz)");
	m_pRfADCPhaseRect->axis(QCPAxis::atLeft)->setLabel("RF/ADC ph(rad)");

	m_pGxRect->axis(QCPAxis::atLeft)->setLabel("GX (Hz/m)");
	m_pGyRect->axis(QCPAxis::atLeft)->setLabel("GY (Hz/m)");
	m_pGzRect->axis(QCPAxis::atLeft)->setLabel("GZ (Hz/m)");

	// share the same time axis
	//m_pRfRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
	//m_pGzRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
	//m_pGyRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
	//m_pGxRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");
	m_pGzRect->axis(QCPAxis::atBottom)->setLabel("Time (us)");

	// Hide all time axis but the last one
	//m_pRfRect->axis(QCPAxis::atBottom)->setVisible(false);
	//m_pGzRect->axis(QCPAxis::atBottom)->setVisible(false);
	//m_pGyRect->axis(QCPAxis::atBottom)->setVisible(false);
	//m_pGxRect->axis(QCPAxis::atBottom)->setVisible(false);
	for (int i = 0; i < m_vecRects.count(); i++)
	{
		//m_vecGraphs.append(ui->customPlot->addGraph(m_vecRects[i]->axis(QCPAxis::atBottom), m_vecRects[i]->axis(QCPAxis::atLeft)));

		// Connect the rangeChanged signal of each x-axis to the synchronizeXAxes slot
		connect(m_vecRects[i]->axis(QCPAxis::atBottom), SIGNAL(rangeChanged(QCPRange)), this, SLOT(synchronizeXAxes(QCPRange)));
	}
	
	// 定义 7 种颜色 用于绘制不同的RF波形
	colors = QVector<QColor>({
		QColor::fromRgbF(0,0.447,0.741),
		QColor::fromRgbF(0.85,0.325,0.098),
		QColor::fromRgbF(0.929,0.694,0.125),
		QColor::fromRgbF(0.494,0.184,0.556),
		QColor::fromRgbF(0.466,0.674,0.188),
		QColor::fromRgbF(0.301,0.745,0.933),
		QColor::fromRgbF(0.635,0.078,0.184)
	});
	
	// 初始化每个 QCPAxisRect 的数据光标
	for (auto rect : m_vecRects)
	{
		QCPItemTracer* tracer = new QCPItemTracer(ui->customPlot);
		tracer->setClipAxisRect(rect);
		tracer->setVisible(false);
		//tracer->setStyle(QCPItemTracer::tsCrosshair);
		tracer->setPen(QPen(Qt::red));
		m_vecTracers.append(tracer);
	}

	for (size_t i = 0; i < m_vecRects.size(); i++)
	{
		m_vecVerticalLine[i]->setClipAxisRect(m_vecRects[i]);
		m_vecVerticalLine[i]->point1->setAxisRect(m_vecRects[i]);
		m_vecVerticalLine[i]->point2->setAxisRect(m_vecRects[i]);
		m_vecVerticalLine[i]->point1->setAxes(m_vecRects[i]->axis(QCPAxis::atBottom), m_vecRects[i]->axis(QCPAxis::atLeft));
		m_vecVerticalLine[i]->point2->setAxes(m_vecRects[i]->axis(QCPAxis::atBottom), m_vecRects[i]->axis(QCPAxis::atLeft));
	}
	// plot RF
	// m_vecRects[1]
	// 模拟多组RF的显示，
	// t:每组RF有对应的block start time(依据之前的block算出来)，rf的delay，block的rfDwellTime_us
	// y:block的rfAmplitude[]和rf的amplitude
	long t0 = 1;

}

void MainWindow::synchronizeXAxes(const QCPRange& newRange)
{
	for(int i = 0; i < m_vecRects.count(); i++)
	{
		// Block signals to prevent recursive updates
		m_vecRects[i]->axis(QCPAxis::atBottom)->blockSignals(true);
	}

	// Update the range of all x-axes
	for (int i = 0; i < m_vecRects.count(); i++)
	{
		m_vecRects[i]->axis(QCPAxis::atBottom)->setRange(newRange);
	}

	// Unblock signals
	for (int i = 0; i < m_vecRects.count(); i++)
	{
		m_vecRects[i]->axis(QCPAxis::atBottom)->blockSignals(false);
	}
	
	// Replot the custom plot
	ui->customPlot->replot();
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
		ui->customPlot->replot();
	}

	m_dTotalDuration_us = 0.;

	//m_mapShapeLib.clear();
	//m_vecRfLib.clear();
	if (nullptr != m_spPulseqSeq.get())
	{
		m_spPulseqSeq->reset();
		m_spPulseqSeq.reset(new ExternalSequence);
		for (uint16_t ushBlockIndex=0; ushBlockIndex < m_vecDecodeSeqBlocks.size(); ushBlockIndex++)
		{
			SAFE_DELETE(m_vecDecodeSeqBlocks[ushBlockIndex]);
		}
		m_vecDecodeSeqBlocks.clear();
		std::cout << m_sPulseqFilePath.toStdString() << " Closed\n";
	}
	this->setWindowFilePath("");
}

bool MainWindow::LoadPulseqFile(const QString& sPulseqFilePath)
{
	this->setEnabled(false);

	// 获得tFactor值
	TimeUnits = "ms";
	tFactor = 1e-3;
	for (int i = 0; i < validTimeUnits.size(); i++)
	{
		if (TimeUnits == validTimeUnits[i])
		{
			tFactor = tFactorList[i];
			break;
		}
	}


	//ClearPulseqCache();
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
	m_vecDecodeSeqBlocks.resize(lSeqBlockNum);
	m_pProgressBar->show();
	int64_t progress(0);
	vecBlockEdges.clear();
	vecBlockEdges.resize(lSeqBlockNum + 1, 0);
	for (int64_t ushBlockIndex=0; ushBlockIndex < lSeqBlockNum; ushBlockIndex++)
	{
		m_vecDecodeSeqBlocks[ushBlockIndex] = m_spPulseqSeq->GetBlock(ushBlockIndex);
		if (!m_spPulseqSeq->decodeBlock(m_vecDecodeSeqBlocks[ushBlockIndex]))
		{
			std::stringstream sLog;
			sLog << "Decode SeqBlock failed, block index: " << ushBlockIndex;
			QMessageBox::critical(this, "File Error", sLog.str().c_str());
			ClearPulseqCache();
			return false;
		}
		progress = ushBlockIndex * 100 / lSeqBlockNum;
		m_pProgressBar->setValue(progress);
		vecBlockEdges[ushBlockIndex + 1] = vecBlockEdges[ushBlockIndex] + m_vecDecodeSeqBlocks[ushBlockIndex]->GetDuration() * tFactor;
	}

	m_pProgressBar->setValue(100);
	this->setEnabled(true);

	nBlockRangeStart = 10;
	nBlockRangeEnd = std::min(int(lSeqBlockNum - 1), 100);

	

	DrawRFWaveform(0, -1);
	DrawADCWaveform(0, -1);
	DrawGWaveform(0, -1);

	if (bShowBlocksEdges)
	{
		DrawBlockEdges();
	}
	// 使用TimeUnits自动设置Label
	m_pGzRect->axis(QCPAxis::atBottom)->setLabel("Time (" + TimeUnits + ")");
	ui->customPlot->replot();
	return true;
}

bool MainWindow::ClosePulseqFile()
{
	ClearPulseqCache();

	return true;
}

std::tuple<double, size_t> MainWindow::calcRfCenter(const std::vector<double>& signal, const std::vector<double>& t)
{
	// 计算 rfmax
	double rfmax = *std::max_element(signal.begin(), signal.end(), [](double a, double b) {
		return std::abs(a) < std::abs(b);
		});

	// 找到 ipeak
	std::vector<size_t> ipeak;
	for (size_t i = 0; i < signal.size(); ++i) 
	{
		if (std::abs(signal[i]) >= rfmax * 0.99999) 
		{
			ipeak.push_back(i);
		}
	}

	// 计算 tc 和 ic
	double tc = (t[ipeak.front()] + t[ipeak.back()]) / 2;
	size_t ic = ipeak[ipeak.size() / 2];

	return std::make_tuple(tc, ic);
}

void MainWindow::DrawRFWaveform(const double& dStartTime, double dEndTime)
{
	double t0 = vecBlockEdges[nBlockRangeStart];
	int cnt = 0;
	double min1 = std::numeric_limits<double>::max(), max1 = std::numeric_limits<double>::min();
	double min2 = std::numeric_limits<double>::max(), max2 = std::numeric_limits<double>::min();
	for (int iBlock = nBlockRangeStart; iBlock <= nBlockRangeEnd; iBlock++)
	{
		const auto& pSeqBlock = m_vecDecodeSeqBlocks[iBlock];
		if (pSeqBlock->isRF())
		{
			const RFEvent& rf = pSeqBlock->GetRFEvent();
			const int& ushSamples = pSeqBlock->GetRFLength();
			const float& fDwell = pSeqBlock->GetRFDwellTime();
			const double& dDuration_us = ushSamples * fDwell;

			// t:每组RF有对应的block start time(依据之前的block算出来)，rf的delay，block的rfDwellTime_us
			// y_1:block的rfAmplitude[]和rf的amplitude
			// y_2:phase,angle(s.*sign(real(s))*exp(1i*rf.phaseOffset).*exp(1i*2*pi*t    *rf.freqOffset))
			double t = t0 + rf.delay * tFactor;
			const float* rfList = pSeqBlock->GetRFAmplitudePtr();
			const float* phaseList = pSeqBlock->GetRFPhasePtr();
			const int RFLength = pSeqBlock->GetRFLength();

			// 添加波形数据点
			QVector<double> timePoints(RFLength, 0.); // us
			QVector<double> amplitudes(RFLength, 0.); // *rfEvent.amplitude Hz
			QVector<double> phases(RFLength, 0.); // rad
			int step = 1;
			//if(RFLength > 200)
			//{
			//	step = round(m_spPulseqSeq->GetGradientRasterTime_us() / fDwell);
			//}

			for (int j = 0; j < RFLength; j = j + step )
			{
				timePoints[j] = (t + j * fDwell * tFactor);
				amplitudes[j] = rfList[j] * rf.amplitude * cos(phaseList[j]);

				std::complex<double> signal = std::polar(double(amplitudes[j]), double(phaseList[j] + rf.phaseOffset + 2 * PI *j * fDwell * rf.freqOffset / 1000000));
				phases[j] = std::arg(signal);
				//phases[j] = phaseList[j] + rfEvent.phaseOffset + 2 * PI * j * fDwell * rfEvent.freqOffset / 1000000;
			}
			QVector<double> timePoints2 = timePoints;

            if(amplitudes.first() != 0)
            {
                timePoints.prepend(timePoints.first());
                amplitudes.prepend(0);
            }
            if (amplitudes.last() != 0)
            {
                timePoints.append(timePoints.last());
                amplitudes.append(0);
            }

			// 在数据点之间插入NaN值以创建不连续的曲线
			//timePoints.append(timePoints.last());
			//amplitudes.append(std::numeric_limits<double>::quiet_NaN());

			// phases
			if (phases.first() != 0)
			{
				timePoints2.prepend(timePoints2.first());
				phases.prepend(0);
			}
			if (phases.last() != 0)
			{
				timePoints2.append(timePoints2.last());
				phases.append(0);
			}
			// 在数据点之间插入NaN值以创建不连续的曲线
			//timePoints2.append(timePoints2.last());
			//phases.append(std::numeric_limits<double>::quiet_NaN());

			
			QPen pen(colors[cnt % colors.size()]);
			pen.setWidthF(1);
			cnt++;

			// 添加波形数据点
			auto graph_RF_Mag = ui->customPlot->addGraph(m_vecRects[1]->axis(QCPAxis::atBottom), m_vecRects[1]->axis(QCPAxis::atLeft));
			graph_RF_Mag->setPen(pen);
			graph_RF_Mag->addData(timePoints, amplitudes);

			auto graph_RFADC_ph = ui->customPlot->addGraph(m_vecRects[2]->axis(QCPAxis::atBottom), m_vecRects[2]->axis(QCPAxis::atLeft));
			graph_RFADC_ph->setPen(pen);
			graph_RFADC_ph->addData(timePoints2, phases);

			

			{
				// 调用 calcRfCenter 函数
				std::vector<double> signal(amplitudes.begin(), amplitudes.end());
				std::vector<double> t(timePoints.begin(), timePoints.end());
				auto [tc, ic] = calcRfCenter(signal, t);
				// 设置散点样式为 "x"
				auto graph_RFADC_ph2 = ui->customPlot->addGraph(m_vecRects[2]->axis(QCPAxis::atBottom), m_vecRects[2]->axis(QCPAxis::atLeft));
				QCPScatterStyle scatterStyle(QCPScatterStyle::ssCross, 5);
				graph_RFADC_ph2->setScatterStyle(scatterStyle);
				graph_RFADC_ph2->setPen(QPen(Qt::blue));
				graph_RFADC_ph2->setLineStyle(QCPGraph::lsNone); // 不连接散点
				graph_RFADC_ph2->addData(tc, phases[ic]);
			}

			//graph_RF_Mag->addData(timePoints.last(), std::numeric_limits<double>::quiet_NaN());
			//m_vecGraphs[2]->addData(timePoints2.last(), std::numeric_limits<double>::quiet_NaN());

			{
				// 记录amplitudes列表中的最大值
				max1 = std::max(max1, *std::max_element(amplitudes.begin(), amplitudes.end()));
				min1 = std::min(min1, *std::min_element(amplitudes.begin(), amplitudes.end()));
			}
			{
				// 记录ph列表中的最大值
				max2 = std::max(max2, *std::max_element(phases.begin(), phases.end()));
				min2 = std::min(min2,  *std::min_element(phases.begin(), phases.end()));
			}
		}

		t0 += pSeqBlock->GetDuration() * tFactor;
	}

	// Y轴范围
	// Y1
	{
		double range1 = min1 - 0.03 * (max1 - min1);
		double range2 = max1 + 0.03 * (max1 - min1);
		// 设置m_vecGraphs[1]的Y轴范围为0到maxAmplitude
		m_vecRects[1]->axis(QCPAxis::atLeft)->setRange(range1, range2);
	}
	// Y2
	{
		double range1 = min2 - 0.03 * (max2 - min2);
		double range2 = max2 + 0.03 * (max2 - min2);
		// 设置m_vecGraphs[2]的Y轴范围为0到maxAmplitude
		m_vecRects[2]->axis(QCPAxis::atLeft)->setRange(range1, range2);
	}

	// X轴范围
	ui->customPlot->xAxis->setRange(vecBlockEdges[nBlockRangeStart], vecBlockEdges[nBlockRangeEnd]);
	ui->customPlot->replot();
}

void MainWindow::DrawADCWaveform(const double& dStartTime, double dEndTime)
{
	double t0 = vecBlockEdges[nBlockRangeStart];
	int cnt = 0;
	double min1 = std::numeric_limits<double>::max(), max1 = std::numeric_limits<double>::min();
	double min2 = std::numeric_limits<double>::max(), max2 = std::numeric_limits<double>::min();
	for (int iBlock = nBlockRangeStart; iBlock <= nBlockRangeEnd; iBlock++)
	{
		const auto& pSeqBlock = m_vecDecodeSeqBlocks[iBlock];
		if (pSeqBlock->isADC())
		{
			const ADCEvent& adc = pSeqBlock->GetADCEvent();
			//int numSamples;     /**< @brief Number of samples */
			//int dwellTime;      /**< @brief Dwell time of ADC readout (ns) */
			//int delay;          /**< @brief Delay before first sample (us) */
			//float freqOffset;   /**< @brief Frequency offset of receiver (Hz) */
			//float phaseOffset;  /**< @brief Phase offset of receiver (rad) */
			QVector<double> t(adc.numSamples, 0.);
			QVector<double> y(adc.numSamples, 0.);
			QVector<double> ADC_phase(adc.numSamples, 0.);
			for (size_t i = 0; i < adc.numSamples; i++)
			{
				t[i] = t0 + adc.delay* tFactor + (i+0.5) * adc.dwellTime / 1000.0 * tFactor;
				std::complex<double> signal = std::polar(double(1), double(adc.phaseOffset + 2 * PI * (t[i]-t0) * adc.freqOffset / 1000000.0));
				ADC_phase[i] = std::arg(signal);;
			}
			
			
			{
				// 设置散点样式为 "x"
				auto graph_ADC = ui->customPlot->addGraph(m_vecRects[0]->axis(QCPAxis::atBottom), m_vecRects[0]->axis(QCPAxis::atLeft));
				QCPScatterStyle scatterStyle(QCPScatterStyle::ssCross, 5);
				graph_ADC->setScatterStyle(scatterStyle);
				graph_ADC->setPen(QPen(Qt::red));
				graph_ADC->setLineStyle(QCPGraph::lsNone); // 不连接散点
				graph_ADC->addData(t, y);
			}
			{
				// 设置散点样式为 "."
				auto graph_RFADC_ph = ui->customPlot->addGraph(m_vecRects[2]->axis(QCPAxis::atBottom), m_vecRects[2]->axis(QCPAxis::atLeft));
				QCPScatterStyle scatterStyle(QCPScatterStyle::ssDot, 1);
				graph_RFADC_ph->setScatterStyle(scatterStyle);
				graph_RFADC_ph->setPen(QPen(Qt::blue));
				graph_RFADC_ph->setLineStyle(QCPGraph::lsNone); // 不连接散点
				graph_RFADC_ph->addData(t, ADC_phase);
			}



		}
		t0 += pSeqBlock->GetDuration() * tFactor;
	}

	ui->customPlot->xAxis->setRange(vecBlockEdges[nBlockRangeStart], vecBlockEdges[nBlockRangeEnd]);
	ui->customPlot->replot();
}

void MainWindow::DrawGWaveform(const double& dStartTime, double dEndTime)
{
	int channel = 0;
	for (int channel = 0; channel < 3; channel++)
	{
		double t0 = vecBlockEdges[nBlockRangeStart];
		int cnt1 = 0;
		double min1 = std::numeric_limits<double>::max(), max1 = std::numeric_limits<double>::min();
		for(int iBlock = nBlockRangeStart; iBlock <= nBlockRangeEnd; iBlock++)
		{
			Event eventG = static_cast<Event>(channel + GX);
			const auto& pSeqBlock = m_vecDecodeSeqBlocks[iBlock];
			if (pSeqBlock->isTrapGradient(channel) || pSeqBlock->isArbitraryGradient(channel) || pSeqBlock->isExtTrapGradient(channel))
			{
				const GradEvent& grad = pSeqBlock->GetGradEvent(channel);

				//float amplitude;      /**< @brief Amplitude of gradient (Hz/m) */
				//int delay;
				// Trapezoid:
				//long rampUpTime;      /**< @brief Ramp up time of trapezoid (us) */
				//long flatTime;        /**< @brief Flat-top time of trapezoid (us) */
				//long rampDownTime;    /**< @brief Ramp down time of trapezoid (us) */
				// Arbitrary:
				//int waveShape;        /**< @brief whave shape ID for arbitrary gradient */
				//int timeShape;        /**< @brief time shaoeID for arbitrary gradient; 0 means regular sampling */
				if (pSeqBlock->isTrapGradient(channel))
				{
					QVector<double> t(5, 0.);
					QVector<double> y(5, 0.);
					t[0] = 0;
					t[1] = t[0] + grad.delay* tFactor;
					t[2] = t[1] + grad.rampUpTime* tFactor;
					t[3] = t[2] + grad.flatTime* tFactor;
					t[4] = t[3] + grad.rampDownTime* tFactor;
					y[0] = 0;
					y[1] = 0;
					y[2] = grad.amplitude / 1000;
					y[3] = grad.amplitude / 1000;
					y[4] = 0;

					{
						QVector<double> tplot(5, 0.);
						for(int j = 0; j < t.size(); j++)
						{
							tplot[j] = t0 + t[j];
						}
						auto graph_G = ui->customPlot->addGraph(m_vecRects[channel + 3]->axis(QCPAxis::atBottom), m_vecRects[channel + 3]->axis(QCPAxis::atLeft));
						QPen pen(colors[cnt1 % colors.size()]);
						pen.setWidthF(1);
						graph_G->setPen(pen);
						graph_G->addData(tplot, y);
					}

					cnt1++;

				}
				else if (pSeqBlock->isArbitraryGradient(channel))
				{
					// Arbitrary:
					//int waveShape;        /**< @brief whave shape ID for arbitrary gradient */
					//int timeShape;        /**< @brief time shape ID for arbitrary gradient; 0 means regular sampling */
					/**
					* @brief Return the number of samples of the arbitrary gradient on the given gradient channel.
					* Only relevant for arbitrary gradients
					*/
					//int     GetArbGradNumSamples(int channel);

					/**
					 * @brief Directly get a pointer to the samples of the arbitrary gradient
					 */
					//float* GetArbGradShapePtr(int channel);
					float* gradAmplitude = pSeqBlock->GetArbGradShapePtr(channel);
					int gradNumSamples = pSeqBlock->GetArbGradNumSamples(channel);
					QVector<double> t(gradNumSamples, 0.);
					QVector<double> y(gradNumSamples, 0.);
					double gradRasterTime_us = m_spPulseqSeq->GetGradientRasterTime_us();
					for (size_t i = 0; i < gradNumSamples; i++)
					{
						t[i] = t0 + grad.delay * tFactor + i * gradRasterTime_us * tFactor;
						y[i] = gradAmplitude[i] / 1000;
					}
					{
						auto graph_G = ui->customPlot->addGraph(m_vecRects[channel + 3]->axis(QCPAxis::atBottom), m_vecRects[channel + 3]->axis(QCPAxis::atLeft));
						QPen pen(colors[cnt1 % colors.size()]);
						pen.setWidthF(1);
						graph_G->setPen(pen);
						graph_G->addData(t, y);
					}
					cnt1++;
				}
				else if (pSeqBlock->isExtTrapGradient(channel))
				{
					/**
					* @brief Return the timening and the shape of the ExtTrp grdient on the given gradient channel.
					* Only relevant for ExtTrap gradients
					*/
					//const std::vector<long>& GetExtTrapGradTimes(int channel);

					/**
					 * @brief Return the timening and the shape of the ExtTrp grdient on the given gradient channel.
					 * Only relevant for ExtTrap gradients
					 */
					//const std::vector<float>& GetExtTrapGradShape(int channel);
					std::vector<long> gradTimes = pSeqBlock->GetExtTrapGradTimes(channel);
					std::vector<float> gradShape = pSeqBlock->GetExtTrapGradShape(channel);
					if (gradTimes.size() != gradShape.size())
					{
						std::cout << "gradTimes.size() != gradShape.size()\n";
						return;
					}
					// 添加波形数据点
					QVector<double> t(gradTimes.size(), 0.);
					QVector<double> y(gradTimes.size(), 0.);
					for (size_t i = 0; i < gradTimes.size(); i++)
					{
						// todo:check ExtTrapGradTimes  t0 + grad.delay + gradTimes[i] * tFactor
						t[i] = t0 + grad.delay * tFactor + gradTimes[i] * tFactor * tFactor;
						y[i] = gradShape[i] / 1000;
					}
					{
						auto graph_G = ui->customPlot->addGraph(m_vecRects[channel + 3]->axis(QCPAxis::atBottom), m_vecRects[channel + 3]->axis(QCPAxis::atLeft));
						QPen pen(colors[cnt1 % colors.size()]);
						pen.setWidthF(1);
						graph_G->setPen(pen);
						graph_G->addData(t, y);
					}
				}

				{
					// 记录amplitudes列表中的最大值
					max1 = std::max(max1, double(grad.amplitude / 1000));
					min1 = std::min(min1, double(grad.amplitude / 1000));
				}
			}
			t0 += pSeqBlock->GetDuration()*tFactor;
			
		}

		// Y轴范围
		{
			double range1 = min1 - 0.03 * (max1 - min1);
			double range2 = max1 + 0.03 * (max1 - min1);
			// 设置m_vecGraphs[channel + 3]的Y轴范围为0到maxAmplitude
			m_vecRects[channel + 3]->axis(QCPAxis::atLeft)->setRange(range1, range2);
		}

		
		ui->customPlot->xAxis->setRange(vecBlockEdges[nBlockRangeStart], vecBlockEdges[nBlockRangeEnd]);
		ui->customPlot->replot();

		
	}

	
}

void MainWindow::DrawBlockEdges()
{
	if (vecBlockEdges.size() == 0)
	{
		return;
	}

	{
		// 假设外部参数提供的 tick 位置：
		QVector<double> tickPositions;
		for (int i = nBlockRangeStart; i <= nBlockRangeEnd; i++)
		{
			tickPositions.append(vecBlockEdges[i]);
		}

		// 创建一个 QCPAxisTickerText 对象
		QSharedPointer<QCPAxisTickerText> ticker(new QCPAxisTickerText);

		// 用一个 QMap 将 tick 位置映射为空字符串（也可以不用填充文本）
		QMap<double, QString> ticksMap;
		for (double pos : tickPositions) {
			ticksMap.insert(pos, QString::number(pos));  // 空字符串表示不显示标签
		}
		ticker->setTicks(ticksMap);
		for (int i = 0; i < m_vecRects.size(); i++)
		{
			// 将自定义的 ticker 应用到 x 轴
			m_vecRects[i]->axis(QCPAxis::atBottom)->setTicker(ticker);
		}
	}

	ui->customPlot->xAxis->setRange(vecBlockEdges[nBlockRangeStart], vecBlockEdges[nBlockRangeEnd]);
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


void MainWindow::onMouseMove(QMouseEvent* event)
{
	//double x = ui->customPlot->xAxis->pixelToCoord(event->pos().x());

	//double minDist = std::numeric_limits<double>::max();
	//double closestX = 0;
	//bool found = false;

	//// 遍历所有 QCPAxisRect 中的所有图表
	//for (auto rect : m_vecRects)
	//{
	//	for (int i = 0; i < rect->graphs().size(); ++i)
	//	{
	//		QCPGraph* graph = rect->graphs().at(i);
	//		if (graph)
	//		{
	//			for (int j = 0; j < graph->dataCount(); ++j)
	//			{
	//				double dist = std::abs(graph->data()->at(j)->key - x);
	//				if (dist < minDist)
	//				{
	//					minDist = dist;
	//					closestX = graph->data()->at(j)->key;
	//					found = true;
	//				}
	//			}
	//		}
	//	}
	//}

	//if (found)
	//{
	//	for (int i = 0; i < m_vecRects.size(); ++i)
	//	{
	//		m_vecTracers[i]->position->setCoords(closestX, m_vecRects[i]->axis(QCPAxis::atLeft)->range().center());
	//		m_vecTracers[i]->setVisible(true);
	//	}
	//	ui->customPlot->replot();
	//}
	//else
	//{
	//	for (auto tracer : m_vecTracers)
	//	{
	//		tracer->setVisible(false);
	//	}
	//}

	double x = ui->customPlot->xAxis->pixelToCoord(event->pos().x());

	double minDist = std::numeric_limits<double>::max();
	double closestX = 0;
	bool found = false;

	// 遍历所有 QCPAxisRect 中的所有图表
	for (auto rect : m_vecRects)
	{
		for (int i = 0; i < rect->graphs().size(); ++i)
		{
			QCPGraph* graph = rect->graphs().at(i);
			if (graph)
			{
				auto it = graph->data()->findBegin(x);
				if (it != graph->data()->end())
				{
					double dist = std::abs(it->key - x);
					if (dist < minDist)
					{
						minDist = dist;
						closestX = it->key;
						found = true;
					}
				}
			}
		}
	}

	if (found)
	{
		// 计算当前是第多少个 block
		int currentBlock = -1;
		for (int i = 0; i < vecBlockEdges.size() - 1; ++i)
		{
			if (closestX >= vecBlockEdges[i] && closestX < vecBlockEdges[i + 1])
			{
				currentBlock = i;
				break;
			}
		}

		QString coordText = QString("Time: %1 %2").arg(closestX).arg(TimeUnits);
		if (currentBlock != -1)
		{
			coordText += QString(", Block: %1").arg(currentBlock);
		}

		QVector<QString> yTexts = {"ADC/labels", "RF mag", "RF/ADC ph", "Gx", "Gy", "Gz"};
		QVector<QString> yUnits = { "", "Hz", "rad", "KHz/m", "KHz/m", "KHz/m" };
		for (int i = 0; i < m_vecVerticalLine.count(); i++)
		{
			m_vecVerticalLine[i]->point1->setCoords(closestX, m_vecRects[i]->axis(QCPAxis::atBottom)->range().lower);
			m_vecVerticalLine[i]->point2->setCoords(closestX, m_vecRects[i]->axis(QCPAxis::atBottom)->range().upper);
			m_vecVerticalLine[i]->setVisible(true);

			for (int j = 0; j < m_vecRects[i]->graphs().size(); ++j)
			{
				QCPGraph* graph = m_vecRects[i]->graphs().at(j);
				if (graph)
				{
					double y = std::numeric_limits<double>::quiet_NaN();
					for (auto it = graph->data()->begin(); it != graph->data()->end(); ++it)
					{
						if (it->key == closestX)
						{
							y = it->value;
							break;
						}
					}
					if (!isnan(y))
					{
						coordText += QString(", %1: %2 %3").arg(yTexts[i]).arg(y).arg(yUnits[i]);
					}
				}
			}
		}
		m_pCoordLabel->setText(coordText);
		ui->customPlot->replot();
	}
	else
	{
		for (auto VerticalLine : m_vecVerticalLine)
		{
			VerticalLine->setVisible(false);
		}
	}
}

void MainWindow::showContextMenu(const QPoint& pos)
{
	m_rightClickPos = pos; // 记录鼠标右键点击的位置
	QMenu contextMenu(tr("Context menu"), this);

	QAction actionInformation("Information", this);
	connect(&actionInformation, &QAction::triggered, this, &MainWindow::showBlockInformation);
	contextMenu.addAction(&actionInformation);

	contextMenu.exec(ui->customPlot->mapToGlobal(pos));
}

void MainWindow::showBlockInformation()
{
	double x = ui->customPlot->xAxis->pixelToCoord(m_rightClickPos.x());

	double minDist = std::numeric_limits<double>::max();
	double closestX = 0;
	bool found = false;

	// 遍历所有 QCPAxisRect 中的所有图表
	for (auto rect : m_vecRects)
	{
		for (int i = 0; i < rect->graphs().size(); ++i)
		{
			QCPGraph* graph = rect->graphs().at(i);
			if (graph)
			{
				auto it = graph->data()->findBegin(x);
				if (it != graph->data()->end())
				{
					double dist = std::abs(it->key - x);
					if (dist < minDist)
					{
						minDist = dist;
						closestX = it->key;
						found = true;
					}
				}
			}
		}
	}

	if (found)
	{
		// 计算当前是第多少个 block
		int currentBlock = -1;
		for (int i = 0; i < vecBlockEdges.size() - 1; ++i)
		{
			if (closestX >= vecBlockEdges[i] && closestX < vecBlockEdges[i + 1])
			{
				currentBlock = i;
				break;
			}
		}

		if (currentBlock != -1)
		{

			if (!m_pBlockInfoDialog)
			{
				m_pBlockInfoDialog = new EventBlockInfoDialog(this);
			}

			setBlockInfoContent(currentBlock);

			m_pBlockInfoDialog->show();
		}
	}
}

void MainWindow::setBlockInfoContent(int currentBlock)
{
	QString blockInfo = QString("/-----------------------------------------------------------------------------------------------/\n");
	blockInfo += QString("Block: %1\nStart Time: %2 %3\nEnd Time: %4 %5\n")
		.arg(currentBlock)
		.arg(vecBlockEdges[currentBlock])
		.arg(TimeUnits)
		.arg(vecBlockEdges[currentBlock + 1])
		.arg(TimeUnits);

	
	const auto& pSeqBlock = m_vecDecodeSeqBlocks[currentBlock];
	if (pSeqBlock->isRF())
	{
		blockInfo += QString("|-----------------------------------------------------------------------------------------------|\n");
		const RFEvent& rf = pSeqBlock->GetRFEvent();
		blockInfo += QString("RF Event:\nAmplitude: %1 Hz\nFrequency Offset: %2 Hz\nPhase Offset: %3 rad\nDelay: %4 us\n")
			.arg(rf.amplitude)
			.arg(rf.freqOffset)
			.arg(rf.phaseOffset)
			.arg(rf.delay);
	}

	if (pSeqBlock->isADC())
	{
		blockInfo += QString("|-----------------------------------------------------------------------------------------------|\n");
		const ADCEvent& adc = pSeqBlock->GetADCEvent();
		blockInfo += QString("ADC Event:\nNumber of Samples: %1\nDwell Time: %2 ns\nDelay: %3 us\nFrequency Offset: %4 Hz\nPhase Offset: %5 rad\n")
			.arg(adc.numSamples)
			.arg(adc.dwellTime)
			.arg(adc.delay)
			.arg(adc.freqOffset)
			.arg(adc.phaseOffset);
	}

	std::array<QString, 3> gradChannels = { "Gx", "Gy", "Gz" };
	for (int channel = 0; channel < 3; ++channel)
	{
		if (pSeqBlock->isTrapGradient(channel) || pSeqBlock->isArbitraryGradient(channel) || pSeqBlock->isExtTrapGradient(channel))
		{
			blockInfo += QString("|-----------------------------------------------------------------------------------------------|\n");
			const GradEvent& grad = pSeqBlock->GetGradEvent(channel);
			blockInfo += QString("Gradient Event (Channel %1):\nAmplitude: %2 Hz/m\nDelay: %3 us")
				.arg(gradChannels[channel])
				.arg(grad.amplitude)
				.arg(grad.delay);

			if (pSeqBlock->isTrapGradient(channel))
			{
				blockInfo += QString("\nRamp Up Time: %1 us\nFlat Time: %2 us\nRamp Down Time: %3 us")
					.arg(grad.rampUpTime)
					.arg(grad.flatTime)
					.arg(grad.rampDownTime);
			}
			else if (pSeqBlock->isArbitraryGradient(channel))
			{
				blockInfo += QString("\nWave Shape ID: %1\nTime Shape ID: %2")
					.arg(grad.waveShape)
					.arg(grad.timeShape);
			}

			blockInfo += QString("\n");
		}
	}

	blockInfo += QString("\\-----------------------------------------------------------------------------------------------\\");
	m_pBlockInfoDialog->setInfoContent(blockInfo);
}