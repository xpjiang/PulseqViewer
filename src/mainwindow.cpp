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
	, m_dTotalDuration_us(0.)
	, m_dDragStartRange(0.)
{
	ui->setupUi(this);
	setAcceptDrops(true);
	Init();

	m_listRecentPulseqFilePaths.resize(10);

	m_pSelectionRect = new QCPItemRect(ui->customPlot);
	m_pSelectionRect->setVisible(false);
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
	//connect(ui->customPlot, &QCustomPlot::mousePress, this, &MainWindow::onMousePress);
	//connect(ui->customPlot, &QCustomPlot::mouseMove, this, &MainWindow::onMouseMove);
	//connect(ui->customPlot, &QCustomPlot::mouseRelease, this, &MainWindow::onMouseRelease);
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
		m_vecRects[i]->axis(QCPAxis::atLeft)->setRange(0, 10);
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
	m_vecSeqBlocks.resize(lSeqBlockNum);
	m_pProgressBar->show();
	int64_t progress(0);
	for (int64_t ushBlockIndex=0; ushBlockIndex < lSeqBlockNum; ushBlockIndex++)
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
	// 定义 7 种颜色 用于绘制不同的RF波形
	QVector<QColor> colors = {
		QColor::fromRgbF(0,0.447,0.741),
		QColor::fromRgbF(0.85,0.325,0.098),
		QColor::fromRgbF(0.929,0.694,0.125),
		QColor::fromRgbF(0.494,0.184,0.556),
		QColor::fromRgbF(0.466,0.674,0.188),
		QColor::fromRgbF(0.301,0.745,0.933),
		QColor::fromRgbF(0.635,0.078,0.184)
	};

	double t0 = 0;
	int cnt = 0;
	double min1 = std::numeric_limits<double>::max(), max1 = std::numeric_limits<double>::min();
	double min2 = std::numeric_limits<double>::max(), max2 = std::numeric_limits<double>::min();
	for (long long i = 0; i < m_spPulseqSeq->GetNumberOfBlocks(); i++)
	{

		SeqBlock* pSeqBlock = m_spPulseqSeq->GetBlock(i);
		m_spPulseqSeq->decodeBlock(pSeqBlock);
		if (pSeqBlock->isRF())
		{
			const RFEvent& rfEvent = pSeqBlock->GetRFEvent();
			const int& ushSamples = pSeqBlock->GetRFLength();
			const float& fDwell = pSeqBlock->GetRFDwellTime();
			const double& dDuration_us = ushSamples * fDwell;

			// t:每组RF有对应的block start time(依据之前的block算出来)，rf的delay，block的rfDwellTime_us
			// y_1:block的rfAmplitude[]和rf的amplitude
			// y_2:phase,angle(s.*sign(real(s))*exp(1i*rf.phaseOffset).*exp(1i*2*pi*t    *rf.freqOffset))
			double t = t0 + rfEvent.delay;
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
				timePoints[j] = (t + j * fDwell);
				amplitudes[j] = rfList[j] * rfEvent.amplitude * cos(phaseList[j]);

				std::complex<double> signal = std::polar(double(amplitudes[j]), double(phaseList[j] + rfEvent.phaseOffset + 2 * PI *j * fDwell * rfEvent.freqOffset / 1000000));
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

			// 设置散点样式为 "x"
			auto graph_RFADC_ph2 = ui->customPlot->addGraph(m_vecRects[2]->axis(QCPAxis::atBottom), m_vecRects[2]->axis(QCPAxis::atLeft));
			QCPScatterStyle scatterStyle(QCPScatterStyle::ssCross, 5);
			graph_RFADC_ph2->setScatterStyle(scatterStyle);
			graph_RFADC_ph2->setPen(QPen(Qt::blue));

			{
				// 调用 calcRfCenter 函数
				std::vector<double> signal(amplitudes.begin(), amplitudes.end());
				std::vector<double> t(timePoints.begin(), timePoints.end());
				auto [tc, ic] = calcRfCenter(signal, t);
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

		t0 += pSeqBlock->GetDuration();
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
	ui->customPlot->xAxis->setRange(0, t0);
	ui->customPlot->replot();
}

//void MainWindow::DrawRFWaveform(const double& dStartTime, double dEndTime)
//{
//	// @TODO: fix time range, add reset. fix right click
//	if (m_vecSeqBlocks.size() == 0) return;
//	if (m_vecRfLib.size() == 0) return;
//	if(dEndTime < 0) dEndTime = m_dTotalDuration_us;
//
//	double dMaxAmp(0.);
//	double dMinAmp(0.);
//	for(const auto& rfInfo : m_vecRfLib)
//	{
//		const auto& rf = rfInfo.event;
//		const std::vector<float>& vecAmp = m_mapShapeLib[rf->magShape];
//		const std::vector<float>& vecPhase = m_mapShapeLib[rf->phaseShape];
//
//		// 添加波形数据点
//		double signal(0.);
//		for(uint32_t index = 0; index < vecAmp.size(); index++)
//		{
//			const float& amp = vecAmp[index];
//			const float& phase = vecPhase[index];
//			signal = std::abs(std::polar(amp, phase)) * rfInfo.event->amplitude;
//			dMaxAmp = std::max(dMaxAmp, signal);
//			dMinAmp = std::min(dMinAmp, signal);
//		}
//	}
//	double margin = (dMaxAmp - dMinAmp) * 0.1;
//	m_pRfMagRect->axis(QCPAxis::atLeft)->setRange(dMinAmp - margin, dMaxAmp+ margin);
//	QCPRange newRange(dStartTime, dEndTime);
//	m_pRfMagRect->axis(QCPAxis::atBottom)->setRange(newRange);
//	m_pRfADCPhaseRect->axis(QCPAxis::atBottom)->setRange(newRange);
//
//	// 只处理时间范围内的RF
//	for(const auto& rfInfo : m_vecRfLib)
//	{
//		// 跳过范围外的RF
//		if(rfInfo.startAbsTime_us + rfInfo.duration_us < dStartTime) continue;
//		if(rfInfo.startAbsTime_us > dEndTime) break;
//
//		QCPGraph* rfGraph = ui->customPlot->addGraph(m_pRfMagRect->axis(QCPAxis::atBottom),
//			m_pRfMagRect->axis(QCPAxis::atLeft));
//		m_vecGraphs.append(rfGraph);
//
//		const auto& rf = rfInfo.event;
//		double sampleTime = rfInfo.startAbsTime_us;
//		const std::vector<float>& vecAmp = m_mapShapeLib[rf->magShape];
//		const std::vector<float>& vecPhase = m_mapShapeLib[rf->phaseShape];
//
//		QVector<double> timePoints(vecAmp.size(), 0.);
//		QVector<double> amplitudes(vecAmp.size(), 0.);
//
//		// 添加波形数据点
//		double signal(0.);
//		for(uint32_t index = 0; index < vecAmp.size(); index++)
//		{
//			const float& amp = vecAmp[index];
//			const float& phase = vecPhase[index];
//			signal = std::abs(std::polar(amp, phase)) * rfInfo.event->amplitude;
//			timePoints[index] = sampleTime;
//			amplitudes[index] = signal;
//			sampleTime += rfInfo.dwell;
//		}
//		rfGraph->setData(timePoints, amplitudes);
//		rfGraph->setPen(QPen(Qt::blue));
//		rfGraph->setSelectable(QCP::stWhole);
//	}
//
//	ui->customPlot->xAxis->setRange(dStartTime, dEndTime);
//	ui->customPlot->replot();
//}

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


