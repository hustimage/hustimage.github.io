#ifndef __TRACKER_H__
#define __TRACKER_H__

#include "opencv/cv.h"
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

using namespace std;
using namespace cv;

class Tracker {
public:
	///<summary>Initialization parameters.</summary>
	struct InitParams {
		///<summary>Set default parameter values.</summary>
		InitParams();

		int histDims;
		int vMin;
		int vMax;
		int sMin;
		int sBox;
		bool showBackproject;
		bool showControlsGUI;
		bool showHistogram;
		float histRanges[2];
	};

	static int g_selId;
	static bool g_selectObject;
	static int g_initTracking;
	static cv::Point g_selOrigin;
	static cv::Rect g_selRect;

	Tracker();
	~Tracker();

	void Init(const cv::Size &frameSize, const InitParams &initParams);
	// void Deinit();
	void InitTrackWindow(const cv::Mat &img, const cv::Rect &selRect);
	void ProcessFrame(const cv::Mat &img, cv::Mat &out);

	void ShowControlsGUI();
	void HideControlsGUI();
	bool ToggleShowBackproject();

	bool IsTracking() const;
	void StopTracking();

	const cv::Rect &TrackWindow() const;
	const cv::RotatedRect &TrackBox() const;
	const CvHistogram &TrackHistogram() const;
	int TrackHistogramDims() const;
	const cv::Mat Hue() const;
	const cv::Mat BackProjection() const;
	const std::string &Name() const;

private:
	static void OnMouse(int event, int x, int y, int flags, void *param);
	void PredictPos();
	void DrawStuff(cv::Mat &out);

	cv::KalmanFilter m_KF;

	cv::Mat m_state;
	cv::Mat m_measured;
	cv::Mat m_estimated;

	vector<Point> m_past, m_kalmanv;

	int m_id;
	cv::Size m_frameSize;

	Mat m_imgHue, m_imgMask, m_imgHSV, m_hist;
	Mat m_frame, m_imgBackproject;
	// Mat objmask;
	// CvHistogram* m_histTrackWnd;

	int m_histDims, m_hSize;
	int m_binWidth;
	float m_histRanges[2];
	Mat m_histImg;

	///<summary>Controls the amount of the new track window histogram to weight
	///into the current tracking histogram.</summary>
	int m_ageRatio;

	bool m_showBackproject;
	bool m_showControlsGUI;
	bool m_showHistogram;

	std::string m_controlsGUIWndName;
	std::string m_backprojectWndName;
	std::string m_histogramWndName;

	int m_tracking;
	bool m_initialized;

	cv::Rect m_trackWindow;
	cv::RotatedRect m_trackBox;

	cv::Rect m_trackCompRect;
	cv::Point m_trackPosTwoFramesBack;
	float m_trackAreaTwoFramesBack;
	cv::Scalar m_velocity;

	bool m_reaquire;

	// Camshift parameters.
	int m_vMin;
	int m_vMax;
	int m_sMin;
	int m_sBox;


};

#endif  //ADAPTIVE_HISTOGRAM_CAMSHIFT
