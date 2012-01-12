#ifndef _FEATURE_H_
#define _FEATURE_H_

#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;
using std::string;

class FeatureMatcher
{
public:
	FeatureMatcher(string detectorType = "SURF", string extractorType="SURF", string matcherType="FlannBased"):
	  _detectorType(detectorType),_extractorType(extractorType),_matcherType(matcherType){
		  _detector = FeatureDetector::create(detectorType);
		  _extractor = DescriptorExtractor::create(extractorType);
		  _matcher = DescriptorMatcher::create(matcherType);
	  }

	  bool setObjectImage(const Mat& src)
	  {
		  _img1 = src;
		  return _calculate(src, keypoints1, descriptors1);
	  }

	  bool setSceneImage(const Mat& src)
	  {
		  _img2 = src;
		  if (!_calculate(src, keypoints2, descriptors2))
			  return false;
		  _matcher->match( descriptors1, descriptors2, matches);
#if 1
		  max_dist = 0; 
		  min_dist = 100;

		  //-- Quick calculation of max and min distances between keypoints
		  for( int i = 0; i < descriptors1.rows; i++ )
		  { 
			  double dist = matches[i].distance;
			  if( dist < min_dist ) min_dist = dist;
			  if( dist > max_dist ) max_dist = dist;
		  }
#endif
		  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
		  //-- PS.- radiusMatch can also be used here.
		  good_matches.clear();

		  for( int i = 0; i < descriptors1.rows; i++ )
		  { 
			  float dist = matches[i].distance;
			  if( dist >= min_dist*k_min*0.01 && dist <= max_dist*k_max*0.01)
			  {
				  good_matches.push_back( matches[i]);
			  }
		  }
		  return true;
	  }

	  Mat findHomography(int ransacThresh = 3)
	  {
		  vector<Point2f> pt1,pt2;

		  int n_matches = good_matches.size();
		  for(int i = 0; i < n_matches; i++){
			  pt1.push_back(keypoints1[good_matches[i].queryIdx].pt);
			  pt2.push_back(keypoints2[good_matches[i].trainIdx].pt);
		  }
		  return cv::findHomography(pt1, pt2, Mat(), CV_RANSAC, ransacThresh);
	  }

	  void draw(Mat& match_img)
	  {
		  drawMatches(_img1, keypoints1, _img2, keypoints2, good_matches, match_img, Scalar::all(-1), Scalar::all(-1), 
			  vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	  }

private:
	bool _calculate(const Mat& src, vector<KeyPoint>& pts, Mat& descr)
	{
		_detector->detect(src, pts);
		_extractor->compute(src, pts, descr);

		return pts.size() > 0;
	}

	string _detectorType;
	string _extractorType;
	string _matcherType;

	Ptr<FeatureDetector> _detector;
	Ptr<DescriptorExtractor> _extractor;
	Ptr<DescriptorMatcher> _matcher;

	vector<KeyPoint> keypoints1,keypoints2;
	Mat descriptors1, descriptors2;
	vector<DMatch> matches;	
	vector<DMatch> good_matches;

	double max_dist, min_dist;

	Mat _img1,_img2;
};

#endif
