#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

const char kImageWindow[] = "Source Image";
const char kResultWindow[] = "Result window";

int match_method;
const int kMaxTrackbar = 5;

cv::Point MatchingMethod( cv::Mat img, cv::Mat templ, int match_method = cv::TM_SQDIFF );

static void helpAndExit()
{
    printf("\nThis program extracts coordinates from UI layout images\n"
        "Using cv::matchTemplate AND cv::minMaxLoc:\n"
        "\n"
        "Usage:\n LayoutHacker <image_of_background> <image_of_button>\n");

    exit(-1);
}

int main( int argc, char** argv )
{
    if (argc < 3)
    {
        helpAndExit();
    }

    cv::Mat background = cv::imread( argv[1], cv::IMREAD_COLOR );
    cv::Mat templ = cv::imread( argv[2], cv::IMREAD_COLOR );

    if (background.empty() || templ.empty())
    {
        helpAndExit();
    }

    /// Create Trackbar
    //   char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
    //   createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );

    //enum { TM_SQDIFF=0, TM_SQDIFF_NORMED=1, TM_CCORR=2, TM_CCORR_NORMED=3, TM_CCOEFF=4, TM_CCOEFF_NORMED=5 };
    cv::Point foundCoord = MatchingMethod(background, templ, cv::TM_SQDIFF);
    printf("%d, %d\n", 
        foundCoord.x, foundCoord.y);

    cv::waitKey(0);

    return 0;
}

//TODO: fail to find?
//TODO: cv::namedWindow should provide an option

cv::Point MatchingMethod( cv::Mat img, cv::Mat templ, int match_method/* = cv::TM_SQDIFF */)
{
    /// Create windows
    cv::namedWindow( kImageWindow, cv::WINDOW_AUTOSIZE );
    cv::namedWindow( kResultWindow, cv::WINDOW_AUTOSIZE );

    /// Source image to display
    cv::Mat img_display;
    img.copyTo( img_display );

    /// Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    cv::Mat result( result_cols, result_rows, CV_32FC1 );

    /// Do the Matching and Normalize
    cv::matchTemplate( img, templ, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }

    /// Show me what you got
    cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
    cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

    cv::imshow( kImageWindow, img_display );
    cv::imshow( kResultWindow, result );

    return matchLoc;
}