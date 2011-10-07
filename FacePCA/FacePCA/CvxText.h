//====================================================================
//====================================================================
//
// 文件: CvxText.h
//
// 说明: OpenCV汉字输出
//
// 时间:
//
// 作者: 柴树杉
//       chaishushan#gmail.com
//       chaishushan.googlepages.com
//
//====================================================================
//====================================================================

#ifndef OPENCV_CVX_TEXT_2007_08_31_H
#define OPENCV_CVX_TEXT_2007_08_31_H

/**
* \file CvxText.h
* \brief OpenCV汉字输出接口
*
* 实现了汉字输出功能。
*/

#include <ft2build.h>
#include FT_FREETYPE_H

#include <cv.h>
#include <highgui.h>

/**
* \class CvxText
* \brief OpenCV中输出汉字
*
* OpenCV中输出汉字。字库提取采用了开源的FreeFype库。由于FreeFype是
* GPL版权发布的库，和OpenCV版权并不一致，因此目前还没有合并到OpenCV
* 扩展库中。
*
* 显示汉字的时候需要一个汉字字库文件，字库文件系统一般都自带了。
* 这里采用的是一个开源的字库：“文泉驿正黑体”。
*
* 关于"OpenCV扩展库"的细节请访问
* http://code.google.com/p/opencv-extension-library/
*
* 关于FreeType的细节请访问
* http://www.freetype.org/
*
* 例子：
*
* \code
int main(int argc, char *argv[])
{
   // 打开一个影象

   IplImage *img = cvLoadImage("test.jpg", 1);

   // 输出汉字

   {
      // "wqy-zenhei.ttf"为文泉驿正黑体

      CvxText text("wqy-zenhei.ttf");

      const char *msg = "在OpenCV中输出汉字！";

      float p = 0.5;
      text.setFont(NULL, NULL, NULL, &p);   // 透明处理

      text.putText(img, msg, cvPoint(100, 150), CV_RGB(255,0,0));
   }
}
* \endcode
*/

class CvxText 
{
   // 禁止copy

   CvxText& operator=(const CvxText&);

   //================================================================
   //================================================================

public:

   /**
    * 装载字库文件
    */

   CvxText(const char *freeType);
   virtual ~CvxText();

   //================================================================
   //================================================================

   /**
    * 获取字体。目前有些参数尚不支持。
    *
    * \param font        字体类型, 目前不支持
    * \param size        字体大小/空白比例/间隔比例/旋转角度
    * \param underline   下画线
    * \param diaphaneity 透明度
    *
    * \sa setFont, restoreFont
    */

   void getFont(int *type,
      CvScalar *size=NULL, bool *underline=NULL, float *diaphaneity=NULL);

   /**
    * 设置字体。目前有些参数尚不支持。
    *
    * \param font        字体类型, 目前不支持
    * \param size        字体大小/空白比例/间隔比例/旋转角度
    * \param underline   下画线
    * \param diaphaneity 透明度
    *
    * \sa getFont, restoreFont
    */

   void setFont(int *type,
      CvScalar *size=NULL, bool *underline=NULL, float *diaphaneity=NULL);

   /**
    * 恢复原始的字体设置。
    *
    * \sa getFont, setFont
    */

   void restoreFont();

   //================================================================
   //================================================================

   /**
    * 输出汉字(颜色默认为黑色)。遇到不能输出的字符将停止。
    *
    * \param img  输出的影象
    * \param text 文本内容
    * \param pos  文本位置
    *
    * \return 返回成功输出的字符长度，失败返回-1。
    */

   int putText(IplImage *img, const char    *text, CvPoint pos);

   /**
    * 输出汉字(颜色默认为黑色)。遇到不能输出的字符将停止。
    *
    * \param img  输出的影象
    * \param text 文本内容
    * \param pos  文本位置
    *
    * \return 返回成功输出的字符长度，失败返回-1。
    */

   int putText(IplImage *img, const wchar_t *text, CvPoint pos);

   /**
    * 输出汉字。遇到不能输出的字符将停止。
    *
    * \param img   输出的影象
    * \param text  文本内容
    * \param pos   文本位置
    * \param color 文本颜色
    *
    * \return 返回成功输出的字符长度，失败返回-1。
    */

   int putText(IplImage *img, const char    *text, CvPoint pos, CvScalar color);

   /**
    * 输出汉字。遇到不能输出的字符将停止。
    *
    * \param img   输出的影象
    * \param text  文本内容
    * \param pos   文本位置
    * \param color 文本颜色
    *
    * \return 返回成功输出的字符长度，失败返回-1。
    */
   int putText(IplImage *img, const wchar_t *text, CvPoint pos, CvScalar color);

   //================================================================
   //================================================================

private:

   // 输出当前字符, 更新m_pos位置

   void putWChar(IplImage *img, wchar_t wc, CvPoint &pos, CvScalar color);

   //================================================================
   //================================================================

private:

   FT_Library   m_library;   // 字库
   FT_Face      m_face;      // 字体

   //================================================================
   //================================================================

   // 默认的字体输出参数

   int         m_fontType;
   CvScalar   m_fontSize;
   bool      m_fontUnderline;
   float      m_fontDiaphaneity;

   //================================================================
   //================================================================
};

#endif // OPENCV_CVX_TEXT_2007_08_31_H

