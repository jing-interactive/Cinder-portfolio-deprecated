//  ---------------------------------------------------------------------------
//
//  @file       TwFonts.h
//  @brief      Bitmaps fonts
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see docs/AntTweakBar/License.txt
//
//  notes:      Private header
//              TAB=4
//
//  ---------------------------------------------------------------------------


#if !defined ANT_TW_FONTS_INCLUDED
#define ANT_TW_FONTS_INCLUDED

//#include <AntTweakBar.h>

/*
A source bitmap includes 224 characters starting from ascii char 32 (i.e. space) to ascii char 255:
  
 !"#$%&'()*+,-./0123456789:;<=>?
@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_
`abcdefghijklmnopqrstuvwxyz{|}~
€亗儎厗噲墛媽崕彁憭摂晼棙櫄洔潪?
牎ⅲぅΗī氨渤吹斗腹夯冀究
懒旅呐魄壬仕掏蜗醒矣哉肿刭谯茌捱
噌忏溴骁栝觌祉铒瘃蝮趱鲼?

First column of a source bitmap is a delimiter with color=zero at the end of each line of characters.
Last row of a line of characters is a delimiter with color=zero at the last pixel of each character.

*/


struct CTexFont
{
    unsigned char * m_TexBytes;
    int             m_TexWidth;     // power of 2
    int             m_TexHeight;    // power of 2
    float           m_CharU0[256];
    float           m_CharV0[256];
    float           m_CharU1[256];
    float           m_CharV1[256];
    int             m_CharWidth[256];
    int             m_CharHeight;
    int             m_NbCharRead;

    CTexFont();
    ~CTexFont();
};


CTexFont *TwGenerateFont(const unsigned char *_Bitmap, int _BmWidth, int _BmHeight);


extern CTexFont *g_DefaultSmallFont;
extern CTexFont *g_DefaultNormalFont;
extern CTexFont *g_DefaultLargeFont;

void TwGenerateDefaultFonts();
void TwDeleteDefaultFonts();


#endif  // !defined ANT_TW_FONTS_INCLUDED
