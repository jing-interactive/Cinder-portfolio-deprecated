#include "Config.h"
#include "Cinder/Filesystem.h"
#include "Cinder/app/App.h"
#include "cinder/Xml.h"

extern float mRandomColorIndex;

AnimConfig::AnimConfig()
{
    lightValue = 0.5f;
    lightValue2 = 0;
    loopCount = 1;
}

Color AnimConfig::getColor() const
{
    float value = 0;
    if (lightValue2 != 0.0f)
    {
        float lv2 = lightValue2;
        float lv = lightValue;
        if (lv2 < lv)
        {
            std::swap<float>(lv, lv2);
        }
        const float range = math<float>::max(lv2 - lv, 0.001f);
        int k = mRandomColorIndex / range;
        value = (mRandomColorIndex - k * range) * range;
        if (k % 2 == 0)
        {
            value = lv + value;
        }
        else
        {
            value = lv2 - value;
        }
    }
    else
    {
        value = lightValue;
    }
    return Color(0.0f, value, 1.0f - value);
}

ostream& operator<<(ostream& lhs, const AnimConfig& rhs)
{
    lhs << rhs.lightValue << " " << rhs.lightValue2 << " " << rhs.loopCount;
    return lhs;
}

istream& operator>>(istream& lhs, AnimConfig& rhs)
{
    lhs >> rhs.lightValue >> std::ws >> rhs.lightValue2 >> std::ws >> rhs.loopCount;
    return lhs;
}


ostream& operator<<(ostream& lhs, const Config& rhs)
{
    for (int i=0; i<AnimConfig::kCount; i++)
    {
        lhs << rhs.animConfigs[i] << " ";
    }
    return lhs;
}

istream& operator>>(istream& lhs, Config& rhs)
{
    for (int i=0; i<AnimConfig::kCount; i++)
    {
        lhs >> rhs.animConfigs[i] >> std::ws;
    }
    return lhs;
}

Config mConfigs[Config::kCount];
int mConfigIds[kHourCount];

static const string kProgSettingFileName = "ProgramSettings.xml";

void readProgramSettings()
{
    fs::path configPath = app::getAssetPath("") / kProgSettingFileName;
    try
    {
        XmlTree tree(loadFile(configPath));
        for (int i=0; i<Config::kCount; i++)
        {
            mConfigs[i] = tree.getChild(toString(i)).getValue<Config>();
        }

        string str = tree.getChild("ids").getValue();
        for (int i=0; i<kHourCount; i++)
        {
            mConfigIds[i] = tree.getChild("ids").getChild(toString(i)).getValue<int>();
        }
    }
    catch (exception& e)
    {
        writeProgramSettings();
    }
}

void writeProgramSettings()
{
    XmlTree tree = XmlTree::createDoc();
    for (int i=0; i<Config::kCount; i++)
    {
        XmlTree item(toString(i), toString(mConfigs[i]));
        tree.push_back(item);
    }
    XmlTree ids("ids", "");
    for (int i=0; i<kHourCount; i++)
    {
        ids.push_back(XmlTree(toString(i), toString(mConfigIds[i])));
    }
    tree.push_back(ids);

    fs::path configPath = app::getAssetPath("") / kProgSettingFileName;
    tree.write(writeFile(configPath));
}
