#pragma once

#include<vector>
#include<cstring>
#include<unordered_map>
#include "Labs/Common/ImageRGB.h"
#include "Labs/5-Visualization/Data.h"

namespace VCX::Labs::Visualization {
    glm::vec2 archimedeanSpiral(float x, float y, float t);
    float get_size(float t);
    bool isLegal(std::string s);
    bool cmp(const std::pair<std::string, int> & a, const std::pair<std::string, int> & b);
    bool printText(Common::ImageRGB & canvas, glm::vec4 color, glm::vec2 pos, float lineHeight, std::string const & caption, bool check, bool cir);
    bool DrawWordCloud(Common::ImageRGB & input, std::unordered_map<std::string,int> & m, int shape, float density, bool force);
};
