#include "Labs/5-Visualization/tasks.h"
#include "Labs/5-Visualization/Data.h"

#include <numbers>
#include<cstring>
#include<algorithm>
#include<vector>
#include<unordered_map>

using VCX::Labs::Common::ImageRGB;
namespace VCX::Labs::Visualization {

    float standard_size = 0.2f;//权重最大词块的大小
    float step = 0.0075f, distance = 0.005f;//step控制采样点间距,distance控制螺线间距

    //使用极坐标变换,获取阿基米德螺线上的点坐标
    glm::vec2 archimedeanSpiral(float x, float y, float t){
        float factor = x / y;
        return glm::vec2{factor*(distance*t*step)*glm::cos(t*step)+0.5,(distance*t*step)*glm::sin(t*step)+0.5};
    }
    //使用二次函数获取词块大小
    // float get_size(float t){
    //     return standard_size*(1-(t-1)*(t-1));
    // }
    //检查是否落在圆形区域
    bool boundary_circle(int size_x, int size_y, int x, int y){
        int ori_x = static_cast<int>(std::floor(0.5f * (size_x - 1))), ori_y = static_cast<int>(std::floor(0.5f * (size_y - 1)));
        return (x-ori_x)*(x-ori_x)+(y-ori_y)*(y-ori_y) <= size_x*size_x/4+5;//默认 size_x == size_y,允许一定误差
    }
    //检查是否落在心形区域
    bool boundary_heart(int size_x, int size_y, int x, int y){
        float xx = -(float)x / size_x + 0.5f, yy = -(float)y / size_y + 0.5f;
        return xx*xx + yy*yy - std::abs(xx)*yy < 0.2f;
    }
    //权重比较规则,大者优先
    bool cmp(const std::pair<std::string, int> & a, const std::pair<std::string, int> & b){
        return a.second > b.second;
    }
    //随机颜色
    glm::vec3 rand_color(){
        float f1 = (std::rand()+1)%1000/(float)1000, f2 = (std::rand()+1)%1000/(float)1000, f3 = (std::rand()+1)%1000/(float)1000;
        return {f1, f2, f3};
        // if(std::rand()%2)return {0.8,0.1,0.5};
        // else return {0.0,0.0,1.0}; //双色
    }   

    //绘制词块;若check为true,则做一次碰撞检查,逐像素比较
    bool printText(Common::ImageRGB & canvas, glm::vec4 color, glm::vec2 pos, float lineHeight, std::string const & caption, bool check, int sh) {
        lineHeight           = lineHeight * (canvas.GetSizeY() - 1);
        auto &         io    = ImGui::GetIO();
        std::uint8_t * tex   = nullptr;
        int            w     = 0;
        int            h     = 0;
        float          x     = std::floor(pos.x * (canvas.GetSizeX() - 1));
        float          y     = std::floor(pos.y * (canvas.GetSizeY() - 1)) - lineHeight * 0.5f;
        float          scale = lineHeight * 0.05f;
        io.Fonts->GetTexDataAsAlpha8(&tex, &w, &h, nullptr);
        for (std::size_t s = 0; s < caption.length(); ++s) {
            char c = caption[s];
            if (! (c >= 32 && c < 0x80)) return false;
            ImFontGlyph const * glyph = io.Fonts->Fonts[0]->FindGlyph(c);
            x -= scale * glyph->AdvanceX * 0.5f;
        }
        for (std::size_t s = 0; s < caption.length(); ++s) {
            char c = caption[s];
            ImFontGlyph const * glyph = io.Fonts->Fonts[0]->FindGlyph(c);
            if (glyph == NULL) return false;
            if (glyph->Visible) {
                float dU        = (glyph->U1 - glyph->U0) / ((glyph->X1 - glyph->X0) * scale);
                float dV        = (glyph->V1 - glyph->V0) / ((glyph->Y1 - glyph->Y0) * scale);
                float x1        = x + glyph->X0 * scale;
                float x2        = x + glyph->X1 * scale;
                float y1        = y + glyph->Y0 * scale;
                float y2        = y + glyph->Y1 * scale;
                for (int xx = static_cast<int>(std::ceil(x1)); xx < x2; ++xx) {
                    for (int yy = static_cast<int>(std::ceil(y1)); yy < y2; ++yy) {
                        //检查是否落在范围内
                        bool in;
                        if(sh==0)in = static_cast<std::size_t>(yy) < canvas.GetSizeY() - 1 && static_cast<std::size_t>(xx) < canvas.GetSizeX() - 1;
                        else if(sh==1)in = boundary_circle(canvas.GetSizeX(), canvas.GetSizeY(), static_cast<int>(xx), static_cast<int>(yy));
                        else if(sh==2)in = boundary_heart(canvas.GetSizeX(), canvas.GetSizeY(), static_cast<int>(xx), static_cast<int>(yy));
                        if (in) {
                            if(!check){
                                float     u      = (glyph->U0 + (xx - x1) * dU) * (w - 1);
                                float     v      = (glyph->V0 + (yy - y1) * dV) * (h - 1);
                                int       u0     = static_cast<int>(std::floor(u));
                                int       v0     = static_cast<int>(std::floor(v));
                                u                = u - u0;
                                v                = v - v0;
                                glm::vec4 sp     = glm::vec4(tex[v0 * w + u0], tex[v0 * w + u0 + 1], tex[(v0 + 1) * w + u0], tex[(v0 + 1) * w + u0 + 1]);
                                glm::vec4 ws     = glm::vec4((1 - u) * (1 - v), u * (1 - v), v * (1 - u), u * v);
                                float     weight = glm::dot(sp, ws) * color.a / 255.f;
                                //printf("%f\n",weight);
                                auto &&   proxy  = canvas.At(xx, yy);
                                proxy            = glm::vec3(color) * weight + static_cast<glm::vec3>(proxy) * (1.0f - weight);
                            }
                            else{
                                if((glm::vec3)canvas.At(xx, yy)!=glm::vec3{1.0f,1.0f,1.0f})return false;//颜色不为背景白色,说明该像素被占用
                            }
                        }
                        else return false;
                    }
                }
            }
            x += scale * glyph->AdvanceX;
        }
        return true;
    }

    //主函数
    bool DrawWordCloud(Common::ImageRGB & input, std::unordered_map<std::string,int> & m, int shape, float density, bool force){
        if(m.size()==0)return true;
        if(!force)return false;//避免反复绘制
        std::srand(std::time(NULL));
        //auto copy = input;//备份,用于重新绘制

        //printf("%d\n",m.size());
        standard_size = 0.15f;
        distance = density;
        int forward =  50;

        int size_x = input.GetSizeX(), size_y = input.GetSizeY();
        std::vector< std::pair<std::string, int> > data(m.begin(), m.end());
        int len = data.size();
        sort(data.begin(), data.end(), cmp);//按照权重大小排序，权重大的先绘制
        int max_weight = data[0].second;//按照权重比例设置词块大小

        //绘制阿基米德螺线
        // for(int i = 0; i <= 50000; i += forward){
        //     glm::vec2 pos1=archimedeanSpiral(size_x,size_y,i);
        //     glm::vec2 pos2=archimedeanSpiral(size_x,size_y,i+forward);
        //     DrawLine(input,glm::vec4{1.0,0.0,0.0,1.0},pos1,pos2,0.01);
        //     PrintText(input, {1.0,0.0,0.0,1.0}, pos1+glm::vec2{0.01,-0.01}, 0.02, std::to_string(i/forward+1));
        // }

        //绘制词云
        for(int i = 0; i < len; ++i){
            //float size_font = std::max(get_size((float)data[i].second / max_weight), 0.02f);
            float size_font = std::max(standard_size * (float)data[i].second / max_weight, 0.01f);
            //printf("%f\n",standard_size);
            bool found = false;
            glm::vec4 col= {rand_color(), 1.0};
            for(int k = 0; k <= 50000; k += forward){
                glm::vec2 pos=archimedeanSpiral(size_x, size_y, k);
                if(printText(input, col, pos, size_font, data[i].first, 1, shape)){
                    printText(input, col, pos, size_font, data[i].first, 0, shape);
                    found = true;
                    break;
                }
            }
            if(!found){
                standard_size -= 0.01f;//缩小,重新绘制
                i--;
            }
        }

        return true;
    }
}; // namespace VCX::Labs::Visualization