#include <fstream>
#include<iostream>
#include<unordered_map>
#include<cstring>
#include<direct.h>

#include "Labs/5-Visualization/CaseWordCloud.h"
#include "Labs/5-Visualization/tasks.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Labs/5-Visualization/Data.h"

namespace VCX::Labs::Visualization {
    static constexpr auto c_Size = std::pair(640U, 640U);

    CaseWordCloud::CaseWordCloud():
        _texture(
            Engine::GL::SamplerOptions {
                .MinFilter = Engine::GL::FilterMode::Linear,
                .MagFilter = Engine::GL::FilterMode::Nearest }),
        _empty(c_Size.first, c_Size.second),
        _msaa(false),
        _shape(0),
        _density(0.005f),
        _recompute(true){
            _newword = new char[20];
            _newweight = 1;
    }

    void CaseWordCloud::OnSetupPropsUI() {
        _recompute |= ImGui::Checkbox("Anti-Aliasing", &_msaa);
        Common::ImGuiHelper::SaveImage(_texture, {c_Size.first * (_msaa ? 2 : 1), c_Size.second * (_msaa ? 2 : 1)}, false);
        //_recompute |= ImGui::Checkbox("Circle", &_circle);
        if(ImGui::BeginCombo("Shape", _shape_names[_shape].c_str())){//下拉选择形状
            for (int i = 0; i < 3; ++i) {
                bool selected = i == _shape;
                if (ImGui::Selectable(_shape_names[i].c_str(), selected)) {
                    if (! selected) {
                        _shape  = i;
                        _recompute = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        _recompute |= ImGui::SliderFloat("Distance", &_density, 0.005f, 0.015f);
        if(Common::ImGuiHelper::AddNewWordAndWeight(_newword, &_newweight)){
            std::string s = _newword;
            mp[s] = _newweight;//使用unordered_map,避免重复输入(权重的更新)
            _recompute = true;
        }
        _recompute |= Common::ImGuiHelper::add_from_file(mp);//从文件中读入数据
        if(ImGui::Button("Reset")){//清空
            mp.clear();
            _recompute = true;
            //printf("%d\n",mp.size());
        }
    }

    Common::CaseRenderResult CaseWordCloud::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        Common::ImageRGB result;
        if(_shape != 1)result = Common::CreatePureImageRGB(c_Size.first * (_msaa ? 2 : 1), c_Size.second * (_msaa ? 2 : 1), {1.,1.,1.});
        else{
            result = Common::ImageRGB(c_Size.first * (_msaa ? 2 : 1), c_Size.second * (_msaa ? 2 : 1));
            DrawFilledCircle(result, {1.,1.,1.,1.}, {.5,.5}, {.5});//圆形词云
        }
        
        if(DrawWordCloud(result, mp, _shape, _density, _recompute)){
            _recompute = false;
            _texture.Update(result);
        }

        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseWordCloud::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        auto &       io      = ImGui::GetIO();
        ImVec2 const delta   = io.MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        _proxy.Update(
            ImVec2(c_Size.first, c_Size.second),
            pos,
            delta,
            hovered,
            ImGui::IsMouseDown(ImGuiMouseButton_Left),
            ImGui::IsMouseDown(ImGuiMouseButton_Right));
    }
}