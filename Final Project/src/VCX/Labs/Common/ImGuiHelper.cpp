#include<fstream>
#include<algorithm>
#include<vector>
#include<cstring>
#include <stb_image_write.h>

#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Common::ImGuiHelper {
    void TextCentered(std::string_view const text) {
        auto const textWidth   = ImGui::CalcTextSize(text.data()).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - textWidth * .5f);
        ImGui::Text(text.data());
    }

    void TextRight(std::string_view const text) {
        auto const textWidth   = ImGui::CalcTextSize(text.data()).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - textWidth);
        ImGui::Text(text.data());
    }

    void ZoomTooltip(
        Engine::GL::UniqueTexture2D const &     tex,
        std::pair<std::uint32_t, std::uint32_t> texSize,
        ImVec2 const &                          pos,
        bool const                              flipped,
        float const                             regionSize,
        float const                             zoomLevel) {
        auto const [width, height] = texSize;
        ImGui::BeginTooltip();
        float regionX = std::clamp(pos.x - regionSize * .5f, 0.f, width - regionSize);
        float regionY = std::clamp(pos.y - regionSize * .5f, 0.f, height - regionSize);
        ImGui::Text("Min: (%.0f, %.0f)", regionX, regionY);
        ImGui::Text("Max: (%.0f, %.0f)", regionX + regionSize, regionY + regionSize);
        ImVec2 uv0 = ImVec2(regionX / width, flipped ? 1 - regionY / height : regionY / height);
        ImVec2 uv1 = ImVec2((regionX + regionSize) / width, flipped ? 1 - (regionY + regionSize) / height : (regionY + regionSize) / height);
        ImGui::Image(
            reinterpret_cast<void *>(std::uintptr_t(tex.Get())),
            ImVec2(regionSize * zoomLevel, regionSize * zoomLevel),
            uv0,
            uv1,
            { 1, 1, 1, 1 },
            { 1, 1, 1, .5f });
        ImGui::EndTooltip();
    }

    void SaveImage(
        Engine::GL::UniqueTexture2D const &     tex,
        std::pair<std::uint32_t, std::uint32_t> texSize,
        bool const                              flipped) {
        static char path[128]   = "a.png";
        bool        enableWrite = ImGui::Button("Save PNG Image");
        static bool saving      = true;
        ImGui::SameLine();
        ImGui::InputText("", path, IM_ARRAYSIZE(path));

        if (enableWrite) {
            saving = true;
            gl_using(tex);
            char * rawImg = new char[sizeof(char) * texSize.first * texSize.second * 3];
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, rawImg);
            glPixelStorei(GL_PACK_ALIGNMENT, 4);
            stbi_flip_vertically_on_write(flipped);
            stbi_write_png(path, texSize.first, texSize.second, 3, rawImg, 3 * texSize.first);
            delete[] rawImg;
            ImGui::OpenPopup("Saved");
        }

        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        if (ImGui::BeginPopupModal("Saved", &saving, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Image saved to %s", path);
            ImGui::EndPopup();
        }
    }

    bool isLegal(std::string s){
        for(char c: s){
            if(!(c>=32 && c<0x80))return false;
        }
        return true;
    }

    bool AddNewWordAndWeight(char * word, int * weight) {
        static char newword[20] = "abc";
        static int newweight = 1;
        bool AddOrNot = ImGui::Button("Add New Word");
        static bool adding = true, error1 = true, error2 = true;
        bool added = false;
        ImGui::InputText("new word", newword, IM_ARRAYSIZE(newword));
        ImGui::InputInt("frequency",&newweight);

        if (AddOrNot) {
            if(!isLegal(newword)){//检查是否有非法字符
                error1 = true;
                ImGui::OpenPopup("Error1");
            }
            else if(newweight <= 0){//权重须为正值
                error2 = true;
                ImGui::OpenPopup("Error2");
            }
            else{
                adding = true;
                added = true;
                strcpy(word, newword);
                *weight = newweight;
                ImGui::OpenPopup("Added");
            }
        }
        
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        if(ImGui::BeginPopupModal("Error1", &error1, ImGuiWindowFlags_AlwaysAutoResize)){
            ImGui::Text("%s", "Illegal character !");
            ImGui::EndPopup();
        }
        else if(ImGui::BeginPopupModal("Error2", &error2, ImGuiWindowFlags_AlwaysAutoResize)){
            ImGui::Text("%s", "Frequency should be a positive value !");
            ImGui::EndPopup();
        }
        else if(ImGui::BeginPopupModal("Added", &adding, ImGuiWindowFlags_AlwaysAutoResize)){
            ImGui::Text("Word %s Added With Frequency %d !", newword, newweight);
            ImGui::EndPopup();
        }

        return added;
    }

    bool add_from_file(std::unordered_map<std::string,int> & data){
        static char mypath[50] = "assets/misc/wordcloud.txt";
        static bool op = true;
        bool opened = false;

        bool OpenFile = ImGui::Button("Add From File");
        ImGui::SameLine();
        ImGui::InputText(" ", mypath, IM_ARRAYSIZE(mypath));

        if(OpenFile){
            std::ifstream file(mypath);
            if(!file){
                op = true;
                ImVec2 center = ImGui::GetMainViewport()->GetCenter();
                ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
                ImGui::OpenPopup("Failed");
                if(ImGui::BeginPopupModal("Failed", &op, ImGuiWindowFlags_AlwaysAutoResize)){
                    ImGui::Text("%s", "Fail to open the file !");
                    ImGui::EndPopup();
                }
                return false;
            }
            opened = true;
            std::string s;
            int total = 0, fre = 0;//total为总数
            file >> total;
            for(int i = 0; i < total; ++i){
                file >> s >> fre;
                if(isLegal(s) && fre>0)data[s] = fre;
            }

            file.close();
        }
        
        return opened;
    }

} // namespace VCX::Labs::Common::ImGuiHelper
