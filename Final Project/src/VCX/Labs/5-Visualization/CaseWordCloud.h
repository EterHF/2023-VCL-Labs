#pragma once

#include<vector>
#include<cstring>
#include<unordered_map>
#include "Engine/Async.hpp"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/ICase.h"
#include "Labs/5-Visualization/Data.h"

namespace VCX::Labs::Visualization {

	class CaseWordCloud : public Common::ICase {
    public:
        CaseWordCloud();

        virtual std::string_view const GetName() override { return "Word Cloud"; }

        virtual void                        OnSetupPropsUI() override;
        virtual Common::CaseRenderResult    OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                        OnProcessInput(ImVec2 const & pos) override;
        char *                              _newword;
        int                                 _newweight;
        int                                 _shape;
        float                               _density;
        std::unordered_map<std::string,int> mp;
        std::string                         _shape_names[3]{"Square", "Circle", "Heart"};

    private:
        Engine::GL::UniqueTexture2D      _texture;
        Common::ImageRGB                 _empty;
        InteractProxy                    _proxy;
        bool                             _msaa;

        bool                            _recompute;

    };

}